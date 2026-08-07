#!/usr/bin/env python3
"""
offboard_pd_controller.py — 级联 PD 位置/速度控制器（Python 版）
============================================================
与 C++ 版（src/offboard_pd_controller.cpp）逻辑完全一致：

  目标位置         位置误差              速度指令            速度误差         加速度指令      姿态+油门
  (x,y,z)  ──→ [P控制器] ──→ vel_des ──→ [PD控制器] ──→ accel_des ──→ [加速度→姿态] ──→ AttitudeTarget
                ↑ 外环                   ↑ 内环              ↑ 转换
            Kp_pos × 位置误差       Kp_vel × 速度误差    推力向量 → 四元数姿态
                                  + Kd_vel × 速度误差微分

与 offboard_position.py 的区别：
  位置节点只是告诉 PX4 "我要去 (x,y,z)"，如何飞完全由 PX4 内部控制。
  本节点自己写控制器，算出需要的姿态角和油门，直接发送给飞控执行，
  绕过 PX4 的位置环和速度环（但仍使用 PX4 内部姿态控制器）。

坐标系：ENU（East-North-Up）
  X 轴指向东、Y 轴指向北、Z 轴指向上，重力加速度 g 方向为 (0, 0, -9.81)

运行方式：rosrun offboard_control offboard_pd_controller.py
参数：可通过 rosparam 或 rosrun _Kp_pos:=0.8 等运行时覆盖
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

# ======================== 全局变量 ========================

current_state = State()          # 飞控当前状态（连接、模式、解锁）
current_pose = PoseStamped()     # 无人机当前位置 (ENU)
current_vel = TwistStamped()     # 无人机当前速度 (ENU)

# ======================== PD 控制器参数 ========================
# 通过 launch 文件或 rosrun _Kp_pos:=xxx 可动态调节

Kp_pos = 0.95       # 位置外环 P 增益：位置误差 → 期望速度（越大响应越快，过大振荡）
Kp_vel = 1.8        # 速度内环 P 增益：速度误差 → 加速度（比例部分）
Kd_vel = 0.4        # 速度内环 D 增益：抑制速度变化过快（微分部分，减振）
max_vel = 3.0       # 期望速度上限 [m/s]（防止飞太快）
max_accel = 5.0     # 期望加速度上限 [m/s²]（防止倾角过大）
max_tilt = 0.5      # 最大倾斜角 [rad] ≈ 28.6°（安全限制，防止翻倒）
hover_thrust = 0.5  # 悬停油门：水平且加速度为 0 时的归一化推力

# ======================== 回调函数 ========================

def state_cb(msg):
    """飞控状态回调

    从 /mavros/state 订阅，获得飞控的连接状态、当前飞行模式、是否解锁等
    用途：判断何时可以切换 offboard 模式和解锁
    """
    global current_state
    current_state = msg


def pose_cb(msg):
    """位置回调

    从 /mavros/local_position/pose 订阅，获取无人机在 ENU 坐标系下的 x, y, z 位置
    用途：计算位置误差 = 目标位置 - 当前位置
    """
    global current_pose
    current_pose = msg


def vel_cb(msg):
    """速度回调

    从 /mavros/local_position/velocity_local 订阅，获取无人机在 ENU 坐标系下的线速度
    用途：计算速度误差 = 期望速度 - 当前速度
    """
    global current_vel
    current_vel = msg


# ======================== 工具函数 ========================

def clamp(val, lo, hi):
    """限幅函数（Clamp）

    将 val 限制在 [lo, hi] 区间内，防止控制量过大导致不稳定
    """
    return max(lo, min(hi, val))


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角 → 四元数

    用半角公式将横滚/俯仰/偏航角转换为 (x, y, z, w) 四元数。
    这是 ROS 中描述姿态的标准方式（单位四元数）。

    @param roll   横滚角 [rad]
    @param pitch  俯仰角 [rad]
    @param yaw    偏航角 [rad]
    @return       (x, y, z, w) 四元数
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def accel_to_attitude(ax, ay, az, yaw):
    """加速度 → 姿态四元数 + 油门 转换（核心函数）

    给定"我想让飞机如何加速"，算出"飞机需要倾斜到什么角度、加多大油门"。

    物理原理：
      四旋翼只能产生沿机体 Z 轴（螺旋桨面法线方向）的推力。
      要水平移动，必须倾斜机体，让推力在水平方向产生分量。

    计算步骤：
      1. 期望推力方向（世界坐标系 ENU）：f = 期望加速度 + (0, 0, g)
         +g 是为了抵消重力（要悬停就必须产生向上的推力对抗重力）
      2. 归一化得到机体 Z 轴在世界中的方向 z_b
      3. 结合偏航角 yaw 反解横滚角 roll 与俯仰角 pitch：
           roll  = atan2(zb_x*sin(yaw) - zb_y*cos(yaw), zb_z)
           pitch = atan2(zb_x*cos(yaw) + zb_y*sin(yaw), zb_z)
      4. 欧拉角 → 四元数
      5. 油门 ∝ |f|：悬停时 f_norm = g，输出 hover_thrust

    @param ax   期望 x 方向加速度 [m/s²] (ENU)
    @param ay   期望 y 方向加速度 [m/s²] (ENU)
    @param az   期望 z 方向加速度 [m/s²] (ENU)
    @param yaw  期望偏航角 [rad]
    @return     ((qx, qy, qz, qw), thrust)
    """
    g = 9.81  # 重力加速度 [m/s²]

    # ========== 第 1 步：计算期望推力方向（世界坐标系 ENU）==========
    # 牛二：F = m*a。推力需要同时提供"抵消重力"和"产生加速度"两部分
    # fx/fy 不需要抵消重力（重力只在 Z 方向）；fz = az + g 向上加速需加大油门
    fx, fy, fz = ax, ay, az + g

    # ========== 第 2 步：归一化得到机体 Z 轴方向 ==========
    f_norm = math.sqrt(fx * fx + fy * fy + fz * fz)  # 推力向量长度
    if f_norm < 1e-6:
        # 推力为 0 的异常情况：保持水平（w=1），油门 0
        return (0.0, 0.0, 0.0, 1.0), 0.0

    zb_x, zb_y, zb_z = fx / f_norm, fy / f_norm, fz / f_norm
    # 悬停时：zb = (0, 0, 1)，即机体 Z 轴竖直向上 ✓

    # ========== 第 3 步：倾斜角限制（安全保护）==========
    # 倾斜角 = 机体 Z 轴与世界 Z 轴的夹角；超过上限则等比例缩小 X、Y 分量
    tilt = math.acos(clamp(zb_z, -1.0, 1.0))
    if tilt > max_tilt:
        scale = math.sin(max_tilt) / math.sin(tilt)
        zb_x *= scale
        zb_y *= scale
        zb_z = math.cos(max_tilt)
        renorm = math.sqrt(zb_x * zb_x + zb_y * zb_y + zb_z * zb_z)
        zb_x /= renorm
        zb_y /= renorm
        zb_z /= renorm

    # ========== 第 4 步：由 z_b 和 yaw 反解 roll / pitch ==========
    # 由旋转矩阵推导：
    #   zb_x = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)
    #   zb_y = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)
    #   zb_z = cos(pitch)*cos(roll)
    # 消元可得：
    roll = math.atan2(zb_x * math.sin(yaw) - zb_y * math.cos(yaw), zb_z)
    pitch = math.atan2(zb_x * math.cos(yaw) + zb_y * math.sin(yaw), zb_z)

    # ========== 第 5 步：欧拉角 → 四元数 ==========
    q = euler_to_quaternion(roll, pitch, yaw)

    # ========== 第 6 步：计算油门 ==========
    # 悬停时 f_norm = g，输出 hover_thrust；上升时加大，下降时减小
    thrust = clamp((f_norm / g) * hover_thrust, 0.05, 1.0)

    return q, thrust


# ======================== 主函数 ========================

def main():
    """主函数：初始化节点，进入 PD 控制循环

    流程：
      1. 等待飞控连接
      2. 预热 setpoint 流（发送初始姿态指令，让 PX4 准备好接收 offboard 指令）
      3. 切换到 OFFBOARD 模式 + 解锁
      4. 主循环：位置 P 环 → 速度 PD 环 → 加速度→姿态+油门 → 发布
    """
    global current_state, current_pose, current_vel
    global Kp_pos, Kp_vel, Kd_vel, max_vel, max_accel, max_tilt, hover_thrust

    rospy.init_node("offboard_pd_controller", anonymous=True)

    # ---- 从参数服务器加载 PD 参数（可被 launch 文件覆盖）----
    Kp_pos = rospy.get_param("~Kp_pos", 0.95)
    Kp_vel = rospy.get_param("~Kp_vel", 1.8)
    Kd_vel = rospy.get_param("~Kd_vel", 0.4)
    max_vel = rospy.get_param("~max_vel", 3.0)
    max_accel = rospy.get_param("~max_accel", 5.0)
    max_tilt = rospy.get_param("~max_tilt", 0.5)
    hover_thrust = rospy.get_param("~hover_thrust", 0.5)

    # ---- 订阅：飞控状态 / 当前位置 / 当前速度 ----
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, vel_cb)

    # ---- 发布：姿态 + 油门指令 ----
    # AttitudeTarget 消息包含：orientation（目标姿态四元数）、thrust（归一化油门）
    att_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget,
                              queue_size=10)

    # ---- 服务客户端 ----
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # 控制频率 50Hz（姿态控制比位置控制需要更快的更新频率）
    rate = rospy.Rate(50)

    # ===== 等待飞控连接 =====
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("飞控已连接")

    # ===== 构建 AttitudeTarget 消息模板 =====
    att_setpoint = AttitudeTarget()
    # type_mask 位掩码：忽略横滚/俯仰/偏航角速度，只发姿态 + 油门
    att_setpoint.type_mask = (
        AttitudeTarget.IGNORE_ROLL_RATE
        | AttitudeTarget.IGNORE_PITCH_RATE
        | AttitudeTarget.IGNORE_YAW_RATE
    )
    # 初始姿态：水平（w=1 的四元数即无旋转），油门 0
    att_setpoint.orientation.w = 1.0
    att_setpoint.thrust = 0.0

    # ===== 预热 setpoint 流（原因同 offboard_position.py）=====
    for _ in range(50):
        if rospy.is_shutdown():
            break
        att_setpoint.header.stamp = rospy.Time.now()
        att_pub.publish(att_setpoint)
        rate.sleep()
    rospy.loginfo("Setpoint 预热完成")

    # ===== 准备模式切换 / 解锁 =====
    last_request = rospy.Time.now()
    start_time = rospy.Time.now()

    # ===== 主控制循环 =====
    prev_evx, prev_evy, prev_evz = 0.0, 0.0, 0.0  # 上一时刻速度误差（用于微分项）

    while not rospy.is_shutdown():
        # ---- 模式切换与解锁（同 offboard_position.py）----
        if current_state.mode != "OFFBOARD" and \
                (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            resp = set_mode_client(0, "OFFBOARD")
            if resp.mode_sent:
                rospy.loginfo("OFFBOARD 模式已启用")
            last_request = rospy.Time.now()
        elif not current_state.armed and \
                (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            resp = arming_client(True)
            if resp.success:
                rospy.loginfo("无人机已解锁")
            last_request = rospy.Time.now()

        # ---- 轨迹规划：随时间改变目标位置 ----
        elapsed = (rospy.Time.now() - start_time).to_sec()

        if 10.0 < elapsed <= 20.0:
            setpoint = (5.0, 0.0, 3.0)
        elif 20.0 < elapsed <= 30.0:
            setpoint = (5.0, 5.0, 3.0)
        elif 30.0 < elapsed <= 40.0:
            setpoint = (0.0, 5.0, 3.0)
        elif elapsed > 40.0:
            setpoint = (0.0, 0.0, 2.0)
        else:
            setpoint = (0.0, 0.0, 2.0)
        des_x, des_y, des_z = setpoint

        # ===================================================================
        # 核心控制逻辑：级联 PD 控制器
        # ===================================================================

        # ---- 第 1 层：外环 P 控制（位置 → 期望速度）----
        # 误差 = 目标位置 - 当前位置，乘 Kp 得到期望速度
        # 物理含义：离目标越远，飞得越快（线性比例）
        ex = des_x - current_pose.pose.position.x
        ey = des_y - current_pose.pose.position.y
        ez = des_z - current_pose.pose.position.z

        # 期望速度 = Kp_pos × 位置误差（限幅防止过快）
        vel_des_x = clamp(Kp_pos * ex, -max_vel, max_vel)
        vel_des_y = clamp(Kp_pos * ey, -max_vel, max_vel)
        vel_des_z = clamp(Kp_pos * ez, -max_vel, max_vel)
        # 例：Kp_pos=0.95，误差=3m → 期望速度=2.85m/s
        # 如果误差很小（接近目标），期望速度也小，自然会减速

        # ---- 第 2 层：内环 PD 控制（速度 → 期望加速度）----
        # 速度误差 = 期望速度 - 当前速度
        evx = vel_des_x - current_vel.twist.linear.x
        evy = vel_des_y - current_vel.twist.linear.y
        evz = vel_des_z - current_vel.twist.linear.z

        # 速度误差的微分（用上一时刻的误差差分近似求导）
        # 微分项作用：预测速度变化趋势，提前抑制，减少超调和振荡
        # d(ev)/dt ≈ (当前误差 - 上次误差) × 控制频率
        d_evx = (evx - prev_evx) * 50.0   # ×50 = 除以 dt(dt=1/50s)
        d_evy = (evy - prev_evy) * 50.0
        d_evz = (evz - prev_evz) * 50.0
        prev_evx, prev_evy, prev_evz = evx, evy, evz

        # PD 控制律：a_des = Kp_vel × e_v + Kd_vel × d(e_v)/dt
        # P 项：速度误差越大，加速度越大（驱动飞机追赶期望速度）
        # D 项：速度误差变化越快，反向抑制（"刹车"作用，防止冲过头）
        ax = clamp(Kp_vel * evx + Kd_vel * d_evx, -max_accel, max_accel)
        ay = clamp(Kp_vel * evy + Kd_vel * d_evy, -max_accel, max_accel)
        az = clamp(Kp_vel * evz + Kd_vel * d_evz, -max_accel, max_accel)

        # ---- 第 3 层：加速度 → 姿态 + 油门 ----
        yaw = 0.0  # 偏航角固定 0（机头朝北）
        q, thrust = accel_to_attitude(ax, ay, az, yaw)

        # ---- 发布控制指令 ----
        att_setpoint.header.stamp = rospy.Time.now()
        att_setpoint.orientation.x = q[0]
        att_setpoint.orientation.y = q[1]
        att_setpoint.orientation.z = q[2]
        att_setpoint.orientation.w = q[3]
        att_setpoint.thrust = thrust
        att_pub.publish(att_setpoint)

        rate.sleep()  # 保持 50Hz


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
