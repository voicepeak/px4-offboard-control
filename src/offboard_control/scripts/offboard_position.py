#!/usr/bin/env python3
"""
offboard_position.py — PX4 Offboard 位置控制节点（Python 版）
================================================
功能：通过 MAVROS 向 PX4 飞控发送目标位置（x, y, z），由 PX4 内部控制器完成
      姿态和速度的闭环控制。这是最简单的 offboard 控制方式，PX4 官网示例风格。

与 C++ 版（src/offboard_position.cpp）逻辑完全一致：
  1. 等待飞控连接（FCU connection）
  2. 先持续发送 setpoint，让 PX4 收到足够多的位置指令（否则切 offboard 会失败）
  3. 切换到 OFFBOARD 模式
  4. 解锁（arm）
  5. 循环发送目标位置，无人机自动飞过去

发布话题：/mavros/setpoint_position/local  (geometry_msgs/PoseStamped)
订阅话题：/mavros/state                      (飞控状态，用于判断连接/模式/解锁)
          /mavros/local_position/pose         (当前位置，坐标系 ENU)

坐标系：MAVROS 默认使用 ENU（East-North-Up）
        - X 指向东，Y 指向北，Z 指向上
        - 起飞高度 2m 就是 z=2.0

运行方式：rosrun offboard_control offboard_position.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# 全局变量存储飞控当前状态（连接、模式、解锁等）
current_state = State()
# 全局变量存储无人机当前位置
current_pose = PoseStamped()


def state_cb(msg):
    """飞控状态回调

    每当 /mavros/state 话题有更新时，将最新状态存入全局变量 current_state
    后续用 current_state.connected / .mode / .armed 来判断飞控状态
    """
    global current_state
    current_state = msg


def pose_cb(msg):
    """位置回调

    每当 /mavros/local_position/pose 话题有更新时，存入 current_pose
    本节点虽然订阅了位置，但只是用来监控，控制逻辑不依赖它（交给 PX4 自己控制）
    """
    global current_pose
    current_pose = msg


def main():
    """主函数：初始化节点，等待连接，切 OFFBOARD 模式并解锁，循环发布目标位置"""
    global current_state, current_pose

    rospy.init_node("offboard_position", anonymous=True)

    # ---- 订阅飞控状态和位置 ----
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)

    # ---- 发布目标位置 ----
    # PX4 要求 offboard 模式下至少 2Hz 持续收到 setpoint，否则会自动切回 Position 模式
    setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped,
                                   queue_size=10)

    # ---- 服务客户端：用于解锁和切换飞行模式 ----
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # 控制频率 20Hz（PX4 offboard 最低要求 2Hz，建议 20~50Hz）
    rate = rospy.Rate(20)

    # ====================================================================
    # 第 1 步：等待飞控连接
    # 阻塞直到 MAVROS 与 PX4 SITL 建立连接（current_state.connected == True）
    # ====================================================================
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("飞控已连接，准备起飞")

    # ====================================================================
    # 第 2 步：在切 OFFBOARD 模式之前，先连续发送 setpoint
    # PX4 要求进入 offboard 前必须已收到一定数量的 setpoint 消息，
    # 否则模式切换会被拒绝。这里发 50 次（约 2.5 秒）。
    # ====================================================================
    setpoint = PoseStamped()
    # 初始目标位置：原点上方 2m，保持水平
    setpoint.pose.position.x = 0.0
    setpoint.pose.position.y = 0.0
    setpoint.pose.position.z = 2.0
    # 四元数 w=1 表示无旋转（水平姿态）
    setpoint.pose.orientation.w = 1.0

    for _ in range(50):
        if rospy.is_shutdown():
            break
        setpoint.header.stamp = rospy.Time.now()
        setpoint_pub.publish(setpoint)
        rate.sleep()
    rospy.loginfo("Setpoint 预热完成，准备切换 OFFBOARD 模式")

    # ====================================================================
    # 第 3 步：准备模式切换和解锁请求
    # ====================================================================
    # OFFBOARD 是 PX4 的自定义模式字符串
    offboard_mode = SetMode()
    offboard_mode.custom_mode = "OFFBOARD"

    # 记录上次请求时间，防止频繁重试；记录启动时间，用于轨迹计时
    last_request = rospy.Time.now()
    start_time = rospy.Time.now()

    # ====================================================================
    # 第 4 步：主控制循环
    # ====================================================================
    while not rospy.is_shutdown():
        # ---- 4a. 尝试切换到 OFFBOARD 模式 ----
        # 如果当前不在 OFFBOARD 模式，且距上次请求已过 5 秒，则发送切换请求
        if current_state.mode != "OFFBOARD" and \
                (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            resp = set_mode_client(0, offboard_mode.custom_mode)
            if resp.mode_sent:
                rospy.loginfo("OFFBOARD 模式已启用")
            last_request = rospy.Time.now()

        # ---- 4b. 尝试解锁 ----
        # 已进入 OFFBOARD 但尚未解锁时，发送解锁请求
        elif not current_state.armed and \
                (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            resp = arming_client(True)
            if resp.success:
                rospy.loginfo("无人机已解锁")
            last_request = rospy.Time.now()

        # ---- 4c. 轨迹规划：随时间变化的目标位置 ----
        elapsed = (rospy.Time.now() - start_time).to_sec()

        if 10.0 < elapsed <= 20.0:
            # 10~20 秒：飞往 (5, 0, 3) —— 向东 5 米，高度 3 米
            setpoint.pose.position.x = 5.0
            setpoint.pose.position.y = 0.0
            setpoint.pose.position.z = 3.0
        elif 20.0 < elapsed <= 30.0:
            # 20~30 秒：飞往 (5, 5, 3) —— 向东北角
            setpoint.pose.position.x = 5.0
            setpoint.pose.position.y = 5.0
            setpoint.pose.position.z = 3.0
        elif 30.0 < elapsed <= 40.0:
            # 30~40 秒：飞往 (0, 5, 3) —— 向北 5 米
            setpoint.pose.position.x = 0.0
            setpoint.pose.position.y = 5.0
            setpoint.pose.position.z = 3.0
        elif elapsed > 40.0:
            # 40 秒后：返回原点 (0, 0, 2)
            setpoint.pose.position.x = 0.0
            setpoint.pose.position.y = 0.0
            setpoint.pose.position.z = 2.0

        # ---- 4d. 发布目标位置 ----
        # 必须带时间戳，PX4 用时间戳判断 setpoint 是否新鲜
        setpoint.header.stamp = rospy.Time.now()
        setpoint_pub.publish(setpoint)

        rate.sleep()  # 保持 20Hz


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
