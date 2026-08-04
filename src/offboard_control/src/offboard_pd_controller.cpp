/*
 * offboard_pd_controller.cpp — 级联 PD 位置/速度控制器
 * ============================================================
 * 与 offboard_position.cpp 的区别：
 *   offboard_position 只是告诉 PX4 "我要去 (x,y,z)"，如何飞完全由 PX4 内部控制。
 *   offboard_pd_controller 是自己写控制器，算出需要的姿态角和油门，直接发送给飞控执行。
 *
 * 控制架构（级联控制 Cascade Control）：
 *
 *   目标位置         位置误差              速度指令            速度误差         加速度指令      姿态+油门
 *   (x,y,z)  ──→ [P控制器] ──→ vel_des ──→ [PD控制器] ──→ accel_des ──→ [加速度→姿态] ──→ AttitudeTarget
 *                 ↑ 外环                   ↑ 内环              ↑ 转换
 *             Kp_pos × 位置误差       Kp_vel × 速度误差    推力向量 → 四元数姿态
 *                                   + Kd_vel × 速度误差微分
 *
 * 为什么用级联控制？
 *   1. 外环（位置环）只关心"离目标还有多远"，输出期望速度
 *   2. 内环（速度环）负责"怎么达到那个速度"，输出期望加速度
 *   3. 最后把加速度转成"飞行角度 + 油门大小"
 *   这样每一层只负责一件事，调参清晰，稳定性好。
 *
 * 为什么输出 AttitudeTarget 而不是 PositionTarget？
 *   PX4 的 setpoint_position/local 会走它自己的位置/速度/姿态全闭环，
 *   我们无法干预。如果我们要自己实现控制算法（比如加扰动观测器、MPC 等），
 *   就需要绕过 PX4 的位置/速度环，直接控制姿态和油门。
 *
 * 坐标系：ENU（East-North-Up）
 *   X 轴指向东、Y 轴指向北、Z 轴指向上
 *   重力加速度 g 方向为 (0, 0, -9.81)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

// ======================== 全局变量 ========================

mavros_msgs::State current_state;          // 飞控当前状态（连接、模式、解锁）
geometry_msgs::PoseStamped current_pose;   // 无人机当前位置 (ENU)
geometry_msgs::TwistStamped current_vel;   // 无人机当前速度 (ENU)

// ======================== PD 控制器参数 ========================
// 通过 launch 文件或 rosrun _Kp_pos:=xxx 可动态调节

double Kp_pos    = 0.95;   // 位置外环 P 增益：位置误差 → 期望速度（越大响应越快，过大振荡）
double Kp_vel    = 1.8;    // 速度内环 P 增益：速度误差 → 加速度（比例部分）
double Kd_vel    = 0.4;    // 速度内环 D 增益：抑制速度变化过快（微分部分，减振）
double max_vel   = 3.0;    // 期望速度上限 [m/s]（防止飞太快）
double max_accel = 5.0;    // 期望加速度上限 [m/s²]（防止倾角过大）
double max_tilt  = 0.5;    // 最大倾斜角 [rad] ≈ 28.6°（安全限制，防止翻倒）
double hover_thrust = 0.5; // 悬停油门：飞机水平且加速度为 0 时的归一化推力

// ======================== 目标位置 ========================

geometry_msgs::PoseStamped desired_pose;

// ======================== 回调函数 ========================

/**
 * 飞控状态回调
 * 从 /mavros/state 订阅，获得飞控的连接状态、当前飞行模式、是否解锁等
 * 用途：判断何时可以切换 offboard 模式和解锁
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

/**
 * 位置回调
 * 从 /mavros/local_position/pose 订阅，获取无人机在 ENU 坐标系下的 x, y, z 位置
 * 用途：计算位置误差 = 目标位置 - 当前位置
 */
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) { current_pose = *msg; }

/**
 * 速度回调
 * 从 /mavros/local_position/velocity_local 订阅，获取无人机在 ENU 坐标系下的线速度
 * 用途：计算速度误差 = 期望速度 - 当前速度
 */
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) { current_vel = *msg; }

// ======================== 工具函数 ========================

/**
 * 限幅函数（Clamp）
 * 将 val 限制在 [lo, hi] 区间内，防止控制量过大导致不稳定
 *
 * @param val  输入值
 * @param lo   下限
 * @param hi   上限
 * @return     限制后的值
 */
double clamp(double val, double lo, double hi)
{
    return std::max(lo, std::min(hi, val));
}

/**
 * 加速度 → 姿态四元数 + 油门 转换（核心函数）
 * ======================================================
 * 这个函数是整个控制器的核心：给定"我想让飞机如何加速"，
 * 算出"飞机需要倾斜到什么角度、加多大油门"。
 *
 * 物理原理：
 *   四旋翼只能产生沿机体 Z 轴（螺旋桨面法线方向）的推力。
 *   要水平移动，必须倾斜机体，让推力在水平方向产生分量。
 *
 *   例如：想向东加速 a_x，就需要让机身向东倾斜。
 *         倾角 θ ≈ atan(a_x / g)，就可实现。
 *
 * 计算步骤：
 *   1. 期望推力方向（世界坐标系 ENU）：
 *      f_world = 期望加速度 + (0, 0, g)
 *      其中 +g 是为了抵消重力（要悬停就必须产生向上的推力对抗重力）
 *
 *   2. 将该方向归一化 → 机体 Z 轴在世界的方向 z_b_unit
 *
 *   3. 用偏航角（yaw）和 z_b_unit 构造完整的旋转矩阵，
 *      再提取四元数（即无人机姿态）s
 *
 *   4. 油门大小 ≈ |f_world|，归一化到 [0, 1]
 *      悬停时油门大约 0.5~0.6（因为需要对抗重力）
 *
 * @param ax      期望 x 方向加速度 [m/s²] (ENU)
 * @param ay      期望 y 方向加速度 [m/s²] (ENU)
 * @param az      期望 z 方向加速度 [m/s²] (ENU)
 * @param yaw     期望偏航角 [rad]（这里固定 0，即机头朝北）
 * @param q       [输出] 姿态四元数
 * @param thrust  [输出] 归一化油门 [0.05, 1.0]
 */
void accel_to_attitude(const double ax, const double ay, const double az,
                       const double yaw, geometry_msgs::Quaternion &q,
                       double &thrust)
{
    const double g = 9.81;  // 重力加速度 [m/s²]

    // ========== 第 1 步：计算期望推力方向（世界坐标系 ENU）==========
    // 牛二：F = m*a
    // 推力需要同时提供"抵消重力"和"产生加速度"两部分
    // f_x 和 f_y 不需要抵消重力（重力只在 Z 方向）
    // f_z = a_z + g：向上加速需要加大油门，向下加速需要减小油门
    double fx = ax;          // 推力在 X 方向的分量（不需要对抗重力）
    double fy = ay;          // 推力在 Y 方向的分量（不需要对抗重力）
    double fz = az + g;      // 推力在 Z 方向的分量（对抗重力 + 产生 Z 加速度）
    // 例：要悬停，ax=ay=az=0，则 f = (0, 0, g)，即推力方向竖直向上 ✓

    // ========== 第 2 步：归一化得到机体 Z 轴方向 ==========
    // 机体 Z 轴（螺旋桨推力方向）在世界坐标系中应该指向哪里
    double f_norm = std::sqrt(fx*fx + fy*fy + fz*fz);  // 推力向量长度
    if (f_norm < 1e-6)
    {
        // 推力为 0 的异常情况：保持水平，油门 0
        q.w = 1.0; q.x = q.y = q.z = 0.0;
        thrust = 0.0;
        return;
    }

    double zb_x = fx / f_norm;   // 机体 Z 轴在世界的 X 分量
    double zb_y = fy / f_norm;   // 机体 Z 轴在世界的 Y 分量
    double zb_z = fz / f_norm;   // 机体 Z 轴在世界的 Z 分量
    // 悬停时：zb = (0, 0, 1)，即机体 Z 轴竖直向上 ✓

    // ========== 第 3 步：倾斜角限制（安全保护）==========
    // 计算当前需要的倾斜角：机体 Z 轴与世界 Z 轴的夹角
    double tilt = std::acos(clamp(zb_z, -1.0, 1.0));
    if (tilt > max_tilt)
    {
        // 如果需要的倾斜角超过了安全上限，等比例缩小 X、Y 分量
        // 同时修正 Z 分量，保证仍是单位向量
        double scale = std::sin(max_tilt) / std::sin(tilt);
        zb_x *= scale;
        zb_y *= scale;
        zb_z = std::cos(max_tilt);
        // 重新归一化（防止数值误差导致不是单位向量）
        double renorm = std::sqrt(zb_x*zb_x + zb_y*zb_y + zb_z*zb_z);
        zb_x /= renorm; zb_y /= renorm; zb_z /= renorm;
    }

    // ========== 第 4 步：用偏航角构造完整的旋转矩阵 ==========
    // 已知：机体 Z 方向在世界中的方向 = (zb_x, zb_y, zb_z)
    // 还需要：机体 X 轴（机头方向所在）在世界的方向
    //
    // 偏航角 yaw 表示机头朝向（绕世界 Z 轴旋转的角度）
    // 在世界 XY 平面中，机头初始指向 (cos(yaw), sin(yaw))
    //
    // 构造机体 X 轴：
    //   1. 先取机头在水平面的方向：xh = (cos(yaw), sin(yaw), 0)
    //   2. 机体 X 轴必须垂直于机体 Z 轴（因为机体是刚体，坐标轴两两正交）
    //   3. 用 Gram-Schmidt 正交化：X = Xh - (Xh·Z)*Z，然后归一化
    double yc = std::cos(yaw);
    double ys = std::sin(yaw);

    // 机头在水平面的投影方向（这只是一个"大致方向"，后面会正交化）
    double xb_x =  yc * zb_z;          // 直接构造垂直于 Z 的向量
    double xb_y =  ys * zb_z;
    double xb_z = -yc * zb_x - ys * zb_y;
    // 归一化 X 轴
    double xb_norm = std::sqrt(xb_x*xb_x + xb_y*xb_y + xb_z*xb_z);
    xb_x /= xb_norm; xb_y /= xb_norm; xb_z /= xb_norm;

    // 机体 Y 轴 = Z × X（叉积，保证右手坐标系）
    double yb_x = zb_y * xb_z - zb_z * xb_y;
    double yb_y = zb_z * xb_x - zb_x * xb_z;
    double yb_z = zb_x * xb_y - zb_y * xb_x;

    // ========== 第 5 步：旋转矩阵 → 四元数 ==========
    // 旋转矩阵 R 的列分别是机体坐标轴在世界中的表示
    // R = [X_body_in_world, Y_body_in_world, Z_body_in_world]
    tf2::Matrix3x3 R(xb_x, yb_x, zb_x,
                     xb_y, yb_y, zb_y,
                     xb_z, yb_z, zb_z);
    tf2::Quaternion tf2_q;
    R.getRotation(tf2_q);  // tf2 库自动把矩阵转成四元数
    q.w = tf2_q.w(); q.x = tf2_q.x(); q.y = tf2_q.y(); q.z = tf2_q.z();

    // ========== 第 6 步：计算油门 ==========
    // 油门与需要的推力大小成正比
    // f_norm=g 时输出 hover_thrust；加速上升时提高，下降时降低。
    // 不同机型的悬停油门不同，因此通过 ROS 参数配置。
    thrust = clamp((f_norm / g) * hover_thrust, 0.05, 1.0);
}

// ======================== 主函数 ========================

/**
 * 主函数：初始化 ROS 节点，进入 PD 控制循环
 *
 * 整体流程与 offboard_position.cpp 类似，但控制逻辑不同：
 *   1. 等待飞控连接
 *   2. 预热 setpoint 流（发送初始姿态指令，让 PX4 准备好接收 offboard 指令）
 *   3. 切换到 OFFBOARD 模式 + 解锁
 *   4. 进入主循环：
 *      a. 读取当前位置和速度
 *      b. 计算位置误差 → 外环 P 控制 → 期望速度
 *      c. 计算速度误差 → 内环 PD 控制 → 期望加速度
 *      d. 加速度 → 姿态 + 油门
 *      e. 发布 AttitudeTarget 给飞控执行
 */
int main(int argc, char **argv)
{
    // ---- ROS 初始化 ----
    ros::init(argc, argv, "offboard_pd_controller");
    ros::NodeHandle nh("~");  // "~" 表示私有命名空间，参数可通过 launch 文件传入

    // ---- 从参数服务器加载 PD 参数（可被 launch 文件覆盖）----
    nh.param("Kp_pos",    Kp_pos,    0.95);
    nh.param("Kp_vel",    Kp_vel,    1.8);
    nh.param("Kd_vel",    Kd_vel,    0.4);
    nh.param("max_vel",   max_vel,   3.0);
    nh.param("max_accel", max_accel, 5.0);
    nh.param("max_tilt",  max_tilt,  0.5);
    nh.param("hover_thrust", hover_thrust, 0.5);

    // ---- 订阅：飞控状态 / 当前位置 / 当前速度 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, vel_cb);

    // ---- 发布：姿态 + 油门指令 ----
    // AttitudeTarget 消息包含：
    //   - orientation: 目标姿态四元数（机身朝向）
    //   - thrust: 归一化油门 [0, 1]
    //   - type_mask: 位掩码，决定哪些字段生效（这里忽略角速度所以不需要改动）
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);

    // ---- 服务客户端 ----
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");

    // 控制频率 50Hz（姿态控制比位置控制需要更快的更新频率）
    ros::Rate rate(50.0);

    // ===== 等待飞控连接 =====
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接");

    // ===== 初始化目标位置：原点上方 2m =====
    desired_pose.pose.position.x = 0.0;
    desired_pose.pose.position.y = 0.0;
    desired_pose.pose.position.z = 2.0;
    desired_pose.pose.orientation.w = 1.0;

    // ===== 构建 AttitudeTarget 消息模板 =====
    mavros_msgs::AttitudeTarget att_setpoint;
    // type_mask 位掩码：
    //   IGNORE_ROLL_RATE  = 忽略横滚角速度（用姿态控制，不需要主动角速度指令）
    //   IGNORE_PITCH_RATE = 忽略俯仰角速度
    //   IGNORE_YAW_RATE   = 忽略偏航角速度
    // 即：我们只发姿态 + 油门，不发角速度指令
    att_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    // 初始姿态：水平（w=1 的四元数即无旋转）
    att_setpoint.orientation.w = 1.0;
    att_setpoint.thrust = 0.0;  // 油门初始为 0

    // ===== 预热 setpoint 流（原因同 offboard_position.cpp）=====
    for (int i = 50; ros::ok() && i > 0; --i)
    {
        att_setpoint.header.stamp = ros::Time::now();
        att_pub.publish(att_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Setpoint 预热完成");

    // ===== 准备模式切换 / 解锁 =====
    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();

    // ===== 主控制循环 =====
    while (ros::ok())
    {
        // ---- 模式切换与解锁（同 offboard_position.cpp）----
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                ROS_INFO("OFFBOARD 模式已启用");
            last_request = ros::Time::now();
        }
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("无人机已解锁");
            last_request = ros::Time::now();
        }

        // ---- 轨迹规划：随时间改变目标位置 ----
        double elapsed = (ros::Time::now() - start_time).toSec();

        if (elapsed > 10.0 && elapsed <= 20.0)
        {
            desired_pose.pose.position.x = 5.0;
            desired_pose.pose.position.y = 0.0;
            desired_pose.pose.position.z = 3.0;
        }
        else if (elapsed > 20.0 && elapsed <= 30.0)
        {
            desired_pose.pose.position.x = 5.0;
            desired_pose.pose.position.y = 5.0;
            desired_pose.pose.position.z = 3.0;
        }
        else if (elapsed > 30.0 && elapsed <= 40.0)
        {
            desired_pose.pose.position.x = 0.0;
            desired_pose.pose.position.y = 5.0;
            desired_pose.pose.position.z = 3.0;
        }
        else if (elapsed > 40.0)
        {
            desired_pose.pose.position.x = 0.0;
            desired_pose.pose.position.y = 0.0;
            desired_pose.pose.position.z = 2.0;
        }

        // ===================================================================
        // 核心控制逻辑：级联 PD 控制器
        // ===================================================================

        // ---- 第 1 层：外环 P 控制（位置 → 期望速度）----
        // 误差 = 目标位置 - 当前位置，乘 Kp 得到期望速度
        // 物理含义：离目标越远，飞得越快（线性比例）
        double ex = desired_pose.pose.position.x - current_pose.pose.position.x;
        double ey = desired_pose.pose.position.y - current_pose.pose.position.y;
        double ez = desired_pose.pose.position.z - current_pose.pose.position.z;

        // 期望速度 = Kp_pos × 位置误差（限幅防止过快）
        double vel_des_x = clamp(Kp_pos * ex, -max_vel, max_vel);
        double vel_des_y = clamp(Kp_pos * ey, -max_vel, max_vel);
        double vel_des_z = clamp(Kp_pos * ez, -max_vel, max_vel);
        // 例：Kp_pos=0.95，误差=3m → 期望速度=2.85m/s
        // 如果误差很小（接近目标），期望速度也小，自然会减速

        // ---- 第 2 层：内环 PD 控制（速度 → 期望加速度）----
        // 速度误差 = 期望速度 - 当前速度
        double evx = vel_des_x - current_vel.twist.linear.x;
        double evy = vel_des_y - current_vel.twist.linear.y;
        double evz = vel_des_z - current_vel.twist.linear.z;

        // 速度误差的微分（用上一时刻的误差差分近似求导）
        // 微分项作用：预测速度变化趋势，提前抑制，减少超调和振荡
        // d(ev)/dt ≈ (当前误差 - 上次误差) × 控制频率
        static double prev_evx = 0.0, prev_evy = 0.0, prev_evz = 0.0;
        double d_evx = (evx - prev_evx) * 50.0;   // ×50 = 除以 dt(dt=1/50s)
        double d_evy = (evy - prev_evy) * 50.0;
        double d_evz = (evz - prev_evz) * 50.0;
        prev_evx = evx; prev_evy = evy; prev_evz = evz;

        // PD 控制律：a_des = Kp_vel × e_v + Kd_vel × d(e_v)/dt
        // P 项：速度误差越大，加速度越大（驱动飞机追赶期望速度）
        // D 项：速度误差变化越快，反向抑制（"刹车"作用，防止冲过头）
        double ax = clamp(Kp_vel * evx + Kd_vel * d_evx, -max_accel, max_accel);
        double ay = clamp(Kp_vel * evy + Kd_vel * d_evy, -max_accel, max_accel);
        double az = clamp(Kp_vel * evz + Kd_vel * d_evz, -max_accel, max_accel);

        // ---- 第 3 层：加速度 → 姿态 + 油门 ----
        double yaw = 0.0;  // 偏航角固定 0（机头朝北）
        geometry_msgs::Quaternion q;
        double thrust;
        accel_to_attitude(ax, ay, az, yaw, q, thrust);

        // ---- 发布控制指令 ----
        att_setpoint.header.stamp = ros::Time::now();
        att_setpoint.orientation = q;
        att_setpoint.thrust = thrust;
        att_pub.publish(att_setpoint);

        ros::spinOnce();
        rate.sleep();  // 保持 50Hz
    }

    return 0;
}
