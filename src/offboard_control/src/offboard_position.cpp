/*
 * offboard_position.cpp — PX4 Offboard 位置控制节点
 * ================================================
 * 功能：通过 MAVROS 向 PX4 飞控发送目标位置（x, y, z），由 PX4 内部控制器完成
 *       姿态和速度的闭环控制。这是最简单的 offboard 控制方式，PX4 官网示例风格。
 *
 * 工作流程：
 *   1. 等待飞控连接（FCU connection）
 *   2. 先持续发送 setpoint，让 PX4 收到足够多的位置指令（否则切 offboard 会失败）
 *   3. 切换到 OFFBOARD 模式
 *   4. 解锁（arm）
 *   5. 循环发送目标位置，无人机自动飞过去
 *
 * 发布话题：/mavros/setpoint_position/local  (geometry_msgs/PoseStamped)
 * 订阅话题：/mavros/state                      (飞控状态，用于判断连接/模式/解锁)
 *           /mavros/local_position/pose         (当前位置，坐标系 ENU)
 *
 * 坐标系：MAVROS 默认使用 ENU（East-North-Up）
 *         - X 指向东，Y 指向北，Z 指向上
 *         - 起飞高度 2m 就是 z=2.0
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 全局变量存储飞控当前状态（连接、模式、解锁等）
mavros_msgs::State current_state;
// 全局变量存储无人机当前位置
geometry_msgs::PoseStamped current_pose;

/**
 * 飞控状态回调
 * 每当 /mavros/state 话题有更新时，将最新状态存入全局变量 current_state
 * 后续用 current_state.connected / .mode / .armed 来判断飞控状态
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/**
 * 位置回调
 * 每当 /mavros/local_position/pose 话题有更新时，存入 current_pose
 * 本节点虽然订阅了位置，但只是用来监控，控制逻辑不依赖它（交给 PX4 自己控制）
 */
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点，节点名为 "offboard_position"
    ros::init(argc, argv, "offboard_position");
    ros::NodeHandle nh;

    // ---- 订阅飞控状态和位置 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);

    // ---- 发布目标位置 ----
    // PX4 要求 offboard 模式下至少 2Hz 持续收到 setpoint，否则会自动切回 Position 模式
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    // ---- 服务客户端：用于解锁和切换飞行模式 ----
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    // 控制频率 20Hz（PX4 offboard 最低要求 2Hz，建议 20~50Hz）
    ros::Rate rate(20.0);

    // ====================================================================
    // 第 1 步：等待飞控连接
    // 阻塞直到 MAVROS 与 PX4 SITL 建立连接（current_state.connected == true）
    // ====================================================================
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();    // 处理一次 ROS 回调队列（拉取最新 state）
        rate.sleep();       // 休眠以维持固定频率
    }
    ROS_INFO("飞控已连接，准备起飞");

    // ====================================================================
    // 第 2 步：在切 OFFBOARD 模式之前，先连续发送 setpoint
    // PX4 要求进入 offboard 前必须已收到一定数量的 setpoint 消息，
    // 否则模式切换会被拒绝。这里发 50 次（约 2.5 秒）。
    // ====================================================================
    geometry_msgs::PoseStamped setpoint;
    // 初始目标位置：原点上方 2m，保持水平
    setpoint.pose.position.x = 0.0;
    setpoint.pose.position.y = 0.0;
    setpoint.pose.position.z = 2.0;
    // 四元数 w=1 表示无旋转（水平姿态）
    setpoint.pose.orientation.w = 1.0;

    for (int i = 50; ros::ok() && i > 0; --i)
    {
        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Setpoint 预热完成，准备切换 OFFBOARD 模式");

    // ====================================================================
    // 第 3 步：准备模式切换和解锁请求
    // ====================================================================
    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";  // 自定义模式字符串，PX4 的 offboard 叫 "OFFBOARD"

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // true = 解锁，false = 上锁

    ros::Time last_request = ros::Time::now();  // 记录上次请求时间，防止频繁重试
    ros::Time start_time = ros::Time::now();    // 记录程序启动时间，用于轨迹计时

    // ====================================================================
    // 第 4 步：主控制循环
    // ====================================================================
    while (ros::ok())
    {
        // ---- 4a. 尝试切换到 OFFBOARD 模式 ----
        // 如果当前不在 OFFBOARD 模式，且距上次请求已过 5 秒，则发送切换请求
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
            {
                ROS_INFO("OFFBOARD 模式已启用");
            }
            last_request = ros::Time::now();
        }
        // ---- 4b. 尝试解锁 ----
        // 已进入 OFFBOARD 但尚未解锁时，发送解锁请求
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("无人机已解锁");
            }
            last_request = ros::Time::now();
        }

        // ---- 4c. 轨迹规划：随时间变化的目标位置 ----
        double elapsed = (ros::Time::now() - start_time).toSec();

        if (elapsed > 10.0 && elapsed <= 20.0)
        {
            // 10~20 秒：飞往 (5, 0, 3) —— 向东 5 米，高度 3 米
            setpoint.pose.position.x = 5.0;
            setpoint.pose.position.y = 0.0;
            setpoint.pose.position.z = 3.0;
        }
        else if (elapsed > 20.0 && elapsed <= 30.0)
        {
            // 20~30 秒：飞往 (5, 5, 3) —— 向东北角
            setpoint.pose.position.x = 5.0;
            setpoint.pose.position.y = 5.0;
            setpoint.pose.position.z = 3.0;
        }
        else if (elapsed > 30.0 && elapsed <= 40.0)
        {
            // 30~40 秒：飞往 (0, 5, 3) —— 向北 5 米
            setpoint.pose.position.x = 0.0;
            setpoint.pose.position.y = 5.0;
            setpoint.pose.position.z = 3.0;
        }
        else if (elapsed > 40.0)
        {
            // 40 秒后：返回原点 (0, 0, 2)
            setpoint.pose.position.x = 0.0;
            setpoint.pose.position.y = 0.0;
            setpoint.pose.position.z = 2.0;
        }

        // ---- 4d. 发布目标位置 ----
        // 必须带时间戳，PX4 用时间戳判断 setpoint 是否新鲜
        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);

        ros::spinOnce();  // 处理回调
        rate.sleep();     // 保持 20Hz
    }

    return 0;
}