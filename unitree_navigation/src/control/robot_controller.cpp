#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <cmath>
#include <string>
#include <algorithm>
#include "unitree_navigation/robot_controller.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <boost/bind.hpp>

#ifndef NULL
#define NULL 0
#endif  

/**
 * @brief 构造函数，初始化ROS参数、发布者、订阅者和服务
 * @param nh ROS节点句柄
 */
RobotController::RobotController(ros::NodeHandle& nh) : nh_(nh), private_nh_("~") {
    // 获取参数
    private_nh_.param<std::string>("robot_ip", robot_ip_, "192.168.123.220");
    private_nh_.param<double>("control_frequency", control_frequency_, 500.0);
    private_nh_.param<double>("max_linear_speed", max_linear_speed_, 0.5);
    private_nh_.param<double>("max_angular_speed", max_angular_speed_, 0.5);
    
    // 创建发布者和订阅者
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &RobotController::cmdVelCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/unitree/joint_states", 50);
    
    // 创建服务
    stand_service_ = nh_.advertiseService("robot_stand", &RobotController::standService, this);
    sit_service_ = nh_.advertiseService("robot_sit", &RobotController::sitService, this);
    
    // 初始化UDP通信
    InitUDPConnection();
    
    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    
    // 初始化关节状态消息
    joint_state_msg_.name = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };
    joint_state_msg_.position.resize(12, 0.0);
    joint_state_msg_.velocity.resize(12, 0.0);
    joint_state_msg_.effort.resize(12, 0.0);
    
    // 初始化里程计消息
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    
    // 初始化机器人状态
    robot_state_ = RobotState::UNKNOWN;
    
    // 初始化最后命令时间
    last_cmd_time_ = ros::Time::now();
    
    ROS_INFO("机器人控制器已初始化");
    ROS_INFO("机器人IP: %s", robot_ip_.c_str());
    ROS_INFO("控制频率: %.1f Hz", control_frequency_);
    ROS_INFO("最大线速度: %.2f m/s", max_linear_speed_);
    ROS_INFO("最大角速度: %.2f rad/s", max_angular_speed_);
}

/**
 * @brief 析构函数，关闭UDP连接
 */
RobotController::~RobotController() {
    // 关闭UDP连接
    CloseUDPConnection();
}

/**
 * @brief 初始化UDP连接
 */
void RobotController::InitUDPConnection() {
    // 初始化UDP通信，使用Unitree SDK
    ROS_INFO("初始化与机器人的UDP连接: %s", robot_ip_.c_str());
    
    // 创建UDP通信对象，使用高级控制模式
    udp_ = std::make_unique<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::HIGHLEVEL, 
                                                    8090, 
                                                    robot_ip_.c_str(), 
                                                    8082);
    
    // 初始化命令和状态
    udp_->InitCmdData(high_cmd_);
    
    // 初始化机器人状态
    high_cmd_.mode = 0;      // 空闲模式
    high_cmd_.gaitType = 0;  // 默认步态
    high_cmd_.velocity[0] = 0.0f;  // 前进速度
    high_cmd_.velocity[1] = 0.0f;  // 侧向速度
    high_cmd_.yawSpeed = 0.0f;     // 转向速度
    high_cmd_.bodyHeight = 0.0f;   // 身体高度
    
    // 创建控制、发送和接收线程
    control_thread_ = std::make_unique<UNITREE_LEGGED_SDK::LoopFunc>(
        "control_loop", 
        1.0/control_frequency_, 
        boost::bind(&RobotController::RobotControl, this));
        
    udp_send_thread_ = std::make_unique<UNITREE_LEGGED_SDK::LoopFunc>(
        "udp_send", 
        1.0/control_frequency_, 
        3,  // 优先级
        boost::bind(&RobotController::UDPSend, this));
        
    udp_recv_thread_ = std::make_unique<UNITREE_LEGGED_SDK::LoopFunc>(
        "udp_recv", 
        1.0/control_frequency_, 
        3,  // 优先级
        boost::bind(&RobotController::UDPRecv, this));
    
    // 启动线程
    control_thread_->start();
    udp_send_thread_->start();
    udp_recv_thread_->start();
    
    ROS_INFO("UDP连接初始化完成");
}

/**
 * @brief 关闭UDP连接
 */
void RobotController::CloseUDPConnection() {
    // 关闭UDP连接
    ROS_INFO("关闭与机器人的UDP连接");
    
    // 停止线程
    if (control_thread_) {
        control_thread_->shutdown();
    }
    
    if (udp_send_thread_) {
        udp_send_thread_->shutdown();
    }
    
    if (udp_recv_thread_) {
        udp_recv_thread_->shutdown();
    }
}

/**
 * @brief 机器人控制函数，处理和准备命令
 */
void RobotController::RobotControl() {
    // 获取机器人状态
    udp_->GetRecv(high_state_);
    
    // 如果长时间没有收到命令，切换到站立模式
    if (ros::Time::now() - last_cmd_time_ > ros::Duration(3)) {
        high_cmd_.mode = 1;  // 强制站立模式
    }
    
    // 设置命令准备发送
    udp_->SetSend(high_cmd_);
}

/**
 * @brief UDP发送函数
 */
void RobotController::UDPSend() {
    // 只负责发送
    if (udp_) {
        udp_->Send();
    }
}

/**
 * @brief UDP接收函数
 */
void RobotController::UDPRecv() {
    // 只负责接收
    if (udp_) {
        udp_->Recv();
    }
}

/**
 * @brief 速度命令回调函数
 * @param msg 速度命令消息
 */
void RobotController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 更新最后命令时间
    last_cmd_time_ = ros::Time::now();
    
    // 限制速度在安全范围内
    double linear_x = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
    double linear_y = std::clamp(msg->linear.y, -max_linear_speed_, max_linear_speed_);
    double angular_z = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
    
    // 更新命令
    high_cmd_.mode = 2;  // 行走模式
    high_cmd_.velocity[0] = static_cast<float>(linear_x);
    high_cmd_.velocity[1] = static_cast<float>(linear_y);
    high_cmd_.yawSpeed = static_cast<float>(angular_z);
    
    // 如果速度为零，切换到站立模式
    if (std::abs(linear_x) < 0.01 && std::abs(linear_y) < 0.01 && std::abs(angular_z) < 0.01) {
        high_cmd_.mode = 1; 
    }
}

/**
 * @brief 站立服务回调函数
 * @param req 请求
 * @param res 响应
 * @return 是否成功
 */
bool RobotController::standService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    // 让机器人站立
    high_cmd_.mode = 1;      // 强制站立模式 (forced stand)
    high_cmd_.gaitType = 0;  // 默认步态
    high_cmd_.velocity[0] = 0.0f;
    high_cmd_.velocity[1] = 0.0f;
    high_cmd_.yawSpeed = 0.0f;
    high_cmd_.bodyHeight = 0.0f;
    
    robot_state_ = RobotState::STANDING;
    
    res.success = true;
    res.message = "机器人正在站立";
    ROS_INFO("机器人正在站立");
    
    return true;
}

/**
 * @brief 坐下服务回调函数
 * @param req 请求
 * @param res 响应
 * @return 是否成功
 */
bool RobotController::sitService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    // 让机器人坐下
    high_cmd_.mode = 5;      // 坐下模式 (对应example_walk.cpp中的模式5)
    high_cmd_.gaitType = 0;  
    high_cmd_.velocity[0] = 0.0f;
    high_cmd_.velocity[1] = 0.0f;
    high_cmd_.yawSpeed = 0.0f;
    
    robot_state_ = RobotState::SITTING;
    
    res.success = true;
    res.message = "机器人正在坐下";
    ROS_INFO("机器人正在坐下");
    
    return true;
}

/**
 * @brief 更新机器人状态
 */
void RobotController::updateRobotState() {
    // 更新关节状态
    joint_state_msg_.header.stamp = ros::Time::now();
    
    // 使用从机器人接收到的实际关节状态
    for (int i = 0; i < 3; i++) {
        // FR腿
        joint_state_msg_.position[i] = high_state_.motorState[i].q;
        joint_state_msg_.velocity[i] = high_state_.motorState[i].dq;
        joint_state_msg_.effort[i] = high_state_.motorState[i].tauEst;
        
        // FL腿
        joint_state_msg_.position[i+3] = high_state_.motorState[i+3].q;
        joint_state_msg_.velocity[i+3] = high_state_.motorState[i+3].dq;
        joint_state_msg_.effort[i+3] = high_state_.motorState[i+3].tauEst;
        
        // RR腿
        joint_state_msg_.position[i+6] = high_state_.motorState[i+6].q;
        joint_state_msg_.velocity[i+6] = high_state_.motorState[i+6].dq;
        joint_state_msg_.effort[i+6] = high_state_.motorState[i+6].tauEst;
        
        // RL腿
        joint_state_msg_.position[i+9] = high_state_.motorState[i+9].q;
        joint_state_msg_.velocity[i+9] = high_state_.motorState[i+9].dq;
        joint_state_msg_.effort[i+9] = high_state_.motorState[i+9].tauEst;
    }
    
    // 更新里程计
    odom_msg_.header.stamp = ros::Time::now();
    
    // 使用机器人IMU和位置信息更新里程计
    static double x = 0.0, y = 0.0, theta = 0.0;
    static ros::Time last_time = ros::Time::now();
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    
    // 使用IMU数据获取方向
    theta = high_state_.imu.rpy[2];  // yaw角
    
    // 使用速度数据更新位置（简单积分）
    x += high_state_.velocity[0] * dt * cos(theta) - high_state_.velocity[1] * dt * sin(theta);
    y += high_state_.velocity[0] * dt * sin(theta) + high_state_.velocity[1] * dt * cos(theta);
    
    // 设置位置
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = 0.0;
    
    // 设置方向
    tf2::Quaternion q;
    q.setRPY(high_state_.imu.rpy[0], high_state_.imu.rpy[1], high_state_.imu.rpy[2]);
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);
    
    // 设置速度
    odom_msg_.twist.twist.linear.x = high_state_.velocity[0];
    odom_msg_.twist.twist.linear.y = high_state_.velocity[1];
    odom_msg_.twist.twist.angular.z = high_state_.yawSpeed;
}

/**
 * @brief 发布TF变换
 */
void RobotController::publishTransforms() {
    // 发布TF变换
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform.transform.translation.z = odom_msg_.pose.pose.position.z;
    
    transform.transform.rotation = odom_msg_.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

/**
 * @brief 初始化机器人
 */
void RobotController::InitRobot() {
    // 初始化机器人状态
    high_cmd_.mode = 0;      // 空闲模式，默认站立
    high_cmd_.gaitType = 0;
    high_cmd_.speedLevel = 0;
    high_cmd_.footRaiseHeight = 0;
    high_cmd_.bodyHeight = 0;
    high_cmd_.euler[0] = 0;
    high_cmd_.euler[1] = 0;
    high_cmd_.euler[2] = 0;
    high_cmd_.velocity[0] = 0.0f;
    high_cmd_.velocity[1] = 0.0f;
    high_cmd_.yawSpeed = 0.0f;
    high_cmd_.reserve = 0;
    
    // 等待机器人初始化
    ros::Duration(1.0).sleep();
    
    // 让机器人站立
    high_cmd_.mode = 1;  // 强制站立模式
    
    ROS_INFO("机器人初始化完成，进入站立模式");
}

/**
 * @brief 运行控制器主循环
 */
void RobotController::run() {
    ros::Rate rate(control_frequency_);
    
    // 初始化机器人
    InitRobot();
    
    while (ros::ok()) {
        // 更新机器人状态
        updateRobotState();
        
        // 发布关节状态
        joint_states_pub_.publish(joint_state_msg_);
        
        // 发布里程计
        odom_pub_.publish(odom_msg_);
        
        // 发布TF变换
        publishTransforms();
        
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief 初始化控制器
 * @return 是否成功初始化
 */
bool RobotController::Initialize() {
    // 已在构造函数中完成初始化
    return true;
}

/**
 * @brief 启动控制器
 * @return 是否成功启动
 */
bool RobotController::Start() {
    // 初始化机器人
    InitRobot();
    return true;
}

/**
 * @brief 停止控制器
 */
void RobotController::Stop() {
    // 让机器人站立
    high_cmd_.mode = 1;  // 强制站立模式
    high_cmd_.velocity[0] = 0.0f;
    high_cmd_.velocity[1] = 0.0f;
    high_cmd_.yawSpeed = 0.0f;
    
    // 等待命令发送
    ros::Duration(0.1).sleep();
}

/**
 * @brief 设置机器人速度
 * @param cmd_vel 速度命令
 */
void RobotController::SetVelocity(const geometry_msgs::Twist& cmd_vel) {
    // 创建一个临时的Twist消息指针
    geometry_msgs::Twist::ConstPtr msg_temp(new geometry_msgs::Twist(cmd_vel));
    
    // 调用cmdVelCallback
    cmdVelCallback(msg_temp);
}

/**
 * @brief 更新机器人状态
 */
void RobotController::UpdateRobotState() {
    // 直接调用现有方法
    updateRobotState();
    
    // 发布关节状态
    joint_states_pub_.publish(joint_state_msg_);
    
    // 发布里程计
    odom_pub_.publish(odom_msg_);
    
    // 发布TF变换
    publishTransforms();
}