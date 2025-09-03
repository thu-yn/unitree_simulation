/************************************************************************
* 文件名: servo.cpp
* 功能描述: Unitree四足机器人主伺服控制程序
* 
* 主要功能:
* 1. 🔴 核心控制器 - 实现机器人的主要控制逻辑
* 2. 多线程消息处理 - 异步处理传感器数据和关节状态
* 3. 仿真接口桥接 - 连接ROS控制系统与Gazebo仿真环境
* 4. 实时状态管理 - 维护机器人状态信息并发布给上层应用
* 
* 程序架构:
* - 主线程: 负责控制逻辑和状态发布
* - 异步线程: 处理传感器数据订阅和回调
* - 控制频率: 高频实时控制循环
* 
* 在整个项目中的作用:
* - 🔴 第3层中层控制器 - 位于硬件抽象层与导航层之间
* - 是运行 "rosrun unitree_controller unitree_servo" 的主程序
* - 让机器人从平躺状态站立起来并保持控制连接
* 
* 使用方法:
* 1. 先启动Gazebo仿真环境
* 2. 运行此程序让机器人站立
* 3. 机器人进入可控状态，等待上层命令
* 
* Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
* Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"      // 底层控制命令消息
#include "unitree_legged_msgs/LowState.h"    // 底层状态反馈消息
#include "unitree_legged_msgs/MotorCmd.h"    // 单个电机控制命令
#include "unitree_legged_msgs/MotorState.h"  // 单个电机状态反馈
#include <geometry_msgs/WrenchStamped.h>     // 力和力矩消息(足端接触力)
#include <sensor_msgs/Imu.h>                 // IMU传感器消息
#include <std_msgs/Bool.h>                   // 布尔值消息
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>               // 里程计消息
#include "body.h"                            // 引入机器人本体控制头文件

using namespace std;
using namespace unitree_model;  // 使用机器人模型命名空间

/**
* @brief 系统启动标志
* 
* 用途:
* - true: 系统刚启动，还没有接收到关节状态数据
* - false: 系统已正常运行，可以开始控制
* 
* 重要性:
* - 防止在获取初始状态前就开始控制
* - 确保控制系统的安全启动
*/
bool start_up = true;

/**
* @class multiThread
* @brief 多线程消息处理类
* 
* 功能描述:
* - 管理所有传感器数据的订阅和处理
* - 实现12个关节状态的异步接收
* - 处理IMU数据和足端接触力数据
* - 支持不同机器人型号的参数化配置
* 
* 设计模式:
* - 使用ROS的异步回调机制
* - 每个传感器对应一个专门的回调函数
* - 通过成员变量保存机器人名称实现多机器人支持
*/
class multiThread
{
public:
    /**
    * @brief 多线程处理类构造函数
    * 
    * @param rname 机器人名称(如: go1, a1, laikago等)
    * 
    * 功能描述:
    * - 初始化所有传感器数据订阅器
    * - 设置对应的回调函数
    * - 支持多种机器人型号的动态配置
    * 
    * 订阅的话题类型:
    * 1. IMU传感器数据 - 机器人姿态和运动状态
    * 2. 4个足端接触力 - 用于步态规划和平衡控制
    * 3. 12个关节状态 - 位置、速度、力矩反馈
    */
    multiThread(string rname)
    {
        robot_name = rname;  // 保存机器人名称
        
        // ========== 传感器数据订阅器初始化 ==========
        
        /**
        * IMU传感器订阅器
        * 话题: /trunk_imu
        * 消息类型: sensor_msgs::Imu
        * 功能: 获取机器人本体的姿态、角速度、线加速度
        */
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        
        /**
        * 足端接触力订阅器 (4个足端)
        * 话题: /visual/[腿名]_foot_contact/the_force
        * 消息类型: geometry_msgs::WrenchStamped
        * 功能: 获取每个足端与地面的接触力信息
        */
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        
        /**
        * 关节状态订阅器 (12个关节)
        * 话题模式: /{robot_name}_gazebo/{关节名}_controller/state
        * 消息类型: unitree_legged_msgs::MotorState
        * 功能: 获取每个关节的实际位置、速度、力矩等状态信息
        * 
        * 关节命名规则:
        * FR - 前右腿, FL - 前左腿, RR - 后右腿, RL - 后左腿
        * hip - 髋关节, thigh - 大腿关节, calf - 小腿关节
        */
        
        // 前右腿 (Front Right) 关节状态订阅
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        
        // 前左腿 (Front Left) 关节状态订阅
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        
        // 后右腿 (Rear Right) 关节状态订阅
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        
        // 后左腿 (Rear Left) 关节状态订阅
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    // ==================== 传感器数据回调函数 ====================

    /**
    * @brief IMU传感器数据回调函数
    * 
    * @param msg IMU传感器消息，包含姿态、角速度、线加速度
    * 
    * 功能描述:
    * - 将ROS标准IMU消息转换为Unitree消息格式
    * - 更新机器人本体的姿态和运动状态
    * - 为上层控制算法提供姿态反馈
    * 
    * 数据转换:
    * - 四元数姿态: w, x, y, z
    * - 角速度: x, y, z轴的角速度(rad/s)
    * - 线加速度: x, y, z轴的加速度(m/s²)
    */
    void imuCallback(const sensor_msgs::Imu &msg)
    {
        // 转换四元数姿态数据
        lowState.imu.quaternion[0] = msg.orientation.w;    // 四元数实部
        lowState.imu.quaternion[1] = msg.orientation.x;    // 四元数虚部i
        lowState.imu.quaternion[2] = msg.orientation.y;    // 四元数虚部j
        lowState.imu.quaternion[3] = msg.orientation.z;    // 四元数虚部k

        // 转换角速度数据 (陀螺仪)
        lowState.imu.gyroscope[0] = msg.angular_velocity.x;    // 绕x轴角速度
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;    // 绕y轴角速度
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;    // 绕z轴角速度

        // 转换线加速度数据 (加速度计)
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;  // x轴加速度
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;  // y轴加速度
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;  // z轴加速度
    }

    // ==================== 关节状态回调函数 ====================
    // 以下函数处理12个关节的状态反馈，每个关节对应一个回调函数
    // 功能: 将Gazebo中的关节状态数据更新到lowState结构体中

    /**
    * @brief 前右腿髋关节状态回调函数
    * 
    * @param msg 电机状态消息
    * 
    * 特殊功能:
    * - 除了更新关节状态外，还将start_up标志设为false
    * - 表示系统已经开始接收关节数据，可以进行控制
    */
    void FRhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        start_up = false;  // 标记系统启动完成
        lowState.motorState[0].mode = msg.mode;        // 控制模式
        lowState.motorState[0].q = msg.q;              // 关节位置(rad)
        lowState.motorState[0].dq = msg.dq;            // 关节速度(rad/s)
        lowState.motorState[0].tauEst = msg.tauEst;    // 关节力矩估计(N·m)
    }

    /**
    * @brief 前右腿大腿关节状态回调函数
    */
    void FRthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;
    }

    /**
    * @brief 前右腿小腿关节状态回调函数
    */
    void FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;
    }

    /**
    * @brief 前左腿髋关节状态回调函数
    * 同样具有启动检测功能
    */
    void FLhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        start_up = false;  // 标记系统启动完成
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;
    }

    /**
    * @brief 前左腿大腿关节状态回调函数
    */
    void FLthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;
    }

    /**
    * @brief 前左腿小腿关节状态回调函数
    */
    void FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;
    }

    /**
    * @brief 后右腿髋关节状态回调函数
    * 同样具有启动检测功能
    */
    void RRhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        start_up = false;  // 标记系统启动完成
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;
    }

    /**
    * @brief 后右腿大腿关节状态回调函数
    */
    void RRthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;
    }

    /**
    * @brief 后右腿小腿关节状态回调函数
    */
    void RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;
    }

    /**
    * @brief 后左腿髋关节状态回调函数
    * 同样具有启动检测功能
    */
    void RLhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        start_up = false;  // 标记系统启动完成
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    /**
    * @brief 后左腿大腿关节状态回调函数
    */
    void RLthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    /**
    * @brief 后左腿小腿关节状态回调函数
    */
    void RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    // ==================== 足端接触力回调函数 ====================
    // 以下函数处理4个足端的接触力数据

    /**
    * @brief 前右腿足端接触力回调函数
    */
    void FRfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;  // 通常关注z轴(垂直)方向的力
    }

    /**
    * @brief 前左腿足端接触力回调函数
    */
    void FLfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    /**
    * @brief 后右腿足端接触力回调函数
    */
    void RRfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    /**
    * @brief 后左腿足端接触力回调函数
    */
    void RLfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    // ==================== 私有成员变量 ====================
    
    ros::NodeHandle nm;                    // ROS节点句柄
    ros::Subscriber imu_sub;               // IMU数据订阅器
    ros::Subscriber servo_sub[12];         // 12个关节状态订阅器
    ros::Subscriber footForce_sub[4];      // 4个足端接触力订阅器
    string robot_name;                     // 机器人名称(支持多机器人)
};

// ==================== 主函数 ====================

/**
* @brief 主函数 - 伺服控制程序入口
* 
* 执行流程:
* 1. 初始化ROS节点和参数
* 2. 启动多线程异步消息处理
* 3. 等待系统稳定并获取初始状态
* 4. 设置关节控制发布器
* 5. 初始化机器人并执行站立动作
* 6. 进入主控制循环
* 
* 程序特点:
* - 异步消息处理确保实时性
* - 安全的启动序列
* - 高频控制循环
* - 优雅的错误处理
*/
int main(int argc, char **argv)
{
    // ========== ROS系统初始化 ==========
    
    /**
    * 初始化ROS节点
    * 节点名: "unitree_servo_control"
    */
    ros::init(argc, argv, "unitree_servo_control");
    
    /**
    * 获取机器人名称参数
    * 默认值: "go1"
    * 支持的机器人: go1, a1, laikago, aliengo等
    */
    std::string robot_name;
    ros::NodeHandle nh;
    nh.param<std::string>("robot_name", robot_name, "go1");
    
    /**
    * 创建多线程消息处理对象
    * 功能: 异步处理所有传感器数据订阅
    */
    multiThread listen(robot_name);
    
    // ========== 异步消息处理启动 ==========
    
    /**
    * 启动异步消息处理线程
    * 参数: 1 表示使用单线程处理所有回调
    * 优点: 消息处理不阻塞主控制循环
    */
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /**
    * 等待300ms获取初始传感器数据
    * 重要性: 确保在开始控制前已经收到关节状态信息
    * 避免使用未初始化的数据进行控制
    */
    usleep(300000); // must wait 300ms, to get first state

    // ========== ROS发布器初始化 ==========
    
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // for rviz visualization
    // ros::Rate loop_rate(1000);
    
    /**
    * 底层状态发布器
    * 话题: /{robot_name}_gazebo/lowState/state
    * 功能: 向上层应用(如导航系统)发布机器人完整状态
    */
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    
    /**
    * 12个关节控制命令发布器初始化
    * 话题模式: /{robot_name}_gazebo/{关节名}_controller/command
    * 功能: 向Gazebo中的关节控制器发送控制命令
    * 
    * 注意: 这些发布器已经在gazebo.launch中初始化，这里是重新设置
    */
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    // ========== 机器人控制初始化 ==========
    
    /**
    * 执行机器人运动初始化
    * 功能: 
    * 1. 设置PD控制器参数
    * 2. 执行站立动作
    * 
    * 位置定义在 body.cpp 中的 stand() 函数
    * 这是让机器人从平躺状态站立起来的关键步骤
    */
    motion_init(); // the position is defined in body.cpp --> void stand()

    // ========== 主控制循环 ==========
    
    /**
    * 主控制循环
    * 
    * 功能:
    * 1. 持续发布机器人状态信息
    * 2. 持续发送关节控制命令
    * 3. 保持与Gazebo和上层应用的通信
    * 
    * 特点:
    * - 高频率执行(接近1000Hz)
    * - 实时响应ROS系统状态
    * - 支持Ctrl+C优雅退出
    */
    while (ros::ok())
    {
        /*
        控制逻辑区域
        
        在这里可以添加:
        - 步态规划算法
        - 平衡控制逻辑  
        - 导航指令处理
        - 安全监控机制
        - 其他高级控制功能
        
        目前实现的是基础的站立保持功能
        */
        
        /**
        * 发布机器人底层状态
        * 包含: 关节状态、IMU数据、足端接触力等
        * 供上层应用(导航、规划等)使用
        */
        lowState_pub.publish(lowState);
        
        /**
        * 发送关节伺服控制命令
        * 维持机器人的站立姿态或执行运动控制
        */
        sendServoCmd();
    }
    
    return 0;
}