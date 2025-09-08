/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
 * @file IOROS.h
 * @brief ROS/Gazebo仿真环境通信接口头文件
 * 
 * 文件作用说明：
 * 这个文件定义了IOROS类，它是unitree_guide项目中负责与ROS/Gazebo仿真环境进行通信的核心接口。
 * 该类继承自IOInterface抽象基类，实现了与Gazebo仿真环境中四足机器人的双向通信功能。
 * 
 * 主要功能：
 * 1. 通过ROS话题系统与Gazebo仿真环境中的机器人模型通信
 * 2. 发布12个关节的控制命令到各个关节控制器
 * 3. 订阅12个关节的状态反馈和IMU传感器数据
 * 4. 实现消息格式转换（内部消息格式 ↔ ROS消息格式）
 * 5. 提供键盘输入处理功能
 * 
 * 在整个系统中的作用：
 * - 是控制系统与仿真环境的桥梁
 * - 屏蔽了ROS通信的复杂性，为上层控制算法提供统一接口
 * - 支持多线程异步通信，保证500Hz控制频率的实时性
 */

// 条件编译：只有当定义了COMPILE_WITH_ROS宏时才编译此文件
// 这样可以在不同的编译配置下选择性地包含ROS相关代码
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

// ==================== 头文件包含 ====================

// ROS核心库
#include "ros/ros.h"                           // ROS核心功能（节点、话题、服务等）

// 项目内部头文件
#include "interface/IOInterface.h"             // 抽象IO接口基类，定义了统一的通信接口

// Unitree机器人专用的ROS消息类型
#include "unitree_legged_msgs/LowCmd.h"        // 底层控制命令消息（发送给机器人）
#include "unitree_legged_msgs/LowState.h"      // 底层状态反馈消息（从机器人接收）
#include "unitree_legged_msgs/MotorCmd.h"      // 单个电机控制命令消息
#include "unitree_legged_msgs/MotorState.h"    // 单个电机状态反馈消息

// ROS标准传感器消息
#include <sensor_msgs/Imu.h>                   // IMU传感器消息（姿态、角速度、加速度）

// 标准库
#include <string>                              // 字符串处理

/**
 * @class IOROS
 * @brief ROS/Gazebo仿真环境通信接口类
 * 
 * 这个类是IOInterface的具体实现，专门用于与ROS/Gazebo仿真环境通信。
 * 它负责：
 * 1. 管理与Gazebo中12个关节控制器的ROS话题通信
 * 2. 处理IMU传感器数据的订阅
 * 3. 实现消息格式转换和数据同步
 * 4. 提供多线程安全的通信机制
 */
class IOROS : public IOInterface{
public:
    /**
     * @brief 构造函数
     * 
     * 主要功能：
     * 1. 初始化ROS节点和通信参数
     * 2. 从ROS参数服务器获取机器人名称
     * 3. 启动多线程的订阅者和发布者
     * 4. 创建键盘输入处理对象
     * 5. 设置信号处理（用于优雅关闭）
     */
    IOROS();
    
    /**
     * @brief 析构函数
     * 
     * 负责清理资源：
     * 1. 删除命令面板对象
     * 2. 关闭ROS节点
     */
    ~IOROS();
    
    /**
     * @brief 发送命令并接收状态的主接口函数
     * @param cmd 指向底层控制命令的指针（输入）
     * @param state 指向底层状态对象的指针（输出）
     * 
     * 这是IOInterface的核心虚函数实现，每个控制周期（500Hz）都会调用。
     * 功能：
     * 1. 调用sendCmd()发送控制命令到Gazebo
     * 2. 调用recvState()接收机器人状态
     * 3. 从命令面板获取用户输入并更新到state中
     */
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    // ==================== 核心通信函数 ====================
    
    /**
     * @brief 发送控制命令到Gazebo仿真环境
     * @param cmd 底层控制命令指针
     * 
     * 功能：
     * 1. 将内部LowlevelCmd格式转换为ROS unitree_legged_msgs::LowCmd格式
     * 2. 通过12个发布者分别发送到各个关节控制器
     * 3. 调用ros::spinOnce()处理ROS回调
     */
    void sendCmd(const LowlevelCmd *cmd);
    
    /**
     * @brief 从Gazebo仿真环境接收状态数据
     * @param state 底层状态对象指针（输出）
     * 
     * 功能：
     * 1. 将ROS unitree_legged_msgs::LowState格式转换为内部LowlevelState格式
     * 2. 复制12个电机的状态数据（位置、速度、力矩等）
     * 3. 复制IMU数据（四元数、角速度、加速度）
     */
    void recvState(LowlevelState *state);
    
    // ==================== ROS通信对象 ====================
    
    /**
     * @brief ROS节点句柄
     * 用于管理ROS节点的生命周期，创建发布者和订阅者
     */
    ros::NodeHandle _nm;
    
    /**
     * @brief 12个关节状态订阅者数组
     * 每个订阅者负责接收一个关节的状态反馈
     * 索引对应关节编号：
     * [0-2]: FR腿（hip, thigh, calf）
     * [3-5]: FL腿（hip, thigh, calf）
     * [6-8]: RR腿（hip, thigh, calf）
     * [9-11]: RL腿（hip, thigh, calf）
     */
    ros::Subscriber _servo_sub[12];
    
    /**
     * @brief IMU数据订阅者
     * 订阅"/trunk_imu"话题，接收机器人本体的姿态和运动数据
     */
    ros::Subscriber _imu_sub;
    
    /**
     * @brief 12个关节控制命令发布者数组
     * 每个发布者负责向一个关节控制器发送控制命令
     * 发布到话题：/{robot_name}_gazebo/{joint_name}_controller/command
     */
    ros::Publisher _servo_pub[12];
    
    // ==================== ROS消息缓存 ====================
    
    /**
     * @brief ROS底层控制命令消息对象
     * 用于缓存要发送给Gazebo的控制命令
     * 包含12个电机的控制参数（位置、速度、力矩、刚度、阻尼等）
     */
    unitree_legged_msgs::LowCmd _lowCmd;
    
    /**
     * @brief ROS底层状态反馈消息对象
     * 用于缓存从Gazebo接收的状态数据
     * 包含12个电机的状态和IMU数据
     */
    unitree_legged_msgs::LowState _lowState;
    
    /**
     * @brief 机器人名称字符串
     * 从ROS参数服务器读取，用于构建话题名称
     * 例如："a1_gazebo"、"go1_gazebo"等
     */
    std::string _robot_name;

    // ==================== 多线程初始化函数 ====================
    
    /**
     * @brief 初始化接收相关组件
     * 
     * 功能：
     * 1. 创建12个关节状态订阅者
     * 2. 创建IMU数据订阅者
     * 3. 绑定对应的回调函数
     * 
     * 话题命名规则：
     * - 关节状态：/{robot_name}_gazebo/{joint_name}_controller/state
     * - IMU数据：/trunk_imu
     */
    void initRecv();
    
    /**
     * @brief 初始化发送相关组件
     * 
     * 功能：
     * 1. 创建12个关节控制命令发布者
     * 2. 设置合适的队列长度和通信参数
     * 
     * 话题命名规则：
     * /{robot_name}_gazebo/{joint_name}_controller/command
     */
    void initSend();

    // ==================== ROS回调函数 ====================
    
    /**
     * @brief IMU数据回调函数
     * @param msg IMU传感器消息
     * 
     * 功能：
     * 1. 接收机器人本体的IMU数据
     * 2. 提取姿态四元数（w, x, y, z）
     * 3. 提取角速度（wx, wy, wz）
     * 4. 提取线性加速度（ax, ay, az）
     * 5. 将数据存储到_lowState.imu中
     */
    void imuCallback(const sensor_msgs::Imu & msg);

    // ==================== 12个关节状态回调函数 ====================
    // 这些回调函数负责接收各个关节的状态反馈，包括：
    // - 关节位置(q)
    // - 关节速度(dq) 
    // - 关节力矩估计(tauEst)
    // - 控制模式(mode)
    
    // FR腿（前右腿）关节回调函数
    /**
     * @brief 前右髋关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[0]的数据
     */
    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 前右大腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[1]的数据
     */
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 前右小腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[2]的数据
     */
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // FL腿（前左腿）关节回调函数
    /**
     * @brief 前左髋关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[3]的数据
     */
    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 前左大腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[4]的数据
     */
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 前左小腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[5]的数据
     */
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // RR腿（后右腿）关节回调函数
    /**
     * @brief 后右髋关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[6]的数据
     */
    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 后右大腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[7]的数据
     */
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 后右小腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[8]的数据
     */
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // RL腿（后左腿）关节回调函数
    /**
     * @brief 后左髋关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[9]的数据
     */
    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 后左大腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[10]的数据
     */
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    
    /**
     * @brief 后左小腿关节状态回调函数
     * @param msg 电机状态消息
     * 更新_lowState.motorState[11]的数据
     */
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS

/*
设计说明和关键特性：

1. 【接口抽象】
   - 继承IOInterface，提供统一的通信接口
   - 上层控制算法无需关心是仿真还是真实机器人
   - 实现了多态性，便于系统扩展

2. 【多线程通信】
   - 使用ros::AsyncSpinner实现异步消息处理
   - 保证500Hz控制频率下的实时性
   - 避免阻塞主控制循环

3. 【消息格式转换】
   - 内部使用LowlevelCmd/LowlevelState格式
   - ROS通信使用unitree_legged_msgs格式
   - 自动完成两种格式间的转换

4. 【话题命名规范】
   - 控制命令：/{robot_name}_gazebo/{joint}_controller/command
   - 状态反馈：/{robot_name}_gazebo/{joint}_controller/state  
   - IMU数据：/trunk_imu
   - 支持多机器人仿真（通过robot_name区分）

5. 【关节编号映射】
   - 0-2: FR腿 (Front Right)
   - 3-5: FL腿 (Front Left)
   - 6-8: RR腿 (Rear Right)  
   - 9-11: RL腿 (Rear Left)
   - 每条腿3个关节：hip, thigh, calf

6. 【错误处理】
   - 信号处理（SIGINT）实现优雅关闭
   - ROS节点生命周期管理
   - 内存资源自动清理

7. 【扩展性】
   - 条件编译支持（COMPILE_WITH_ROS）
   - 易于添加新的传感器或执行器
   - 模块化设计，便于维护
*/