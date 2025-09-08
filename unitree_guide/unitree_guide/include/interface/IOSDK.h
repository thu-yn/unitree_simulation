/**********************************************************************
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 ***********************************************************************/

/**
 * @file IOSDK.h
 * @brief 真实机器人SDK通信接口头文件
 * 
 * 文件作用：
 * IOSDK类是unitree_guide控制框架中与真实Unitree四足机器人硬件通信的核心接口。
 * 它继承自IOInterface抽象基类，实现了通过Unitree官方SDK与机器人进行UDP通信的功能。
 * 该接口负责：
 * 1. 将控制框架的LowlevelCmd命令转换为SDK格式并发送给机器人
 * 2. 接收机器人的状态数据并转换为控制框架使用的LowlevelState格式
 * 3. 处理无线手柄输入和安全检查
 * 4. 可选地发布ROS关节状态话题用于导航系统集成
 * 
 * 这是连接高层控制算法与真实机器人硬件的关键桥梁，确保控制命令能够
 * 准确、实时地传递给机器人，同时获取机器人的真实状态反馈。
 */

#ifndef IOSDK_H
#define IOSDK_H

// 引入抽象接口基类
#include "interface/IOInterface.h"
// 引入Unitree官方SDK
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// 如果编译时启用了move_base导航功能，则包含ROS相关头文件
#ifdef COMPILE_WITH_MOVE_BASE
    #include <ros/ros.h>           // ROS核心功能
    #include <ros/time.h>          // ROS时间处理
    #include <sensor_msgs/JointState.h>  // ROS关节状态消息类型
#endif  // COMPILE_WITH_MOVE_BASE

/**
 * @class IOSDK
 * @brief 真实机器人SDK通信接口类
 * 
 * 该类继承自IOInterface抽象基类，专门用于与真实的Unitree四足机器人进行通信。
 * 它封装了Unitree官方SDK的UDP通信协议，提供了统一的接口给上层控制框架使用。
 * 
 * 主要功能：
 * - UDP通信：通过以太网与机器人进行实时数据交换
 * - 数据转换：在控制框架消息格式和SDK消息格式之间进行转换
 * - 安全检查：集成SDK的安全机制，确保控制命令的安全性
 * - 手柄处理：处理无线手柄输入，支持手动控制和模式切换
 * - ROS集成：可选地发布关节状态用于ROS导航系统
 */
class IOSDK : public IOInterface{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化SDK通信接口，包括：
     * - 根据机器人类型（A1或Go1）配置UDP连接参数
     * - 初始化安全检查模块
     * - 设置无线手柄处理
     * - 如果启用ROS功能，则设置关节状态发布器
     * 
     * 网络配置：
     * - Go1机器人：连接到192.168.123.10:8007，本地端口8090
     * - A1机器人：使用默认SDK配置
     */
    IOSDK();
    
    /**
     * @brief 析构函数
     * 
     * 清理资源，SDK对象会自动处理网络连接的关闭
     */
    ~IOSDK(){}
    
    /**
     * @brief 发送控制命令并接收状态反馈
     * @param cmd 指向待发送的底层控制命令的常量指针
     * @param state 指向用于存储接收状态数据的对象指针
     * 
     * 这是IOSDK的核心功能函数，执行一个完整的通信周期：
     * 
     * 发送阶段：
     * 1. 将LowlevelCmd中的12个关节控制命令转换为SDK格式
     * 2. 复制控制模式、目标位置、速度、PD增益、力矩等参数
     * 3. 通过UDP发送给机器人
     * 
     * 接收阶段：
     * 1. 从机器人接收状态数据包
     * 2. 解析12个关节的实际位置、速度、力矩估计等
     * 3. 解析IMU数据（四元数姿态、角速度、加速度）
     * 4. 处理无线手柄输入
     * 5. 如果启用ROS功能，发布关节状态话题
     * 
     * 该函数通常在500Hz的实时控制循环中被调用
     */
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    // ==================== SDK核心组件 ====================
    
    /**
     * @brief UDP通信对象
     * 
     * 负责与机器人进行以太网UDP通信的核心对象。
     * 配置为LOWLEVEL模式，直接控制关节级别的运动。
     * 
     * 通信参数：
     * - 协议：UDP
     * - 控制级别：LOWLEVEL（关节级控制）
     * - 通信频率：支持高达1000Hz
     * - 数据包大小：优化后的二进制格式
     */
    UNITREE_LEGGED_SDK::UDP _udp;
    
    /**
     * @brief 安全检查对象
     * 
     * Unitree SDK内置的安全检查机制，用于：
     * - 验证控制命令的合法性（关节角度、速度、力矩限制）
     * - 检测异常情况并触发保护机制
     * - 防止危险的控制指令损坏机器人
     * 
     * 安全限制包括：
     * - 关节角度限制：防止关节超出机械限位
     * - 速度限制：防止关节运动过快
     * - 力矩限制：防止电机过载
     */
    UNITREE_LEGGED_SDK::Safety _safe;
    
    /**
     * @brief SDK格式的底层控制命令
     * 
     * 用于存储转换为SDK格式的控制命令数据。
     * 包含12个关节（每条腿3个关节）的控制参数：
     * - mode: 控制模式（位置、速度、力矩或混合模式）
     * - q: 目标关节角度（弧度）
     * - dq: 目标关节角速度（弧度/秒）
     * - tau: 前馈力矩（牛顿·米）
     * - Kp: 位置比例增益
     * - Kd: 速度微分增益
     */
    UNITREE_LEGGED_SDK::LowCmd _lowCmd;
    
    /**
     * @brief SDK格式的底层状态反馈
     * 
     * 用于存储从机器人接收的状态数据。
     * 包含：
     * - 12个关节的状态：位置、速度、加速度、估计力矩、运行模式
     * - IMU数据：四元数姿态、三轴角速度、三轴加速度
     * - 足端接触力（如果有传感器）
     * - 电池电压、温度等系统状态
     */
    UNITREE_LEGGED_SDK::LowState _lowState;

// 如果编译时启用了move_base导航功能，则包含ROS相关成员变量
#ifdef COMPILE_WITH_MOVE_BASE
    /**
     * @brief ROS节点句柄
     * 
     * 用于ROS功能的节点句柄，管理ROS通信。
     */
    ros::NodeHandle _nh;
    
    /**
     * @brief 关节状态发布器
     * 
     * 发布sensor_msgs::JointState消息到ROS话题系统。
     * 话题名称："/realRobot/joint_states"
     * 消息频率：与控制频率同步（500Hz）
     * 
     * 用途：
     * - 为ROS导航系统提供机器人关节状态
     * - 支持rviz可视化
     * - 与move_base等导航包集成
     */
    ros::Publisher _pub;
    
    /**
     * @brief ROS关节状态消息对象
     * 
     * 存储要发布的关节状态数据，包括：
     * - name: 关节名称数组（12个关节）
     * - position: 关节位置数组（弧度）
     * - velocity: 关节速度数组（弧度/秒）
     * - effort: 关节力矩数组（牛顿·米）
     * 
     * 关节命名规则：
     * - FR_hip_joint, FR_thigh_joint, FR_calf_joint (右前腿)
     * - FL_hip_joint, FL_thigh_joint, FL_calf_joint (左前腿)  
     * - RR_hip_joint, RR_thigh_joint, RR_calf_joint (右后腿)
     * - RL_hip_joint, RL_thigh_joint, RL_calf_joint (左后腿)
     */
    sensor_msgs::JointState _joint_state;
#endif  // COMPILE_WITH_MOVE_BASE
};

#endif  // IOSDK_H