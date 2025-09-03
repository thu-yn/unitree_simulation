/************************************************************************
* 文件名: body.h
* 功能描述: Unitree四足机器人本体控制头文件
* 
* 主要功能:
* 1. 定义机器人本体控制的核心函数接口
* 2. 声明全局ROS发布器和消息结构体
* 3. 提供运动控制函数的外部接口
* 4. 定义控制参数的常量值
* 
* 在整个项目中的作用:
* - 作为机器人基础控制功能的接口层
* - 被servo.cpp等程序调用以实现具体的控制逻辑
* - 封装了机器人的基本运动控制算法
* 
* Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
* Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"      // 底层控制命令消息
#include "unitree_legged_msgs/LowState.h"    // 底层状态反馈消息
#include "unitree_legged_msgs/HighState.h"   // 高层状态消息

// 位置停止标志 - 用于表示关节位置控制停止
#define PosStopF (2.146E+9f)
// 速度停止标志 - 用于表示关节速度控制停止
#define VelStopF (16000.f)

/**
* @namespace unitree_model
* @brief Unitree机器人模型控制命名空间
* 
* 包含所有与机器人本体控制相关的全局变量和函数
* 避免命名冲突，提供清晰的代码组织结构
*/
namespace unitree_model {

// ==================== 全局变量声明 ====================

/**
* @brief 12个关节的ROS发布器数组
* 
* 索引映射关系:
* [0-2]:  FR_hip, FR_thigh, FR_calf    (前右腿)
* [3-5]:  FL_hip, FL_thigh, FL_calf    (前左腿)
* [6-8]:  RR_hip, RR_thigh, RR_calf    (后右腿)
* [9-11]: RL_hip, RL_thigh, RL_calf    (后左腿)
*/
extern ros::Publisher servo_pub[12];

/**
* @brief 高层状态发布器
* 用于发布机器人的高层状态信息，如整体姿态、运动状态等
*/
extern ros::Publisher highState_pub;

/**
* @brief 底层控制命令
* 包含12个关节的位置、速度、力矩控制命令
* 以及PD控制器参数(Kp, Kd)和控制模式设置
*/
extern unitree_legged_msgs::LowCmd lowCmd;

/**
* @brief 底层状态反馈
* 包含12个关节的实际位置、速度、力矩状态
* 以及IMU传感器数据、足端接触力等信息
*/
extern unitree_legged_msgs::LowState lowState;

// ==================== 函数接口声明 ====================

/**
* @brief 机器人站立控制函数
* 
* 功能描述:
* - 将机器人从任意姿态控制到标准站立姿态
* - 使用预定义的关节角度组合
* - 采用平滑过渡方式，避免突然运动
* 
* 站立姿态参数:
* - 髋关节(Hip): 0.0 rad (保持身体对称)
* - 大腿关节(Thigh): 0.67 rad (约38.4°)
* - 小腿关节(Calf): -1.3 rad (约-74.5°)
* 
* 调用时机:
* - 系统初始化后
* - 从倒地状态恢复
* - 执行复杂动作前的准备姿态
*/
void stand();

/**
* @brief 运动系统初始化函数
* 
* 功能描述:
* - 初始化所有关节的PD控制器参数
* - 设置关节控制模式
* - 执行站立动作使机器人进入就绪状态
* 
* 执行流程:
* 1. 调用paramInit()设置控制参数
* 2. 调用stand()执行站立动作
* 
* 调用时机:
* - 程序启动时的必要初始化步骤
*/
void motion_init();

/**
* @brief 发送伺服控制命令函数
* 
* 功能描述:
* - 将lowCmd中的控制命令发送到所有12个关节
* - 执行ROS消息处理(spinOnce)
* - 添加适当延时以保证控制频率
* 
* 控制频率: 1000Hz (1ms间隔)
* 
* 调用时机:
* - 每次更新关节控制命令后
* - 在控制循环中周期性调用
*/
void sendServoCmd();

/**
* @brief 平滑运动控制函数
* 
* @param jointPositions 目标关节位置数组(12个关节的目标角度)
* @param duration 运动持续时间(毫秒)
* 
* 功能描述:
* - 从当前位置平滑过渡到目标位置
* - 使用线性插值确保运动的平滑性
* - 支持运行时安全中断(检查ros::ok())
* 
* 算法特点:
* - 线性插值: 避免关节运动突变
* - 实时控制: 每1ms发送一次控制命令
* - 安全检查: 支持中途停止
* 
* 应用场景:
* - 执行复杂的机器人动作序列
* - 从一个姿态平滑过渡到另一个姿态
* - 避免机器人运动过程中的冲击和振动
*/
void moveAllPosition(double* jointPositions, double duration);

} // namespace unitree_model

#endif // __BODY_H__