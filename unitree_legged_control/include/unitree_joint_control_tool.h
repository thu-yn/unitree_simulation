/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// 防止头文件重复包含的宏定义
// 注意：此处同时定义了两个宏，可能是为了兼容性考虑
// #ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
// #define _UNITREE_JOINT_CONTROL_TOOL_H_
#ifndef _LAIKAGO_CONTROL_TOOL_H_
#define _LAIKAGO_CONTROL_TOOL_H_

// 标准C库
#include <stdio.h>      // 标准输入输出
#include <stdint.h>     // 标准整数类型定义
#include <algorithm>    // STL算法库
#include <math.h>       // 数学函数库

// 控制停止标志定义 - 与joint_controller.h中的定义保持一致
#define posStopF (2.146E+9f)  // 位置停止控制模式标志值
#define velStopF (16000.0f)   // 速度停止控制模式标志值

/**
* @brief 伺服控制命令结构体
* 
* 这个结构体定义了发送给单个关节的完整控制命令
* 支持位置、速度、力矩的复合控制模式
*/
typedef struct 
{
uint8_t mode;           // 控制模式 - PMSM(0x0A)正常模式, BRAKE(0x00)制动模式
double pos;             // 目标位置 (弧度) - 关节的期望角度位置
double posStiffness;    // 位置刚度系数 (Kp) - 位置控制的比例增益
double vel;             // 目标速度 (弧度/秒) - 关节的期望角速度
double velStiffness;    // 速度刚度系数 (Kd) - 速度控制的阻尼增益  
double torque;          // 前馈力矩 (N·m) - 直接施加的力矩命令
} ServoCmd;

/**
* @brief 数值限幅函数
* 
* 将输入值限制在指定范围内，用于安全保护
* 
* @param value 待限制的数值（引用传递，会被修改）
* @param min_val 最小允许值
* @param max_val 最大允许值
* @return 限制后的数值
* 
* 例如：clamp(1.5, -1, 1) = 1
*/
double clamp(double&, double, double);

/**
* @brief 计算当前速度
* 
* 基于位置差分和上次速度进行滤波计算当前速度
* 用于平滑速度估计，减少噪声影响
* 
* @param current_position 当前位置
* @param last_position 上次位置  
* @param last_velocity 上次速度
* @param duration 时间间隔
* @return 计算得到的当前速度
*/
double computeVel(double current_position, double last_position, double last_velocity, double duration);

/**
* @brief 计算控制力矩
* 
* 基于当前状态和控制命令计算最终输出力矩
* 实现位置-速度-力矩的复合控制算法
* 
* @param current_position 当前关节位置
* @param current_velocity 当前关节速度
* @param servoCmd 伺服控制命令结构体
* @return 计算得到的控制力矩
* 
* 控制算法：tau = Kp*(pos_desired - pos_current) + Kd*(vel_desired - vel_current) + tau_feedforward
*/
double computeTorque(double current_position, double current_velocity, ServoCmd&);

#endif