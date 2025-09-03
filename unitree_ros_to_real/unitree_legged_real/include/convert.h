/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

// ROS消息类型头文件
#include "unitree_legged_msgs/BmsCmd.h"       // 电池管理系统命令
#include "unitree_legged_msgs/BmsState.h"     // 电池管理系统状态
#include "unitree_legged_msgs/Cartesian.h"    // 笛卡尔坐标
#include "unitree_legged_msgs/HighCmd.h"      // 高层控制命令
#include "unitree_legged_msgs/HighState.h"    // 高层状态反馈
#include "unitree_legged_msgs/IMU.h"          // 惯性测量单元数据
#include "unitree_legged_msgs/LED.h"          // LED控制
#include "unitree_legged_msgs/LowCmd.h"       // 底层控制命令
#include "unitree_legged_msgs/LowState.h"     // 底层状态反馈
#include "unitree_legged_msgs/MotorCmd.h"     // 电机控制命令
#include "unitree_legged_msgs/MotorState.h"   // 电机状态反馈

// Unitree SDK头文件
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "ros/ros.h"

// =============================================================================
// ROS消息 → Unitree SDK命令 转换函数
// 功能：将ROS标准消息格式转换为Unitree SDK可识别的命令格式
// =============================================================================

/**
 * @brief 转换电池管理系统命令：ROS消息 → SDK命令
 * @param msg ROS电池管理命令消息
 * @return Unitree SDK电池管理命令结构体
 */
UNITREE_LEGGED_SDK::BmsCmd rosMsg2Cmd(const unitree_legged_msgs::BmsCmd &msg)
{
    UNITREE_LEGGED_SDK::BmsCmd cmd;

    // 复制关机控制标志
    cmd.off = msg.off;

    // 复制保留字段（用于未来扩展）
    for (std::size_t i(0); i < 3; i++)
        cmd.reserve[i] = msg.reserve[i];

    return cmd;
}

/**
 * @brief 转换LED控制命令：ROS消息 → SDK命令  
 * @param msg ROS LED控制消息
 * @return Unitree SDK LED控制结构体
 */
UNITREE_LEGGED_SDK::LED rosMsg2Cmd(const unitree_legged_msgs::LED &msg)
{
    UNITREE_LEGGED_SDK::LED cmd;
    
    // 复制RGB颜色值
    cmd.r = msg.r;  // 红色分量
    cmd.g = msg.g;  // 绿色分量  
    cmd.b = msg.b;  // 蓝色分量

    return cmd;
}

/**
 * @brief 转换电机控制命令：ROS消息 → SDK命令
 * @param msg ROS电机控制消息
 * @return Unitree SDK电机控制结构体
 */
UNITREE_LEGGED_SDK::MotorCmd rosMsg2Cmd(const unitree_legged_msgs::MotorCmd &msg)
{
    UNITREE_LEGGED_SDK::MotorCmd cmd;

    // 复制电机控制参数
    cmd.mode = msg.mode;    // 控制模式（位置/速度/力矩）
    cmd.q = msg.q;          // 目标位置 (rad)
    cmd.dq = msg.dq;        // 目标速度 (rad/s)
    cmd.tau = msg.tau;      // 前馈力矩 (N·m)
    cmd.Kp = msg.Kp;        // 位置增益
    cmd.Kd = msg.Kd;        // 速度增益

    // 复制保留字段
    for (std::size_t i(0); i < 3; i++)
    {
        cmd.reserve[i] = msg.reserve[i];
    }

    return cmd;
}

/**
 * @brief 转换高层控制命令：ROS消息 → SDK命令
 * @param msg ROS高层控制消息
 * @return Unitree SDK高层控制结构体
 */
UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_legged_msgs::HighCmd &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    // 复制2字节的头部和序列号信息
    for (std::size_t i(0); i < 2; i++)
    {
        cmd.head[i] = msg.head[i];        // 数据包头标识
        cmd.SN[i] = msg.SN[i];            // 序列号
        cmd.version[i] = msg.version[i];  // 版本信息
        cmd.position[i] = msg.position[i]; // 目标位置(x,y)
        cmd.velocity[i] = msg.velocity[i]; // 目标速度(vx,vy)
        cmd.dComXy[i] = msg.dComXy[i];    // 质心位移
        cmd.dstandFootXy[i] = msg.dstandFootXy[i]; // 站立足位移
    }

    // 复制3字节的欧拉角信息
    for (std::size_t i(0); i < 3; i++)
        cmd.euler[i] = msg.euler[i];  // 目标姿态角(roll, pitch, yaw)

    // 复制4个LED控制信息
    for (std::size_t i(0); i < 4; i++)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);

    // 复制40字节的无线遥控器数据
    for (std::size_t i(0); i < 40; i++)
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];

    // 复制控制标志和参数
    cmd.levelFlag = msg.levelFlag;           // 控制级别标志
    cmd.frameReserve = msg.frameReserve;     // 帧保留字段
    cmd.bandWidth = msg.bandWidth;           // 带宽设置
    cmd.mode = msg.mode;                     // 运动模式
    cmd.gaitType = msg.gaitType;             // 步态类型
    cmd.speedLevel = msg.speedLevel;         // 速度等级
    cmd.footRaiseHeight = msg.footRaiseHeight; // 足部抬起高度
    cmd.bodyHeight = msg.bodyHeight;         // 身体高度
    cmd.yawSpeed = msg.yawSpeed;             // 偏航角速度
    cmd.reserve = msg.reserve;               // 保留字段
    cmd.crc = msg.crc;                       // 循环冗余校验

    // 转换电池管理命令
    cmd.bms = rosMsg2Cmd(msg.bms);

    return cmd;
}

/**
 * @brief 转换底层控制命令：ROS消息 → SDK命令
 * @param msg ROS底层控制消息
 * @return Unitree SDK底层控制结构体
 */
UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    // 复制头部信息
    for (std::size_t i(0); i < 2; i++)
    {
        cmd.head[i] = msg.head[i];        // 数据包头
        cmd.SN[i] = msg.SN[i];            // 序列号
        cmd.version[i] = msg.version[i];  // 版本信息
    }

    // 复制无线遥控器数据
    for (std::size_t i(0); i < 40; i++)
    {
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];
    }

    // 转换20个电机的控制命令
    for (std::size_t i(0); i < 20; i++)
    {
        cmd.motorCmd[i] = rosMsg2Cmd(msg.motorCmd[i]);
    }

    // 转换电池管理命令
    cmd.bms = rosMsg2Cmd(msg.bms);

    // 复制控制标志和参数
    cmd.levelFlag = msg.levelFlag;       // 控制级别标志
    cmd.frameReserve = msg.frameReserve; // 帧保留字段
    cmd.bandWidth = msg.bandWidth;       // 带宽设置
    cmd.reserve = msg.reserve;           // 保留字段
    cmd.crc = msg.crc;                   // 循环冗余校验

    return cmd;
}

// =============================================================================
// Unitree SDK状态 → ROS消息 转换函数
// 功能：将Unitree SDK状态数据转换为ROS标准消息格式，供ROS节点使用
// =============================================================================

/**
 * @brief 转换电池管理系统状态：SDK状态 → ROS消息
 * @param state Unitree SDK电池状态结构体
 * @return ROS电池状态消息
 */
unitree_legged_msgs::BmsState state2rosMsg(UNITREE_LEGGED_SDK::BmsState &state)
{
    unitree_legged_msgs::BmsState ros_msg;

    // 复制8个温度传感器数据
    for (std::size_t i(0); i < 8; i++)
    {
        ros_msg.BQ_NTC[i] = state.BQ_NTC[i];   // BQ芯片温度传感器
        ros_msg.MCU_NTC[i] = state.MCU_NTC[i]; // MCU温度传感器
    }

    // 复制30个电芯电压数据
    for (std::size_t i(0); i < 30; i++)
        ros_msg.cell_vol[i] = state.cell_vol[i];

    // 复制电池管理系统基本信息
    ros_msg.version_h = state.version_h;     // 版本号高位
    ros_msg.version_l = state.version_l;     // 版本号低位
    ros_msg.bms_status = state.bms_status;   // BMS状态
    ros_msg.SOC = state.SOC;                 // 电量百分比
    ros_msg.current = state.current;         // 当前电流
    ros_msg.cycle = state.cycle;             // 充放电循环次数

    return ros_msg;
}

/**
 * @brief 转换笛卡尔坐标：SDK状态 → ROS消息
 * @param state Unitree SDK笛卡尔坐标结构体
 * @return ROS笛卡尔坐标消息
 */
unitree_legged_msgs::Cartesian state2rosMsg(UNITREE_LEGGED_SDK::Cartesian &state)
{
    unitree_legged_msgs::Cartesian ros_msg;

    // 复制三维坐标
    ros_msg.x = state.x;  // X坐标
    ros_msg.y = state.y;  // Y坐标
    ros_msg.z = state.z;  // Z坐标

    return ros_msg;
}

/**
 * @brief 转换IMU数据：SDK状态 → ROS消息
 * @param state Unitree SDK IMU状态结构体
 * @return ROS IMU消息
 */
unitree_legged_msgs::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    unitree_legged_msgs::IMU ros_msg;

    // 复制四元数姿态信息
    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.quaternion[i] = state.quaternion[i];  // 四元数(w,x,y,z)
    }

    // 复制三轴传感器数据
    for (std::size_t i(0); i < 3; i++)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];       // 陀螺仪数据(rad/s)
        ros_msg.accelerometer[i] = state.accelerometer[i]; // 加速度计数据(m/s²)
        ros_msg.rpy[i] = state.rpy[i];                   // 欧拉角(roll,pitch,yaw)
    }

    // 复制温度信息
    ros_msg.temperature = state.temperature;  // IMU传感器温度

    return ros_msg;
}

/**
 * @brief 转换电机状态：SDK状态 → ROS消息
 * @param state Unitree SDK电机状态结构体
 * @return ROS电机状态消息
 */
unitree_legged_msgs::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    unitree_legged_msgs::MotorState ros_msg;

    // 复制电机基本状态
    ros_msg.mode = state.mode;         // 控制模式
    ros_msg.q = state.q;               // 当前位置 (rad)
    ros_msg.dq = state.dq;             // 当前速度 (rad/s)
    ros_msg.ddq = state.ddq;           // 当前加速度 (rad/s²)
    ros_msg.tauEst = state.tauEst;     // 估计力矩 (N·m)
    
    // 复制原始传感器数据
    ros_msg.q_raw = state.q_raw;       // 原始位置数据
    ros_msg.dq_raw = state.dq_raw;     // 原始速度数据
    ros_msg.ddq_raw = state.ddq_raw;   // 原始加速度数据
    ros_msg.temperature = state.temperature; // 电机温度

    // 复制保留字段
    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

/**
 * @brief 转换高层状态：SDK状态 → ROS消息
 * @param state Unitree SDK高层状态结构体
 * @return ROS高层状态消息
 */
unitree_legged_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_legged_msgs::HighState ros_msg;

    // 复制头部信息
    for (std::size_t i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];        // 数据包头
        ros_msg.SN[i] = state.SN[i];            // 序列号
        ros_msg.version[i] = state.version[i];  // 版本信息
    }

    // 复制四足相关信息
    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.footForce[i] = state.footForce[i];         // 足端接触力
        ros_msg.footForceEst[i] = state.footForceEst[i];   // 足端力估计值
        ros_msg.rangeObstacle[i] = state.rangeObstacle[i]; // 障碍物距离
        ros_msg.footPosition2Body[i] = state2rosMsg(state.footPosition2Body[i]); // 足端相对身体位置
        ros_msg.footSpeed2Body[i] = state2rosMsg(state.footSpeed2Body[i]);       // 足端相对身体速度
    }

    // 复制机器人整体运动状态
    for (std::size_t i(0); i < 3; i++)
    {
        ros_msg.position[i] = state.position[i];  // 机器人位置(x,y,z)
        ros_msg.velocity[i] = state.velocity[i];  // 机器人速度(vx,vy,vz)
    }

    // 复制无线遥控器数据
    for (std::size_t i(0); i < 40; i++)
    {
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    }

    // 转换20个电机的状态信息
    for (std::size_t i(0); i < 20; i++)
    {
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    }

    // 转换传感器数据
    ros_msg.imu = state2rosMsg(state.imu);  // IMU数据
    ros_msg.bms = state2rosMsg(state.bms);  // 电池管理数据

    // 复制控制和状态标志
    ros_msg.levelFlag = state.levelFlag;         // 控制级别标志
    ros_msg.frameReserve = state.frameReserve;   // 帧保留字段
    ros_msg.bandWidth = state.bandWidth;         // 带宽
    ros_msg.mode = state.mode;                   // 运动模式
    ros_msg.progress = state.progress;           // 运动进度
    ros_msg.gaitType = state.gaitType;           // 步态类型
    ros_msg.footRaiseHeight = state.footRaiseHeight; // 足部抬起高度
    ros_msg.bodyHeight = state.bodyHeight;       // 身体高度
    ros_msg.yawSpeed = state.yawSpeed;           // 偏航角速度
    ros_msg.reserve = state.reserve;             // 保留字段
    ros_msg.crc = state.crc;                     // 循环冗余校验

    return ros_msg;
}

/**
 * @brief 转换底层状态：SDK状态 → ROS消息
 * @param state Unitree SDK底层状态结构体
 * @return ROS底层状态消息
 */
unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    // 复制头部信息
    for (std::size_t i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];        // 数据包头
        ros_msg.SN[i] = state.SN[i];            // 序列号
        ros_msg.version[i] = state.version[i];  // 版本信息
    }

    // 复制足端力信息
    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.footForce[i] = state.footForce[i];       // 足端接触力实际值
        ros_msg.footForceEst[i] = state.footForceEst[i]; // 足端接触力估计值
    }

    // 复制无线遥控器数据
    for (std::size_t i(0); i < 40; i++)
    {
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    }

    // 转换20个电机的状态信息
    for (std::size_t i(0); i < 20; i++)
    {
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    }

    // 转换传感器数据
    ros_msg.imu = state2rosMsg(state.imu);  // IMU数据
    ros_msg.bms = state2rosMsg(state.bms);  // 电池管理数据

    // 复制时间戳和标志信息
    ros_msg.tick = state.tick;               // 时间戳
    ros_msg.reserve = state.reserve;         // 保留字段
    ros_msg.crc = state.crc;                 // 循环冗余校验

    return ros_msg;
}

#endif // _CONVERT_H_