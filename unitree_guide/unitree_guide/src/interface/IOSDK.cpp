/**********************************************************************
 * 文件名: IOSDK.cpp
 * 
 * 文件作用:
 * 该文件实现了与Unitree真实四足机器人硬件的底层通信接口，是控制系统与物理机器人之间的桥梁。
 * 主要功能包括：
 * 1. 通过UDP协议与机器人底层控制器进行实时通信
 * 2. 发送电机控制命令（位置、速度、力矩、刚度参数）
 * 3. 接收机器人状态反馈（关节状态、IMU数据、手柄输入）
 * 4. 处理无线手柄输入命令
 * 5. 与ROS系统集成，发布关节状态信息（可选）
 * 
 * 设计思路:
 * - 继承自IOInterface抽象接口，实现统一的硬件抽象层
 * - 支持Go1和A1两种机器人型号的差异化配置
 * - 采用实时UDP通信确保控制的低延迟和高可靠性
 * - 集成安全机制和无线手柄控制
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 ***********************************************************************/

// 仅在编译真实机器人版本时包含此文件
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"         // IOSDK类定义
#include "interface/WirelessHandle.h" // 无线手柄处理类
#include <stdio.h>                   // 标准输入输出库

/**
 * @brief Go1机器人构造函数
 * 
 * 针对Go1机器人进行特定配置：
 * - 配置UDP通信参数（端口8090，目标IP: 192.168.123.10，目标端口8007）
 * - 初始化安全控制器
 * - 设置无线手柄输入处理
 * - 配置ROS关节状态发布（如果编译了MOVE_BASE）
 */
#ifdef ROBOT_TYPE_Go1
IOSDK::IOSDK()
    : _safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo),  // 初始化安全控制器，使用Aliengo类型
      _udp(UNITREE_LEGGED_SDK::LOWLEVEL,               // 设置为底层控制模式
           8090,                                       // 本地UDP端口号
           "192.168.123.10",                          // Go1机器人的IP地址
           8007)                                       // Go1机器人的UDP端口号
{
    std::cout << "The control interface for real robot" << std::endl;
    
    // 初始化底层命令数据结构，设置默认参数和安全值
    _udp.InitCmdData(_lowCmd);
    
    // 创建无线手柄处理对象，用于接收和解析手柄输入
    cmdPanel = new WirelessHandle();

    // 如果编译了MOVE_BASE支持，则初始化ROS发布器
#ifdef COMPILE_WITH_MOVE_BASE
    // 创建ROS发布器，用于发布机器人关节状态到ROS系统
    _pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
    
    // 预分配关节状态消息的内存空间（12个关节）
    _joint_state.name.resize(12);      // 关节名称数组
    _joint_state.position.resize(12);  // 关节位置数组
    _joint_state.velocity.resize(12);  // 关节速度数组
    _joint_state.effort.resize(12);    // 关节力矩数组
#endif  // COMPILE_WITH_MOVE_BASE
}
#endif

/**
 * @brief A1机器人构造函数
 * 
 * 针对A1机器人进行特定配置：
 * - 使用默认UDP通信参数（由SDK内部确定）
 * - 其他初始化步骤与Go1相同
 */
#ifdef ROBOT_TYPE_A1
IOSDK::IOSDK()
    : _safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo),  // 初始化安全控制器
      _udp(UNITREE_LEGGED_SDK::LOWLEVEL)               // 使用A1的默认UDP配置
{
    std::cout << "The control interface for real robot" << std::endl;
    
    // 初始化底层命令数据结构
    _udp.InitCmdData(_lowCmd);
    
    // 创建无线手柄处理对象
    cmdPanel = new WirelessHandle();

    // 如果编译了MOVE_BASE支持，则初始化ROS发布器
#ifdef COMPILE_WITH_MOVE_BASE
    // 创建ROS发布器，用于发布机器人关节状态
    _pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
    
    // 预分配关节状态消息的内存空间（12个关节）
    _joint_state.name.resize(12);      // 关节名称数组
    _joint_state.position.resize(12);  // 关节位置数组  
    _joint_state.velocity.resize(12);  // 关节速度数组
    _joint_state.effort.resize(12);    // 关节力矩数组
#endif  // COMPILE_WITH_MOVE_BASE
}
#endif

/**
 * @brief 主要通信函数：发送控制命令并接收机器人状态
 * 
 * 这是IOSDK的核心函数，实现了与机器人的双向实时通信。
 * 该函数在控制主循环中以500Hz的频率被调用，确保实时性能。
 * 
 * @param cmd 输入的控制命令指针，包含所有关节的控制参数
 * @param state 输出的状态反馈指针，用于返回机器人的实时状态
 * 
 * 执行流程：
 * 1. 将上层控制命令转换为SDK格式
 * 2. 通过UDP发送命令到机器人
 * 3. 接收机器人状态反馈
 * 4. 解析状态数据并处理手柄输入
 * 5. 发布ROS消息（可选）
 */
void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // ==================== 第一步：转换并发送控制命令 ====================
    // 遍历所有12个关节，将上层控制命令复制到SDK格式的数据结构
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;  // 控制模式（位置/速度/力矩模式）
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;     // 目标位置 [rad]
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;    // 目标速度 [rad/s]
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;    // 位置刚度增益
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;    // 速度阻尼增益
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;   // 前馈力矩 [Nm]
    }
    
    // 设置要发送的命令数据
    _udp.SetSend(_lowCmd);
    
    // 通过UDP协议发送命令到机器人底层控制器
    _udp.Send();

    // ==================== 第二步：接收机器人状态反馈 ====================
    // 从UDP接收数据包
    _udp.Recv();
    
    // 解析接收到的状态数据
    _udp.GetRecv(_lowState);

    // ==================== 第三步：解析关节状态数据 ====================
    // 遍历所有12个关节，提取状态信息
    for(int i(0); i < 12; ++i){
        state->motorState[i].q      = _lowState.motorState[i].q;      // 实际关节位置 [rad]
        state->motorState[i].dq     = _lowState.motorState[i].dq;     // 实际关节速度 [rad/s]
        state->motorState[i].ddq    = _lowState.motorState[i].ddq;    // 关节加速度 [rad/s²]
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst; // 估计关节力矩 [Nm]
        state->motorState[i].mode   = _lowState.motorState[i].mode;   // 当前控制模式
    }

    // ==================== 第四步：解析IMU传感器数据 ====================
    // 提取IMU的四元数、陀螺仪和加速度计数据
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i]    = _lowState.imu.quaternion[i];    // 姿态四元数的xyz分量
        state->imu.gyroscope[i]     = _lowState.imu.gyroscope[i];     // 角速度 [rad/s]
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i]; // 加速度 [m/s²]
    }
    // 四元数的w分量（标量部分）
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    // ==================== 第五步：处理无线手柄输入 ====================
    // 将底层状态数据传递给无线手柄处理器，解析手柄按键和摇杆输入
    cmdPanel->receiveHandle(&_lowState);
    
    // 获取解析后的用户命令和模拟量输入值
    state->userCmd   = cmdPanel->getUserCmd();    // 离散的按键命令（如：START, L2_A等）
    state->userValue = cmdPanel->getUserValue();  // 连续的摇杆值（如：lx, ly, rx, ry, L2触发量）

    // ==================== 第六步：发布ROS消息（可选功能） ====================
#ifdef COMPILE_WITH_MOVE_BASE
    // 设置消息时间戳为当前时间
    _joint_state.header.stamp = ros::Time::now();
    
    // 设置关节名称，按照ROS机器人描述文件的标准命名
    // FR=前右, FL=前左, RR=后右, RL=后左
    // 每条腿3个关节：hip(髋关节), thigh(大腿), calf(小腿)
    _joint_state.name = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",    // 前右腿
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",    // 前左腿
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",    // 后右腿
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"     // 后左腿
    };
    
    // 填充关节状态数据
    for(int i(0); i < 12; ++i){
        _joint_state.position[i] = state->motorState[i].q;      // 关节位置 [rad]
        _joint_state.velocity[i] = state->motorState[i].dq;     // 关节速度 [rad/s]
        _joint_state.effort[i]   = state->motorState[i].tauEst; // 关节力矩 [Nm]
    }

    // 发布关节状态消息到ROS话题，供其他节点（如rviz可视化）使用
    _pub.publish(_joint_state);
#endif  // COMPILE_WITH_MOVE_BASE
}

#endif  // COMPILE_WITH_REAL_ROBOT