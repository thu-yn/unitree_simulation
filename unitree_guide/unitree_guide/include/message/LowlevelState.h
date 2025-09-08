/**********************************************************************
 * 文件作用说明：
 * 本文件定义了Unitree四足机器人底层状态数据结构，包含：
 * 1. MotorState - 单个关节电机的状态信息（位置、速度、力矩等）
 * 2. IMU - 惯性测量单元数据（四元数、陀螺仪、加速度计）
 * 3. LowlevelState - 机器人完整底层状态的集合体
 * 
 * 这是整个控制系统中状态反馈的核心数据结构，用于：
 * - 从硬件或仿真环境获取机器人当前状态
 * - 为控制算法提供状态估计的基础数据
 * - 支持状态机进行状态转换决策
 * - 实现闭环控制和状态监控
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"      // 数学类型定义（Vec3, Mat3, RotMat等）
#include "common/mathTools.h"      // 数学工具函数（坐标变换等）
#include "interface/CmdPanel.h"    // 命令面板接口（用户命令和参数）
#include "common/enumClass.h"      // 枚举类定义

/**
 * @brief 单个关节电机状态结构体
 * 
 * 存储单个电机关节的完整状态信息，包括控制模式、位置、速度、加速度和力矩估计。
 * 这是机器人12个关节电机状态反馈的基本数据单元。
 * 
 * 关节编号规则：
 * - 每条腿3个关节：0-2(前右腿), 3-5(前左腿), 6-8(后右腿), 9-11(后左腿)
 * - 每条腿关节顺序：髋关节外展/内收、髋关节屈伸、膝关节屈伸
 */
struct MotorState
{
    unsigned int mode;      // 电机控制模式（位置模式、力矩模式等）
    float q;                // 关节角度位置 [rad] - 当前关节的绝对角度
    float dq;               // 关节角速度 [rad/s] - 关节旋转的瞬时角速度
    float ddq;              // 关节角加速度 [rad/s²] - 角速度的变化率（通常由数值微分得到）
    float tauEst;           // 估计的关节力矩 [N·m] - 电机输出的实际力矩估计值

    /**
     * @brief 默认构造函数
     * 将所有状态变量初始化为零，确保数据结构的确定性初始状态
     */
    MotorState(){
        q = 0;          // 初始角度为0
        dq = 0;         // 初始角速度为0
        ddq = 0;        // 初始角加速度为0
        tauEst = 0;     // 初始力矩估计为0
    }
};

/**
 * @brief 惯性测量单元(IMU)数据结构体
 * 
 * 包含机器人本体的姿态、角速度和线加速度信息。
 * IMU通常安装在机器人躯干中央，提供机器人在三维空间中的运动状态。
 * 这些数据对于平衡控制、状态估计和步态规划至关重要。
 */
struct IMU
{
    float quaternion[4];    // 四元数姿态表示 [w, x, y, z] - 机器人本体相对于世界坐标系的姿态
    float gyroscope[3];     // 陀螺仪数据 [rad/s] - 绕x,y,z轴的角速度（本体坐标系）
    float accelerometer[3]; // 加速度计数据 [m/s²] - 沿x,y,z轴的线加速度（包含重力）

    /**
     * @brief 默认构造函数
     * 初始化所有IMU数据为零，对应于静止状态的理想初始值
     */
    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;      // 四元数虚部置零
            gyroscope[i] = 0;       // 陀螺仪各轴角速度置零
            accelerometer[i] = 0;   // 加速度计各轴加速度置零
        }
        quaternion[3] = 0;          // 四元数实部置零（注意：实际使用时应为1）
    }

    /**
     * @brief 获取旋转矩阵
     * @return RotMat 3x3旋转矩阵，表示从机器人本体坐标系到世界坐标系的变换
     * 
     * 将四元数转换为旋转矩阵，用于坐标系变换。
     * 旋转矩阵可以将本体坐标系中的向量变换到世界坐标系。
     */
    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);  // 调用数学工具函数进行四元数到旋转矩阵的转换
    }

    /**
     * @brief 获取加速度向量
     * @return Vec3 三维加速度向量 [m/s²]
     * 
     * 将加速度计数组数据转换为Eigen向量格式，便于数学运算
     */
    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    /**
     * @brief 获取角速度向量
     * @return Vec3 三维角速度向量 [rad/s]
     * 
     * 将陀螺仪数组数据转换为Eigen向量格式，便于数学运算
     */
    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    /**
     * @brief 获取四元数
     * @return Quat 四元数对象 [w, x, y, z]
     * 
     * 将四元数数组数据转换为Eigen四元数格式，便于姿态相关计算
     */
    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

/**
 * @brief 机器人底层状态完整数据结构体
 * 
 * 这是整个控制系统的核心状态数据结构，集成了：
 * - IMU姿态和运动信息
 * - 12个关节的完整状态
 * - 用户命令和参数
 * 
 * 该结构体在控制循环中被频繁使用，是状态估计、控制决策和安全监控的数据基础。
 * 通过IOInterface从硬件或仿真环境获取，并传递给控制框架的各个模块。
 */
struct LowlevelState
{
    IMU imu;                    // 惯性测量单元数据
    MotorState motorState[12];  // 12个关节电机状态数组（4条腿×3个关节）
    UserCommand userCmd;        // 用户命令（来自键盘、手柄或ROS话题）
    UserValue userValue;        // 用户参数（步态参数、控制增益等）

    /**
     * @brief 获取所有关节位置矩阵
     * @return Vec34 3×4矩阵，每列代表一条腿的3个关节角度 [rad]
     * 
     * 矩阵格式：
     * [q_hip_abduction_0,  q_hip_abduction_1,  q_hip_abduction_2,  q_hip_abduction_3 ]
     * [q_hip_flexion_0,    q_hip_flexion_1,    q_hip_flexion_2,    q_hip_flexion_3   ]
     * [q_knee_flexion_0,   q_knee_flexion_1,   q_knee_flexion_2,   q_knee_flexion_3  ]
     * 
     * 腿的编号：0=前右, 1=前左, 2=后右, 3=后左
     */
    Vec34 getQ(){
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){  // 遍历4条腿
            qLegs.col(i)(0) = motorState[3*i    ].q;  // 髋关节外展/内收角度
            qLegs.col(i)(1) = motorState[3*i + 1].q;  // 髋关节屈伸角度
            qLegs.col(i)(2) = motorState[3*i + 2].q;  // 膝关节屈伸角度
        }
        return qLegs;
    }

    /**
     * @brief 获取所有关节速度矩阵
     * @return Vec34 3×4矩阵，每列代表一条腿的3个关节角速度 [rad/s]
     * 
     * 矩阵格式与getQ()相同，但包含的是角速度信息。
     * 用于速度控制、阻尼控制和动力学计算。
     */
    Vec34 getQd(){
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){  // 遍历4条腿
            qdLegs.col(i)(0) = motorState[3*i    ].dq;  // 髋关节外展/内收角速度
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;  // 髋关节屈伸角速度
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;  // 膝关节屈伸角速度
        }
        return qdLegs;
    }

    /**
     * @brief 获取机器人本体旋转矩阵
     * @return RotMat 3×3旋转矩阵，从本体坐标系到世界坐标系的变换
     * 
     * 这是IMU数据的封装访问，用于坐标系变换。
     * 在平衡控制和足端轨迹规划中频繁使用。
     */
    RotMat getRotMat(){
        return imu.getRotMat();
    }

    /**
     * @brief 获取机器人本体加速度（本体坐标系）
     * @return Vec3 三维加速度向量 [m/s²]
     * 
     * 返回在机器人本体坐标系中测量的加速度，包含重力分量。
     */
    Vec3 getAcc(){
        return imu.getAcc();
    }

    /**
     * @brief 获取机器人本体角速度（本体坐标系）
     * @return Vec3 三维角速度向量 [rad/s]
     * 
     * 返回机器人本体在三个轴向的旋转角速度。
     */
    Vec3 getGyro(){
        return imu.getGyro();
    }

    /**
     * @brief 获取机器人在世界坐标系中的加速度
     * @return Vec3 世界坐标系中的加速度向量 [m/s²]
     * 
     * 通过旋转矩阵将本体坐标系中的加速度变换到世界坐标系。
     * 用于重心控制和动力学分析。
     */
    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();  // 旋转矩阵左乘向量实现坐标变换
    }

    /**
     * @brief 获取机器人在世界坐标系中的角速度
     * @return Vec3 世界坐标系中的角速度向量 [rad/s]
     * 
     * 通过旋转矩阵将本体坐标系中的角速度变换到世界坐标系。
     */
    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();  // 旋转矩阵左乘向量实现坐标变换
    }

    /**
     * @brief 获取机器人航向角（Yaw角）
     * @return double 航向角 [rad]，绕Z轴的旋转角度
     * 
     * 从旋转矩阵中提取航向角，这是机器人在水平面内的朝向角度。
     * 用于路径跟踪和方向控制。
     */
    double getYaw(){
        return rotMatToRPY(getRotMat())(2);  // 提取Roll-Pitch-Yaw中的Yaw分量
    }

    /**
     * @brief 获取航向角变化率（Yaw角速度）
     * @return double 航向角速度 [rad/s]
     * 
     * 世界坐标系中绕Z轴的角速度，即航向角的变化率。
     * 用于转向控制和航向稳定。
     */
    double getDYaw(){
        return getGyroGlobal()(2);  // 世界坐标系中Z轴角速度分量
    }

    /**
     * @brief 设置所有关节位置
     * @param q Vec12 包含12个关节角度的向量 [rad]
     * 
     * 将关节角度向量设置到各个电机状态中。
     * 向量布局：[腿0的3个关节, 腿1的3个关节, 腿2的3个关节, 腿3的3个关节]
     * 主要用于状态初始化或测试场景。
     */
    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);  // 按顺序设置每个电机的位置
        }
    }
};

#endif  //LOWLEVELSTATE_HPP