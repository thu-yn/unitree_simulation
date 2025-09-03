/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
* @file unitreeRobot.cpp
* @brief 宇树四足机器人整机模型实现文件
* 
* 该文件实现了完整的四足机器人模型，主要功能包括：
* 1. 整机运动学计算：协调四条腿的运动学解算
* 2. 全身逆运动学：由足端期望位置计算所有关节角度
* 3. 全身正运动学：由关节角度计算所有足端位置
* 4. 坐标系变换：支持身体、髋关节、全局坐标系间的转换
* 5. 机器人物理参数管理：质量、惯量、几何参数等
* 6. 具体机器人型号支持：A1机器人和Go1机器人的参数配置
* 
* 设计架构：
* - QuadrupedRobot: 四足机器人基类，提供通用接口
* - A1Robot: A1机器人具体实现，包含A1的物理参数
* - Go1Robot: Go1机器人具体实现，包含Go1的物理参数
* 
* 坐标系说明：
* - BODY坐标系：以机器人身体中心为原点，x轴向前，y轴向左，z轴向上
* - HIP坐标系：以各腿髋关节中心为原点的局部坐标系
* - GLOBAL坐标系：世界坐标系，考虑机器人姿态的全局坐标系
*/

#include "common/unitreeRobot.h"
#include <iostream>

/**
* @brief 获取参考足端位置（通常是右前腿）
* @param state 机器人底层状态
* @return 参考足端在身体坐标系中的位置向量
* 
* 该函数返回指定腿（默认为0号腿，即右前腿）的足端位置，
* 常用作整机运动学计算的参考点。
*/
Vec3 QuadrupedRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

/**
* @brief 计算相对足端位置向量
* @param state 机器人底层状态
* @return 4×3矩阵，每列表示一条腿足端相对于参考足端的位置向量
* 
* 该函数计算四条腿足端相对于参考足端的位置关系，
* 主要用于：
* 1. 步态分析中的足端协调
* 2. 平衡控制中的足端分布计算
* 3. 运动规划中的约束条件设定
* 
* 计算原理：
* vecXP[i] = 第i条腿足端位置 - 参考足端位置
*/
Vec34 QuadrupedRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);           // 获取参考足端位置
    Vec34 vecXP, qLegs;             // 相对位置向量矩阵和关节角度矩阵
    qLegs = state.getQ();           // 获取当前所有关节角度

    // 计算每条腿足端相对于参考足端的位置向量
    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}

/**
* @brief 全身逆运动学：由足端位置计算所有关节角度
* @param vecP 4×3矩阵，每列表示一条腿的足端期望位置
* @param frame 坐标系类型（BODY或HIP）
* @return 12维向量，包含四条腿共12个关节的角度
* 
* 该函数是整机运动控制的核心函数之一，用于：
* 1. 步态生成：根据足端轨迹规划计算关节角度
* 2. 位置控制：实现足端精确定位
* 3. 运动插值：在不同姿态间平滑过渡
* 
* 数据组织：
* - 输入：vecP矩阵，4列分别对应FR、FL、RR、RL四条腿
* - 输出：12维向量，每3个元素对应一条腿的3个关节角度
* - 顺序：[FR_abad, FR_hip, FR_knee, FL_abad, FL_hip, FL_knee, ...]
*/
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    
    // 遍历四条腿，分别计算每条腿的逆运动学
    for(int i(0); i < 4; ++i){
        // 将第i条腿的3个关节角度存储到结果向量的对应位置
        // segment(3*i, 3)表示从第3*i个元素开始的3个连续元素
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}

/**
* @brief 全身速度逆运动学：由足端位置和速度计算关节角速度
* @param pos 4×3矩阵，每列表示一条腿的足端位置
* @param vel 4×3矩阵，每列表示一条腿的足端期望速度
* @param frame 坐标系类型
* @return 12维向量，包含四条腿共12个关节的角速度
* 
* 该函数用于速度控制模式，实现足端速度跟踪：
* 1. 步态执行：跟踪足端轨迹的速度要求
* 2. 动态步行：实现平滑的腿部运动
* 3. 阻抗控制：结合力控制实现柔顺运动
*/
Vec12 QuadrupedRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    
    // 遍历四条腿，分别计算每条腿的速度逆运动学
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

/**
* @brief 全身逆动力学：由关节角度和足端力计算关节力矩
* @param q 12维关节角度向量
* @param feetForce 4×3矩阵，每列表示一条腿的足端受力
* @return 12维向量，包含四条腿共12个关节的输出力矩
* 
* 该函数是力控制的核心，用于：
* 1. 平衡控制：通过足端力分配维持机器人平衡
* 2. 接触控制：实现与环境的柔顺接触
* 3. 跳跃控制：产生推进力和着陆缓冲力
* 4. 阻抗控制：实现期望的机械阻抗特性
* 
* 计算原理：
* 使用雅可比转置将足端力映射到关节力矩空间
*/
Vec12 QuadrupedRobot::getTau(const Vec12 &q, const Vec34 feetForce){
    Vec12 tau;
    
    // 遍历四条腿，分别计算每条腿的逆动力学
    for(int i(0); i < 4; ++i){
        // 提取第i条腿的关节角度和足端力
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

/**
* @brief 单腿正运动学：计算指定腿的足端位置
* @param state 机器人底层状态
* @param id 腿部编号（0:FR, 1:FL, 2:RR, 3:RL）
* @param frame 期望的坐标系类型
* @return 足端在指定坐标系中的位置向量
* 
* 该函数提供单腿的足端位置查询，支持不同坐标系：
* - BODY坐标系：足端相对于机器人身体中心的位置
* - HIP坐标系：足端相对于该腿髋关节中心的位置
*/
Vec3 QuadrupedRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec34 qLegs= state.getQ();      // 获取所有关节角度

    if(frame == FrameType::BODY){
        return _Legs[id]->calcPEe2B(qLegs.col(id));     // 身体坐标系
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));     // 髋关节坐标系
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

/**
* @brief 单腿速度正运动学：计算指定腿的足端速度
* @param state 机器人底层状态
* @param id 腿部编号
* @return 足端在髋关节坐标系中的速度向量
* 
* 该函数通过微分运动学计算足端速度，用于：
* 1. 速度反馈控制
* 2. 运动状态监测
* 3. 步态分析
*/
Vec3 QuadrupedRobot::getFootVelocity(LowlevelState &state, int id){
    Vec34 qLegs = state.getQ();     // 获取关节角度
    Vec34 qdLegs= state.getQd();    // 获取关节角速度
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

/**
* @brief 全身正运动学：计算所有足端位置
* @param state 机器人底层状态
* @param frame 期望的坐标系类型
* @return 4×3矩阵，每列表示一条腿的足端位置
* 
* 该函数计算四条腿的足端位置，支持多种坐标系：
* - BODY/HIP坐标系：直接调用单腿运动学
* - GLOBAL坐标系：考虑机器人姿态，转换到世界坐标系
* 
* 全局坐标系变换：
* P_global = R_body2global * P_body
* 其中R_body2global是机器人姿态旋转矩阵
*/
Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec34 feetPos;
    
    if(frame == FrameType::GLOBAL){
        // 全局坐标系：需要考虑机器人的姿态
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        // 通过旋转矩阵将身体坐标系转换到全局坐标系
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        // 身体或髋关节坐标系：直接计算
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

/**
* @brief 全身速度正运动学：计算所有足端速度
* @param state 机器人底层状态
* @param frame 期望的坐标系类型
* @return 4×3矩阵，每列表示一条腿的足端速度
* 
* 该函数计算四条腿的足端速度，全局坐标系下需要额外考虑：
* 1. 机器人本体旋转对足端速度的影响
* 2. 科里奥利效应：v_global = v_body + ω × r
* 
* 其中：
* - v_body：足端在身体坐标系中的速度
* - ω：机器人角速度（陀螺仪测量）
* - r：足端相对于身体中心的位置向量
*/
Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec34 feetVel;
    
    // 首先计算所有足端在身体坐标系中的速度
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        // 全局坐标系：需要加上由机器人旋转引起的速度分量
        Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
        
        // 添加角速度叉积项：v_global = v_body + ω × r
        // skew()函数将角速度向量转换为反对称矩阵
        feetVel += skew(state.getGyro()) * feetPos;
        
        // 转换到全局坐标系
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        // 身体或髋关节坐标系：直接返回
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

/**
* @brief 获取指定腿的雅可比矩阵
* @param state 机器人底层状态
* @param legID 腿部编号
* @return 3×3雅可比矩阵
* 
* 该函数返回指定腿的雅可比矩阵，用于：
* 1. 速度运动学分析
* 2. 力控制中的坐标变换
* 3. 奇异性分析
*/
Mat3 QuadrupedRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

// =============================================================================
// A1机器人具体实现
// =============================================================================

/**
* @brief A1机器人构造函数
* 
* 初始化A1机器人的所有物理参数和几何配置，包括：
* 1. 四条腿的几何参数和位置
* 2. 理想站立姿态的足端位置
* 3. 运动速度限制
* 4. 质量和惯量参数（区分仿真和真实机器人）
* 
* A1机器人参数特点：
* - 连杆长度：髋外展0.0838m，大腿0.2m，小腿0.2m
* - 质量：真实12.5kg，仿真13.4kg
* - 尺寸：身长约0.361m，身宽约0.094m
*/
A1Robot::A1Robot(){
    // 创建四条腿对象，每条腿包含其在身体坐标系中的髋关节位置
    // Vec3参数：(x, y, z) 分别表示前后、左右、上下方向的偏移量
    _Legs[0] = new A1Leg(0, Vec3( 0.1805, -0.047, 0));  // FR: 右前腿
    _Legs[1] = new A1Leg(1, Vec3( 0.1805,  0.047, 0));  // FL: 左前腿
    _Legs[2] = new A1Leg(2, Vec3(-0.1805, -0.047, 0));  // RR: 右后腿
    _Legs[3] = new A1Leg(3, Vec3(-0.1805,  0.047, 0));  // RL: 左后腿

    // 设置理想站立状态下的足端位置（身体坐标系）
    // 4×3矩阵，每列对应一条腿的足端位置 [x; y; z]
    _feetPosNormalStand <<  0.1805,  0.1805, -0.1805, -0.1805,   // x坐标：前腿在前，后腿在后
                        -0.1308,  0.1308, -0.1308,  0.1308,    // y坐标：右腿为负，左腿为正
                        -0.3180, -0.3180, -0.3180, -0.3180;    // z坐标：所有足端都在身体下方

    // 设置机器人运动速度限制（安全约束）
    _robVelLimitX << -0.4, 0.4;      // 前进/后退速度限制 (m/s)
    _robVelLimitY << -0.3, 0.3;      // 左右移动速度限制 (m/s)  
    _robVelLimitYaw << -0.5, 0.5;    // 旋转角速度限制 (rad/s)

#ifdef COMPILE_WITH_REAL_ROBOT
    // 真实机器人参数（基于实际测量和标定）
    _mass = 12.5;                                           // 质量 (kg)
    _pcb << 0.01, 0.0, 0.0;                                // 质心位置偏移 (m)
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();       // 惯量张量对角线元素 (kg·m²)
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    // 仿真环境参数（Gazebo物理引擎）
    _mass = 13.4;                                           // 仿真质量略大于真实值
    _pcb << 0.0, 0.0, 0.0;                                 // 质心位置设为身体中心
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();       // 使用相同的惯量参数
#endif  // COMPILE_WITH_SIMULATION
}

// =============================================================================
// Go1机器人具体实现  
// =============================================================================

/**
* @brief Go1机器人构造函数
* 
* 初始化Go1机器人的所有物理参数和几何配置。
* Go1是A1的升级版本，具有更好的性能和更精确的参数。
* 
* Go1机器人参数特点：
* - 连杆长度：髋外展0.08m，大腿0.213m，小腿0.213m（比A1略长）
* - 质量：真实10.5kg，仿真12.0kg（比A1更轻）
* - 尺寸：身长约0.3762m，身宽约0.0935m
* - 性能：更好的运动能力和更低的能耗
*/
Go1Robot::Go1Robot(){
    // 创建四条腿对象，Go1的髋关节位置与A1略有不同
    _Legs[0] = new Go1Leg(0, Vec3( 0.1881, -0.04675, 0));  // FR: 右前腿
    _Legs[1] = new Go1Leg(1, Vec3( 0.1881,  0.04675, 0));  // FL: 左前腿  
    _Legs[2] = new Go1Leg(2, Vec3(-0.1881, -0.04675, 0));  // RR: 右后腿
    _Legs[3] = new Go1Leg(3, Vec3(-0.1881,  0.04675, 0));  // RL: 左后腿

    // 设置Go1理想站立状态下的足端位置
    _feetPosNormalStand <<  0.1881,  0.1881, -0.1881, -0.1881,   // x坐标：比A1稍长
                        -0.1300,  0.1300, -0.1300,  0.1300,    // y坐标：左右间距与A1相近  
                        -0.3200, -0.3200, -0.3200, -0.3200;    // z坐标：站立高度与A1相近

    // Go1与A1使用相同的速度限制配置
    _robVelLimitX << -0.4, 0.4;      // 前后速度限制
    _robVelLimitY << -0.3, 0.3;      // 左右速度限制
    _robVelLimitYaw << -0.5, 0.5;    // 旋转速度限制

#ifdef COMPILE_WITH_REAL_ROBOT
    // Go1真实机器人参数
    _mass = 10.5;                                           // 比A1更轻的质量
    _pcb << 0.04, 0.0, 0.0;                                // 质心位置略向前偏移
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();      // Go1专用惯量参数
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    // Go1仿真参数
    _mass = 12.0;                                           // 仿真质量
    _pcb << 0.0, 0.0, 0.0;                                 // 质心位置设为中心
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();      // 使用真实惯量参数
#endif  // COMPILE_WITH_SIMULATION
}