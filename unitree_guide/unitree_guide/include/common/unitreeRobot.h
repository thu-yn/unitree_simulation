/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

/**
 * @file unitreeRobot.h
 * @brief Unitree四足机器人模型基类及具体机器人实现
 * 
 * 本文件定义了四足机器人的通用模型基类和具体机器人型号的实现。
 * 提供了完整的运动学、动力学计算接口，支持不同坐标系之间的转换。
 * 
 * 核心功能：
 * - 正向运动学：从关节角度计算足端位置和速度
 * - 逆向运动学：从足端位置计算关节角度
 * - 动力学计算：从足端力计算关节力矩
 * - 坐标变换：支持机体、髋关节、全局坐标系转换
 * - 机器人参数：提供质量、惯性、几何等物理参数
 * 
 * 支持的机器人型号：
 * - A1Robot：宇树A1四足机器人
 * - Go1Robot：宇树Go1四足机器人
 * 
 * 设计模式：
 * - 模板方法模式：基类定义算法框架，子类实现具体参数
 * - 策略模式：不同机器人型号采用不同的参数配置
 * - 工厂模式：通过编译宏选择具体的机器人型号
 */

/**
 * @brief 四足机器人通用基类
 * 
 * 这是所有Unitree四足机器人的基类，定义了机器人的通用接口和算法。
 * 采用组合模式，通过4个QuadrupedLeg对象来表示4条腿。
 * 
 * 坐标系定义：
 * - GLOBAL：全局坐标系（世界坐标系），Z轴向上
 * - BODY：机体坐标系，原点在机器人重心，X轴指向前方
 * - HIP：髋关节坐标系，原点在髋关节，X轴指向前方
 * 
 * 腿部编号约定：
 * - 0: 前左腿 (Front Left, FL)
 * - 1: 前右腿 (Front Right, FR)
 * - 2: 后左腿 (Hind Left, HL)
 * - 3: 后右腿 (Hind Right, HR)
 */
class QuadrupedRobot{
public:
    /**
     * @brief 默认构造函数
     * 基类构造函数，具体参数由子类初始化
     */
    QuadrupedRobot(){};
    
    /**
     * @brief 虚析构函数
     * 确保派生类对象能够正确析构
     */
    ~QuadrupedRobot(){}

    // ==================== 状态获取函数 ====================
    
    /**
     * @brief 获取机器人在全局坐标系下的位置
     * @param state 机器人底层状态信息
     * @return Vec3 机器人重心在全局坐标系下的3D位置 [x, y, z]
     * 
     * 从底层状态信息中提取机器人的全局位置信息。
     * 通常来源于IMU积分、视觉SLAM或其他定位系统。
     */
    Vec3 getX(LowlevelState &state);
    
    /**
     * @brief 获取机器人位置的向量形式表示
     * @param state 机器人底层状态信息
     * @return Vec34 位置信息的3×4矩阵表示
     * 
     * 将机器人位置信息组织成便于处理的矩阵形式，
     * 可能包含位置、速度等多个3维向量的组合。
     */
    Vec34 getVecXP(LowlevelState &state);

    // ==================== 逆运动学函数 ====================
    
    /**
     * @brief 逆运动学计算：从足端位置计算关节角度
     * @param feetPosition 四个足端的目标位置，3×4矩阵，每列代表一条腿的[x,y,z]坐标
     * @param frame 坐标系类型（BODY/HIP/GLOBAL）
     * @return Vec12 12个关节的目标角度 [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, HL_hip, HL_thigh, HL_calf, HR_hip, HR_thigh, HR_calf]
     * 
     * 根据期望的足端位置计算对应的关节角度。
     * 这是轨迹跟踪控制的关键函数，将笛卡尔空间的轨迹转换为关节空间的指令。
     * 
     * 计算流程：
     * 1. 坐标系转换（如果需要）
     * 2. 对每条腿执行3DOF逆运动学解析解
     * 3. 检查关节角度限制
     * 4. 返回12维关节角度向量
     * 
     * 应用场景：
     * - 足端轨迹跟踪控制
     * - 步态规划中的关节角度计算
     * - 平衡控制中的足端位置调节
     */
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    
    /**
     * @brief 逆运动学速度计算：从足端位置和速度计算关节角速度
     * @param feetPosition 四个足端的当前位置，3×4矩阵
     * @param feetVelocity 四个足端的期望速度，3×4矩阵
     * @param frame 坐标系类型（BODY/HIP/GLOBAL）
     * @return Vec12 12个关节的目标角速度
     * 
     * 基于雅可比矩阵计算关节角速度，用于速度级别的控制。
     * 
     * 数学原理：
     * dq = J^(-1) * dx
     * 其中J是3×3的腿部雅可比矩阵，dx是足端速度，dq是关节角速度
     * 
     * 应用场景：
     * - 速度控制模式
     * - 力控制中的阻抗调节
     * - 动态平衡控制
     */
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    
    /**
     * @brief 逆动力学计算：从足端力计算关节力矩
     * @param q 当前关节角度，12维向量
     * @param feetForce 四个足端的期望作用力，3×4矩阵，每列为一个足端的[fx,fy,fz]
     * @return Vec12 12个关节的输出力矩
     * 
     * 通过雅可比转置将足端力映射到关节力矩空间。
     * 
     * 数学原理：
     * τ = J^T * F
     * 其中τ是关节力矩，J是雅可比矩阵，F是足端力
     * 
     * 应用场景：
     * - 力控制模式
     * - 平衡控制中的重心调节
     * - 地形适应控制
     * - 跳跃和冲击缓冲
     */
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // ==================== 正运动学函数 ====================
    
    /**
     * @brief 单腿正运动学：计算指定腿的足端位置
     * @param state 机器人当前状态
     * @param id 腿部ID（0:FL, 1:FR, 2:HL, 3:HR）
     * @param frame 目标坐标系类型
     * @return Vec3 该腿足端在指定坐标系下的位置
     * 
     * 从当前关节角度计算足端的实际位置，用于状态反馈和位置确认。
     * 
     * 应用场景：
     * - 当前足端位置查询
     * - 碰撞检测
     * - 状态估计验证
     */
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    
    /**
     * @brief 单腿足端速度计算
     * @param state 机器人当前状态
     * @param id 腿部ID
     * @return Vec3 该腿足端在机体坐标系下的速度
     * 
     * 通过雅可比矩阵和关节角速度计算足端速度。
     * v = J * dq
     */
    Vec3 getFootVelocity(LowlevelState &state, int id);
    
    /**
     * @brief 全部足端位置计算
     * @param state 机器人当前状态
     * @param frame 目标坐标系类型
     * @return Vec34 四个足端在指定坐标系下的位置，3×4矩阵
     * 
     * 批量计算所有足端的位置，提高计算效率。
     * 结果矩阵每一列对应一条腿的[x,y,z]坐标。
     */
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    
    /**
     * @brief 全部足端速度计算
     * @param state 机器人当前状态  
     * @param frame 目标坐标系类型
     * @return Vec34 四个足端在指定坐标系下的速度，3×4矩阵
     * 
     * 批量计算所有足端的速度，考虑机体运动对足端速度的影响。
     * 当坐标系为GLOBAL时，会自动叠加机体运动产生的速度分量。
     */
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);

    // ==================== 雅可比矩阵函数 ====================
    
    /**
     * @brief 计算指定腿的雅可比矩阵
     * @param state 机器人当前状态
     * @param legID 腿部ID（0-3）
     * @return Mat3 3×3雅可比矩阵
     * 
     * 雅可比矩阵描述关节角度变化与足端位置变化的关系：
     * dx = J * dq （其中dx为足端位置微分，dq为关节角度微分）
     * 
     * 用途：
     * - 逆运动学求解
     * - 力矩转换计算  
     * - 奇异性分析
     * - 可操作性评估
     */
    Mat3 getJaco(LowlevelState &state, int legID);

    // ==================== 机器人参数获取函数 ====================
    
    /**
     * @brief 获取X方向（前后）速度限制
     * @return Vec2 [最小值, 最大值] 单位：m/s
     * 
     * 机器人在前后方向的安全运动速度范围。
     * 典型值：[-0.4, 0.4] m/s
     */
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    
    /**
     * @brief 获取Y方向（左右）速度限制  
     * @return Vec2 [最小值, 最大值] 单位：m/s
     * 
     * 机器人在左右方向的安全运动速度范围。
     * 典型值：[-0.3, 0.3] m/s
     */
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    
    /**
     * @brief 获取偏航角速度限制
     * @return Vec2 [最小值, 最大值] 单位：rad/s
     * 
     * 机器人绕Z轴旋转的安全角速度范围。
     * 典型值：[-0.5, 0.5] rad/s
     */
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    
    /**
     * @brief 获取理想站立时的足端位置
     * @return Vec34 标准站立姿态下四个足端的位置，3×4矩阵
     * 
     * 定义了机器人的标准站立姿态，用于：
     * - 初始化时的目标位置
     * - 复位动作的参考位置
     * - 步态规划的基准位置
     * - 平衡控制的参考配置
     * 
     * 坐标系：机体坐标系，单位：米
     * 矩阵结构：[FL FR HL HR; x x x x; y y y y; z z z z]
     */
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    
    /**
     * @brief 获取机器人质量
     * @return double 机器人总质量，单位：kg
     * 
     * 包含机器人本体、电池等所有质量。
     * A1: ~12.5kg (真机) / ~13.4kg (仿真)
     * Go1: ~10.5kg (真机) / ~12.0kg (仿真)
     */
    double getRobMass(){return _mass;}
    
    /**
     * @brief 获取重心偏移向量
     * @return Vec3 重心相对于机体坐标系原点的偏移，单位：m
     * 
     * 描述机器人实际重心相对于几何中心的偏移。
     * 用于动力学建模和平衡控制中的重力补偿。
     * 
     * 格式：[x_offset, y_offset, z_offset]
     * 通常y_offset和z_offset接近0，x_offset可能有小的前后偏移。
     */
    Vec3 getPcb(){return _pcb;}
    
    /**
     * @brief 获取机器人惯性张量矩阵
     * @return Mat3 惯性张量矩阵，单位：kg⋅m²
     * 
     * 3×3对角矩阵，描述机器人绕三个轴的转动惯量：
     * [Ixx  0   0 ]
     * [ 0  Iyy  0 ]  
     * [ 0   0  Izz]
     * 
     * 其中：
     * - Ixx：绕X轴（Roll轴）的转动惯量
     * - Iyy：绕Y轴（Pitch轴）的转动惯量  
     * - Izz：绕Z轴（Yaw轴）的转动惯量
     * 
     * 用于动力学建模、姿态控制和运动规划。
     */
    Mat3 getRobInertial(){return _Ib;}

protected:
    // ==================== 核心组件 ====================
    
    /**
     * @brief 四条腿的对象指针数组
     * 
     * 每个QuadrupedLeg对象封装了单腿的运动学、动力学计算。
     * 通过组合模式实现整机的运动学计算。
     * 
     * 数组索引对应：
     * - _Legs[0]: 前左腿 (Front Left)
     * - _Legs[1]: 前右腿 (Front Right) 
     * - _Legs[2]: 后左腿 (Hind Left)
     * - _Legs[3]: 后右腿 (Hind Right)
     */
    QuadrupedLeg* _Legs[4];

    // ==================== 运动限制参数 ====================
    
    Vec2 _robVelLimitX;          ///< X方向（前后）速度限制 [min, max] m/s
    Vec2 _robVelLimitY;          ///< Y方向（左右）速度限制 [min, max] m/s  
    Vec2 _robVelLimitYaw;        ///< 偏航角速度限制 [min, max] rad/s

    // ==================== 几何和物理参数 ====================
    
    /**
     * @brief 标准站立姿态的足端位置
     * 
     * 3×4矩阵，定义机器人标准站立时四个足端的位置：
     * [FL_x FR_x HL_x HR_x]  第1行：X坐标（前后方向）
     * [FL_y FR_y HL_y HR_y]  第2行：Y坐标（左右方向）
     * [FL_z FR_z HL_z HR_z]  第3行：Z坐标（上下方向，通常为负值）
     * 
     * 坐标系：机体坐标系，原点在机器人几何中心
     * 单位：米
     */
    Vec34 _feetPosNormalStand;
    
    double _mass;                ///< 机器人总质量，kg
    Vec3 _pcb;                   ///< 重心偏移向量，m
    Mat3 _Ib;                    ///< 惯性张量矩阵，kg⋅m²
};

/**
 * @brief Unitree A1机器人具体实现类
 * 
 * A1是Unitree公司的第一代商用四足机器人，具有以下特点：
 * - 重量：约12.5kg（真机）/ 13.4kg（仿真）
 * - 尺寸：相对较大，适合科研和教育应用
 * - 负载能力：较强
 * - 运动性能：稳定可靠
 * 
 * 主要参数：
 * - 机体长度：约0.361m（前后髋关节距离）
 * - 机体宽度：约0.094m（左右髋关节距离）  
 * - 标准站立高度：约0.318m
 * - 速度范围：X轴±0.4m/s，Y轴±0.3m/s，Yaw±0.5rad/s
 */
class A1Robot : public QuadrupedRobot{
public:
    /**
     * @brief A1机器人构造函数
     * 
     * 初始化A1机器人的所有参数，包括：
     * 1. 四条腿的几何参数和位置
     * 2. 标准站立姿态的足端位置
     * 3. 运动速度限制
     * 4. 质量和惯性参数（区分真机和仿真）
     * 
     * 髋关节位置设置（机体坐标系）：
     * - 前左腿：( 0.1805, -0.047, 0)
     * - 前右腿：( 0.1805,  0.047, 0)
     * - 后左腿：(-0.1805, -0.047, 0)
     * - 后右腿：(-0.1805,  0.047, 0)
     */
    A1Robot();
    
    /**
     * @brief A1机器人析构函数
     * 清理分配给四条腿的内存资源
     */
    ~A1Robot(){}
};

/**
 * @brief Unitree Go1机器人具体实现类
 * 
 * Go1是Unitree公司的改进型四足机器人，具有以下特点：
 * - 重量：约10.5kg（真机）/ 12.0kg（仿真）
 * - 尺寸：相对A1更紧凑
 * - 机动性：更高的运动灵活性
 * - 成本：更适合大规模应用
 * 
 * 主要参数：
 * - 机体长度：约0.376m（前后髋关节距离）
 * - 机体宽度：约0.0935m（左右髋关节距离）
 * - 标准站立高度：约0.32m  
 * - 速度范围：X轴±0.4m/s，Y轴±0.3m/s，Yaw±0.5rad/s
 */
class Go1Robot : public QuadrupedRobot{
public:
    /**
     * @brief Go1机器人构造函数
     * 
     * 初始化Go1机器人的所有参数，与A1相似但几何参数有所不同：
     * 
     * 髋关节位置设置（机体坐标系）：
     * - 前左腿：( 0.1881, -0.04675, 0)
     * - 前右腿：( 0.1881,  0.04675, 0)  
     * - 后左腿：(-0.1881, -0.04675, 0)
     * - 后右腿：(-0.1881,  0.04675, 0)
     * 
     * 与A1相比的主要差异：
     * - 机体稍长：0.1881m vs 0.1805m
     * - 机体稍窄：0.04675m vs 0.047m
     * - 重量更轻：10.5kg vs 12.5kg（真机）
     * - 惯性参数不同：适应更紧凑的设计
     */
    Go1Robot();
    
    /**
     * @brief Go1机器人析构函数
     * 清理分配给四条腿的内存资源
     */
    ~Go1Robot(){};
};

#endif  // UNITREEROBOT_H