/**********************************************************************
 * 文件作用: 步态生成器头文件
 * 
 * 该文件定义了GaitGenerator类，它是宇树四足机器人步态控制系统的核心组件之一。
 * 主要功能包括：
 * 1. 基于摆线(cycloid)轨迹生成算法，为四足机器人的摆动腿生成平滑的足端轨迹
 * 2. 根据机器人的目标速度和转向角度，实时计算每条腿的期望位置和速度
 * 3. 与WaveGenerator配合工作，实现对四条腿运动的协调控制
 * 4. 支持动态调整步态高度，适应不同的运动需求
 * 
 * 该类是四足机器人实现各种步态（如小跑、奔跑等）的基础，通过数学建模
 * 将高级的运动指令转化为具体的足端轨迹规划。
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"    // 波形生成器，管理步态的相位和时序
#include "Gait/FeetEndCal.h"       // 足端计算器，负责计算足端落足位置

#ifdef COMPILE_DEBUG
#include <common/PyPlot.h>         // Python绘图接口，用于调试时可视化轨迹
#endif  // COMPILE_DEBUG

/**
 * @brief 摆线步态生成器类
 * 
 * GaitGenerator类实现基于摆线(cycloid)轨迹的步态生成算法。
 * 摆线轨迹具有以下优点：
 * - 轨迹光滑，无突变点，减少机械冲击
 * - 在起始和结束点速度为零，实现平稳的接触和离开
 * - 数学表达简洁，计算效率高
 * 
 * 该类与其他组件的协作关系：
 * - WaveGenerator: 提供步态的相位信息和时序控制
 * - FeetEndCal: 计算摆动腿的目标落足位置
 * - Estimator: 获取机器人当前状态信息
 * - ControlFrame: 接收生成的足端轨迹用于控制
 */
class GaitGenerator{
public:
    /**
     * @brief 构造函数
     * @param ctrlComp 控制组件指针，包含所有必要的系统组件引用
     * 
     * 初始化步态生成器，建立与系统其他组件的连接关系
     */
    GaitGenerator(CtrlComponents *ctrlComp);
    
    /**
     * @brief 析构函数
     * 
     * 清理资源，释放动态分配的内存
     */
    ~GaitGenerator();
    
    /**
     * @brief 设置步态参数
     * @param vxyGoalGlobal 全局坐标系下的目标水平速度 [m/s] (Vec2: x方向和y方向)
     * @param dYawGoal 目标偏航角速度 [rad/s] (绕z轴旋转)
     * @param gaitHeight 步态高度 [m] (摆动腿抬起的最大高度)
     * 
     * 该函数设置机器人的运动目标，这些参数将影响：
     * - 足端轨迹的水平跨步距离
     * - 机器人的转向行为
     * - 摆动腿的抬腿高度
     */
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    
    /**
     * @brief 主运行函数
     * @param feetPos 输出参数 - 四条腿的足端位置 [m] (3×4矩阵: x,y,z × 四条腿)
     * @param feetVel 输出参数 - 四条腿的足端速度 [m/s] (3×4矩阵: vx,vy,vz × 四条腿)
     * 
     * 这是步态生成器的核心函数，每个控制周期调用一次。
     * 对于每条腿：
     * - 支撑腿(contact=1): 保持当前位置不变，速度为零
     * - 摆动腿(contact=0): 根据摆线轨迹计算新的位置和速度
     * 
     * 处理逻辑：
     * 1. 检查是否为首次运行，记录初始足端位置
     * 2. 遍历四条腿，根据接触状态分别处理
     * 3. 对摆动腿计算目标落足点和摆线轨迹
     * 4. 更新历史数据用于下一周期计算
     */
    void run(Vec34 &feetPos, Vec34 &feetVel);
    
    /**
     * @brief 获取指定腿的足端位置
     * @param i 腿的编号 (0:右前, 1:左前, 2:右后, 3:左后)
     * @return 足端位置 Vec3 [m] (x, y, z坐标)
     * 
     * 基于当前相位计算摆动腿的足端位置，使用摆线轨迹算法
     */
    Vec3 getFootPos(int i);
    
    /**
     * @brief 获取指定腿的足端速度
     * @param i 腿的编号 (0:右前, 1:左前, 2:右后, 3:左后)
     * @return 足端速度 Vec3 [m/s] (vx, vy, vz分量)
     * 
     * 计算摆动腿足端速度，确保轨迹的连续性和平滑性
     */
    Vec3 getFootVel(int i);
    
    /**
     * @brief 重启步态生成器
     * 
     * 重置内部状态，用于：
     * - 切换步态模式时的初始化
     * - 异常情况下的系统恢复
     * - 从其他控制状态切换到步态控制时
     */
    void restart();

private:
    /**
     * @brief 计算摆线轨迹的水平位置(X/Y方向)
     * @param startXY 起始位置坐标 [m]
     * @param endXY 结束位置坐标 [m] 
     * @param phase 当前相位 [0~1] (0:摆动开始, 1:摆动结束)
     * @return 当前相位对应的位置坐标 [m]
     * 
     * 摆线轨迹方程: pos = (end-start)*(φ-sin(φ))/(2π) + start
     * 其中 φ = 2π*phase
     * 
     * 特点：
     * - 起始点和结束点的速度都为零
     * - 轨迹平滑，无尖点
     * - 中间阶段速度变化平缓
     */
    float cycloidXYPosition(float startXY, float endXY, float phase);
    
    /**
     * @brief 计算摆线轨迹的水平速度(X/Y方向)
     * @param startXY 起始位置坐标 [m]
     * @param endXY 结束位置坐标 [m]
     * @param phase 当前相位 [0~1]
     * @return 当前相位对应的速度 [m/s]
     * 
     * 摆线轨迹速度方程: vel = (end-start)*(1-cos(φ))/T_swing
     * 其中 φ = 2π*phase, T_swing为摆动相时间
     * 
     * 确保在相位0和1处速度为零，实现平稳的着地和离地
     */
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    
    /**
     * @brief 计算摆线轨迹的垂直位置(Z方向)
     * @param startZ 起始高度 [m] (通常为地面高度)
     * @param height 最大抬腿高度 [m] (相对于起始高度)
     * @param phase 当前相位 [0~1]
     * @return 当前相位对应的高度 [m]
     * 
     * 垂直摆线轨迹方程: z = h*(1-cos(φ))/2 + start
     * 其中 φ = 2π*phase
     * 
     * 特点：
     * - 起始和结束时贴近地面
     * - 中点时达到最大高度
     * - 轨迹对称，上升和下降过程相似
     */
    float cycloidZPosition(float startZ, float height, float phase);
    
    /**
     * @brief 计算摆线轨迹的垂直速度(Z方向)
     * @param height 最大抬腿高度 [m]
     * @param phase 当前相位 [0~1]
     * @return 当前相位对应的垂直速度 [m/s]
     * 
     * 垂直摆线轨迹速度方程: vz = h*π*sin(φ)/T_swing
     * 其中 φ = 2π*phase, T_swing为摆动相时间
     * 
     * 确保着地和离地瞬间垂直速度为零，避免冲击
     */
    float cycloidZVelocity(float height, float phase);

    // ========== 系统组件引用 ==========
    WaveGenerator *_waveG;      // 波形生成器指针，管理步态节拍和相位
    Estimator *_est;            // 状态估计器指针，提供机器人当前状态
    FeetEndCal *_feetCal;       // 足端计算器指针，计算目标落足位置
    QuadrupedRobot *_robModel;  // 机器人模型指针，提供运动学参数
    LowlevelState *_state;      // 底层状态指针，获取传感器数据
    
    // ========== 运动目标参数 ==========
    float _gaitHeight;          // 当前设定的步态高度 [m]
    Vec2 _vxyGoal;             // 目标水平速度 [m/s] (x,y分量)
    float _dYawGoal;           // 目标偏航角速度 [rad/s]
    
    // ========== 相位和接触状态 ==========
    Vec4 *_phase;              // 当前各腿相位指针 [0~1] (四条腿)
    Vec4 _phasePast;           // 上一周期的相位状态，用于计算相位变化率
    VecInt4 *_contact;         // 各腿接触状态指针 (1:支撑, 0:摆动)
    
    // ========== 足端位置数据 ==========
    Vec34 _startP;             // 摆动开始时的足端位置 [m] (3×4矩阵)
    Vec34 _endP;               // 摆动结束时的目标足端位置 [m] (3×4矩阵)  
    Vec34 _idealP;             // 理想足端位置 [m] (预留，可用于轨迹规划)
    Vec34 _pastP;              // 上一周期的足端位置 [m] (用于速度计算)
    
    // ========== 控制标志 ==========
    bool _firstRun;            // 首次运行标志，用于初始化

#ifdef COMPILE_DEBUG
    PyPlot _testGaitPlot;      // 调试绘图对象，用于可视化步态轨迹
#endif  // COMPILE_DEBUG

};

#endif  // GAITGENERATOR_H