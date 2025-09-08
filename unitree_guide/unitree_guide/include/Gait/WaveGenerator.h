/**********************************************************************
 * 文件作用: 波形生成器头文件
 * 
 * 本文件定义了WaveGenerator类，该类是四足机器人步态控制系统的核心组件之一。
 * 主要功能包括：
 * 1. 生成四条腿的步态相位信息，相位值范围为[0, 1]
 * 2. 控制足端接触状态，确定每条腿是处于支撑相还是摆动相
 * 3. 支持多种步态模式：小跑步态(Trot)、爬行步态(Crawl)、跳跃步态(Pronk)等
 * 4. 提供步态参数的实时调节能力，包括步态周期、支撑相比例、相位偏移等
 * 
 * 该波形生成器采用线性波形算法，为GaitGenerator提供时序控制信号，
 * 最终实现四足机器人的协调运动和稳定步态。
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"      // 数学类型定义(Vec4, VecInt4等向量类型)
#include "common/timeMarker.h"     // 时间标记工具，用于高精度时间计算
#include "common/enumClass.h"      // 枚举类定义，包含WaveStatus等步态状态枚举
#include <unistd.h>                // UNIX标准库，提供系统调用接口

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"         // Python绘图接口，用于调试时可视化步态波形
#endif  // COMPILE_DEBUG

/**
 * 波形生成器类
 * 
 * 功能描述：
 * 生成线性步态波形，输出相位值范围为[0, 1]的标准化时间信号。
 * 该类为四足机器人的步态控制提供基础的时序控制信号。
 * 
 * 核心概念：
 * - 相位(Phase): 描述每条腿在步态周期中的位置，0表示周期开始，1表示周期结束
 * - 支撑相(Stance Phase): 足端与地面接触的阶段，机器人通过该腿支撑身体重量
 * - 摆动相(Swing Phase): 足端离开地面的阶段，该腿向前摆动准备下一次着地
 * - 相位偏移(Phase Bias): 四条腿之间的时间差，用于实现不同步态模式
 */
class WaveGenerator{
public:
    /**
     * 构造函数
     * 
     * @param period 步态周期时间(秒)，决定一个完整步态循环的时长
     *               典型值: Trot步态0.4-0.5s，Crawl步态0.8-1.2s
     * 
     * @param stancePhaseRatio 支撑相时间比例，范围(0,1)
     *                        表示在一个完整步态周期中，足端接触地面的时间占比
     *                        典型值: Trot步态0.5，Crawl步态0.75
     * 
     * @param bias 四条腿的相位偏移向量Vec4(FR, FL, RR, RL)，范围[0,1]
     *             定义了四条腿启动时的相位差，用于创建不同的步态模式
     *             示例: Trot步态Vec4(0, 0.5, 0.5, 0) - 对角腿同步
     *                  Crawl步态Vec4(0, 0.25, 0.5, 0.75) - 依次抬腿
     */
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
    
    /**
     * 析构函数
     * 释放波形生成器占用的资源
     */
    ~WaveGenerator();
    
    /**
     * 计算接触相位信息 - 主要对外接口函数
     * 
     * 该函数是波形生成器的核心方法，根据当前时间和步态状态，
     * 计算四条腿的相位信息和接触状态。
     * 
     * @param phaseResult 输出参数：四条腿的相位信息Vec4，范围[0,1]
     *                   相位0表示步态周期开始，相位1表示步态周期结束
     * 
     * @param contactResult 输出参数：四条腿的接触状态VecInt4
     *                     1表示该腿处于支撑相(与地面接触)
     *                     0表示该腿处于摆动相(离开地面)
     * 
     * @param status 当前步态状态，枚举类型WaveStatus
     *              可能的状态包括：
     *              - WAVE: 正常步态模式
     *              - STANCE_ALL: 所有腿都支撑
     *              - SWING_ALL: 所有腿都摆动
     */
    void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status);
    
    /**
     * 获取支撑相时长
     * @return 支撑相持续时间(秒) = period * stancePhaseRatio
     */
    float getTstance();
    
    /**
     * 获取摆动相时长  
     * @return 摆动相持续时间(秒) = period * (1 - stancePhaseRatio)
     */
    float getTswing();
    
    /**
     * 获取当前经过的时间
     * @return 从波形生成器创建开始到现在的经过时间(秒)
     */
    float getT();

private:
    /**
     * 内部波形计算函数
     * 
     * 根据当前时间和步态参数计算四条腿的相位和接触状态。
     * 该函数实现了线性波形算法的核心逻辑。
     * 
     * @param phase 输出：四条腿的相位信息
     * @param contact 输出：四条腿的接触状态  
     * @param status 当前步态状态
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    // ================= 步态参数 =================
    double _period;                  // 步态周期时间(秒)，一个完整步态循环的时长
    double _stRatio;                 // 支撑相时间比例，范围(0,1)
    Vec4 _bias;                      // 四条腿相位偏移向量[FR, FL, RR, RL]

    // ================= 运行时状态变量 =================
    Vec4 _normalT;                   // 标准化时间向量，范围[0, 1)
                                    // 表示每条腿在当前步态周期中的时间进度
    
    Vec4 _phase, _phasePast;         // 当前相位和上一周期相位
                                    // 用于相位连续性检查和状态转换
    
    VecInt4 _contact, _contactPast;  // 当前接触状态和上一周期接触状态
                                    // 1=支撑相，0=摆动相
    
    VecInt4 _switchStatus;          // 状态切换标志位，范围{0,1}
                                    // 1表示该腿正在进行状态转换，0表示不转换
                                    // 用于处理步态模式切换时的平滑过渡
    
    WaveStatus _statusPast;         // 上一次的步态状态
                                   // 用于检测步态模式变化，实现状态转换逻辑

    // ================= 时间管理变量 =================
    double _passT;                   // 已经过时间(秒)
                                    // 记录从开始到当前的累积时间
    
    long long _startT;              // 起始时间戳(微秒)  
                                   // 系统启动时的时间基准，用于高精度时间计算

#ifdef COMPILE_DEBUG
    PyPlot _testPlot;               // 调试绘图工具
                                   // 仅在调试模式下编译，用于可视化步态波形
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H