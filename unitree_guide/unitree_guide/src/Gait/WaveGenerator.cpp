/**********************************************************************
 * 文件名: WaveGenerator.cpp
 * 作用: 步态波形生成器实现文件
 * 
 * 功能描述:
 * 这个文件实现了四足机器人的步态波形生成核心算法。WaveGenerator是整个步态控制系统的基础，
 * 负责根据时间生成四条腿的相位和接触状态，是实现小跑、爬行等步态的数学基础。
 * 
 * 核心职责:
 * 1. 生成周期性的步态相位信号（0-1范围的线性波形）
 * 2. 计算四条腿的接触状态（1=着地支撑，0=悬空摆动）
 * 3. 处理不同步态模式之间的平滑切换
 * 4. 支持可配置的步态参数（周期、支撑相比例、相位偏移）
 * 
 * 数学原理:
 * - 时间标准化: 将实际时间映射到[0,1)的标准化周期
 * - 相位偏移: 通过bias参数实现四条腿的相位差
 * - 双阶段划分: 每个步态周期分为支撑相和摆动相
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

/**
 * @brief WaveGenerator构造函数
 * 
 * 初始化步态波形生成器，设置步态的基本参数并进行参数有效性检查
 * 
 * @param period 步态周期，单位：秒
 *               - 小跑步态典型值: 0.45s (2.2Hz步频)
 *               - 爬行步态典型值: 1.1s (0.9Hz步频)
 *               - 决定了机器人的运动节拍
 * 
 * @param stancePhaseRatio 支撑相比例，范围(0,1)
 *               - 0.5: 支撑相和摆动相各占50%（典型小跑）
 *               - 0.75: 支撑相75%，摆动相25%（稳定爬行）
 *               - 值越大越稳定，但运动效率越低
 * 
 * @param bias 四条腿的相位偏移，Vec4类型，范围[0,1]
 *               - [0, 0.5, 0.5, 0]: 对角小跑（FR-RL同相，FL-RR同相，两组反相）
 *               - [0, 0.25, 0.5, 0.75]: 爬行步态（四条腿依次抬起）
 *               - [0, 0, 0, 0]: 四条腿同步（pronk跳跃步态）
 *               - 顺序: [前右, 前左, 后右, 后左]
 */
WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{
    // 参数有效性检查：支撑相比例必须在(0,1)范围内
    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);  // 参数无效，程序终止
    }

    // 参数有效性检查：相位偏移必须在[0,1]范围内
    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);  // 参数无效，程序终止
        }
    }

    // 初始化时间基准：记录构造时的系统时间（微秒）
    _startT = getSystemTime();
    
    // 初始化过去状态：设置初始接触状态为全悬空
    _contactPast.setZero();  // [0, 0, 0, 0] - 所有腿悬空
    
    // 初始化过去相位：设置为中点相位
    _phasePast << 0.5, 0.5, 0.5, 0.5;  // 所有腿相位为0.5（中点）
    
    // 初始化过去状态为全摆动模式
    _statusPast = WaveStatus::SWING_ALL;
}

/**
 * @brief WaveGenerator析构函数
 * 
 * 清理资源，当前实现为空，因为所有成员变量都是栈对象或基础类型
 */
WaveGenerator::~WaveGenerator()
{
    // 当前实现为空 - 无需特殊清理
}

/**
 * @brief 计算四条腿的相位和接触状态
 * 
 * 这是WaveGenerator的核心接口函数，被外部控制循环周期性调用（通常500Hz）
 * 该函数处理步态模式切换的平滑过渡，防止突变导致的机器人不稳定
 * 
 * 工作原理:
 * 1. 根据当前状态计算理想的相位和接触状态
 * 2. 检测步态模式是否发生变化
 * 3. 如果发生变化，启动平滑过渡机制
 * 4. 逐步将每条腿从旧状态过渡到新状态
 * 
 * @param phaseResult 输出参数：四条腿的当前相位，Vec4类型，范围[0,1]
 * @param contactResult 输出参数：四条腿的接触状态，VecInt4类型，0或1
 * @param status 当前期望的步态状态
 *               - WAVE_ALL: 正常步态生成
 *               - STANCE_ALL: 强制所有腿支撑
 *               - SWING_ALL: 强制所有腿摆动
 */
void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{
    // 步骤1: 根据当前状态计算理想的相位和接触状态
    calcWave(_phase, _contact, status);

    // 步骤2: 检测步态状态是否发生变化
    if (status != _statusPast)
    {
        // 状态发生变化，需要启动平滑过渡机制
        if (_switchStatus.sum() == 0)  // 如果当前没有在切换过程中
        {
            _switchStatus.setOnes();  // 标记所有腿都需要切换：[1, 1, 1, 1]
        }
        
        // 计算过去状态的相位和接触状态，用于平滑过渡
        calcWave(_phasePast, _contactPast, _statusPast);
        
        // 处理两种特殊情况：避免在极端状态切换时的数值问题
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            // 从全摆动切换到全支撑：强制设置过去接触状态为全支撑
            _contactPast.setOnes();  // [1, 1, 1, 1]
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            // 从全支撑切换到全摆动：强制设置过去接触状态为全摆动
            _contactPast.setZero();  // [0, 0, 0, 0]
        }
    }

    // 步骤3: 执行平滑过渡逻辑
    if (_switchStatus.sum() != 0)  // 如果还有腿在切换过程中
    {
        // 遍历四条腿，检查每条腿的切换状态
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))  // 如果当前腿的接触状态已经达到目标
            {
                _switchStatus(i) = 0;  // 标记该腿切换完成
            }
            else
            {
                // 该腿还需要继续使用过去的状态，保持平滑过渡
                _contact(i) = _contactPast(i);  // 使用过去的接触状态
                _phase(i) = _phasePast(i);      // 使用过去的相位
            }
        }
        
        // 检查是否所有腿都完成了切换
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;  // 更新过去状态，切换过程结束
        }
    }

    // 步骤4: 输出最终结果
    phaseResult = _phase;      // 返回四条腿的相位
    contactResult = _contact;  // 返回四条腿的接触状态
}

/**
 * @brief 获取支撑相时间
 * 
 * 计算每条腿在一个完整步态周期中的支撑时间
 * 
 * @return 支撑相时间，单位：秒
 *         例如：period=0.45s, stanceRatio=0.5 → 返回0.225s
 */
float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

/**
 * @brief 获取摆动相时间
 * 
 * 计算每条腿在一个完整步态周期中的摆动时间
 * 
 * @return 摆动相时间，单位：秒
 *         例如：period=0.45s, stanceRatio=0.5 → 返回0.225s
 */
float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

/**
 * @brief 获取步态周期
 * 
 * @return 完整步态周期时间，单位：秒
 */
float WaveGenerator::getT()
{
    return _period;
}

/**
 * @brief 步态波形计算核心函数
 * 
 * 根据不同的步态状态计算四条腿的相位和接触状态。
 * 这是整个步态生成器的数学核心，实现了时间到相位的映射。
 * 
 * 数学原理详解:
 * 1. 时间标准化：将连续时间映射到[0,1)的周期性区间
 * 2. 相位偏移：通过bias实现不同腿之间的时序差异
 * 3. 阶段判断：根据标准化时间判断支撑相还是摆动相
 * 4. 相位映射：将阶段内时间映射到[0,1]的相位值
 * 
 * @param phase 输出参数：四条腿的相位，范围[0,1]
 * @param contact 输出参数：四条腿的接触状态，0或1
 * @param status 步态状态枚举
 */
void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)  // 正常步态生成模式
    {
        // 步骤1: 计算从开始到现在的总时间（秒）
        _passT = (double)(getSystemTime() - _startT) * 1e-6;  // 微秒转换为秒
        
        // 步骤2: 为四条腿分别计算相位和接触状态
        for (int i(0); i < 4; ++i)
        {
            // 步骤2.1: 计算标准化时间，包含相位偏移
            // 公式解析：
            // - _passT: 当前经过的时间
            // - _period * _bias(i): 该腿的相位偏移量
            // - + _period: 确保结果为正数
            // - fmod(..., _period): 取模运算，得到周期内的时间
            // - / _period: 标准化到[0,1)区间
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            
            // 步骤2.2: 根据标准化时间判断当前阶段
            if (_normalT(i) < _stRatio)  // 支撑相阶段
            {
                contact(i) = 1;  // 该腿接触地面
                // 支撑相内的相位：将[0, _stRatio)映射到[0, 1)
                phase(i) = _normalT(i) / _stRatio;
            }
            else  // 摆动相阶段
            {
                contact(i) = 0;  // 该腿悬空
                // 摆动相内的相位：将[_stRatio, 1)映射到[0, 1)
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)  // 强制全摆动模式
    {
        // 所有腿都设置为摆动状态
        contact.setZero();  // [0, 0, 0, 0] - 全部悬空
        phase << 0.5, 0.5, 0.5, 0.5;  // 相位设置为中点值
    }
    else if (status == WaveStatus::STANCE_ALL)  // 强制全支撑模式
    {
        // 所有腿都设置为支撑状态
        contact.setOnes();  // [1, 1, 1, 1] - 全部着地
        phase << 0.5, 0.5, 0.5, 0.5;  // 相位设置为中点值
    }
}

/*
 * ==================== 算法核心思想总结 ====================
 * 
 * WaveGenerator实现的是一个优雅的时间-相位映射算法：
 * 
 * 1. 时间标准化：
 *    连续时间 → [0,1)周期性时间 → 考虑相位偏移
 * 
 * 2. 阶段划分：
 *    [0, stanceRatio) → 支撑相，腿接触地面
 *    [stanceRatio, 1) → 摆动相，腿悬空运动
 * 
 * 3. 相位计算：
 *    每个阶段内部，时间被进一步标准化为[0,1)的相位值
 *    这个相位值被上层的轨迹生成器用于计算具体的足端位置
 * 
 * 4. 平滑切换：
 *    通过状态缓存和逐步过渡，避免步态切换时的突变
 * 
 * 5. 典型应用：
 *    - 小跑步态：period=0.45s, stanceRatio=0.5, bias=[0,0.5,0.5,0]
 *    - 爬行步态：period=1.1s, stanceRatio=0.75, bias=[0,0.25,0.5,0.75]
 * 
 * 这种设计使得机器人能够实现各种复杂的步态模式，同时保持算法的
 * 简洁性和可配置性。
 * 
 * ==================== 关键参数影响 ====================
 * 
 * period (周期):
 *   - 越小→步频越高→运动越快，但稳定性下降
 *   - 越大→步频越低→运动越慢，但更稳定
 * 
 * stanceRatio (支撑相比例):
 *   - 越大→接触时间长→更稳定，但效率低
 *   - 越小→接触时间短→更灵活，但稳定性差
 * 
 * bias (相位偏移):
 *   - 决定了步态模式：小跑、爬行、跳跃等
 *   - 四个值分别对应：前右、前左、后右、后左腿
 */