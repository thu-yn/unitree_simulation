/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

/**
* @class State_FixedStand
* @brief 固定站立状态类 - 机器人从躺下到站立的过渡状态
* 
* 主要功能：
* 1. 让机器人从被动状态平滑地站立起来
* 2. 使用位置控制模式，精确控制关节角度
* 3. 通过插值实现平滑的运动轨迹
* 4. 为进入运动状态做准备
* 
* 关键特点：
* - 预设的固定站立姿态（不可调整）
* - 线性插值实现平滑过渡
* - 支持多种状态切换
* - 区分仿真和真实机器人的控制参数
*/
class State_FixedStand : public FSMState{
public:
    /**
    * @brief 构造函数
    * @param ctrlComp 控制组件指针，包含所有控制相关的数据和接口
    */
    State_FixedStand(CtrlComponents *ctrlComp);
    
    /**
    * @brief 析构函数
    * 空实现，无需特殊清理
    */
    ~State_FixedStand(){}
    
    /**
    * @brief 状态进入函数
    * 当从其他状态切换到FIXEDSTAND状态时调用
    * 
    * 主要功能：
    * 1. 设置所有关节为位置控制模式，配置PD参数
    * 2. 记录当前关节位置作为插值起点
    * 3. 设置所有腿为支撑状态（接触地面）
    * 4. 根据平台类型设置不同的控制增益
    */
    void enter();
    
    /**
    * @brief 状态运行函数
    * 在FIXEDSTAND状态下每个控制周期(500Hz)调用一次
    * 
    * 主要功能：
    * 1. 计算插值进度百分比
    * 2. 对12个关节位置进行线性插值
    * 3. 输出平滑的关节位置命令
    */
    void run();
    
    /**
    * @brief 状态退出函数
    * 当从FIXEDSTAND状态切换到其他状态时调用
    * 
    * 主要功能：
    * 重置插值进度，为下次进入该状态做准备
    */
    void exit();
    
    /**
    * @brief 状态切换检查函数
    * 检查是否满足状态切换条件
    * @return FSMStateName 下一个状态名称
    * 
    * 支持的状态切换：
    * - L2_B → PASSIVE（回到被动状态）
    * - L2_X → FREESTAND（进入自由站立）
    * - START → TROTTING（进入小跑运动）
    * - L1_X → BALANCETEST（平衡测试）
    * - L1_A → SWINGTEST（摆动测试）
    * - L1_Y → STEPTEST（步伐测试）
    * - L2_Y → MOVE_BASE（导航模式，可选）
    */
    FSMStateName checkChange();

private:
    /**
    * @brief 目标关节位置数组（12个关节）
    * 
    * 数组排列顺序：[FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
    *               RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
    * 
    * 标准站立姿态参数：
    * - hip关节：0.0 rad（髋关节中立位置）
    * - thigh关节：0.67 rad（大腿向下约38度）
    * - calf关节：-1.3 rad（小腿向上约74度）
    * 
    * 这组角度让机器人形成标准的四足站立姿态
    */
    float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    
    /**
    * @brief 起始关节位置数组
    * 在enter()函数中记录当前关节位置，作为插值的起点
    */
    float _startPos[12];
    
    /**
    * @brief 插值持续时间（控制周期数）
    * 1000步 × 0.002秒 = 2秒完成站立动作
    * 较长的时间确保动作平滑，减少冲击
    */
    float _duration = 1000;   // steps (控制周期数)
    
    /**
    * @brief 插值进度百分比
    * 0.0：起始位置
    * 1.0：目标位置
    * 每个控制周期递增 1/_duration
    */
    float _percent = 0;       // 插值进度百分比
};

#endif  // FIXEDSTAND_H