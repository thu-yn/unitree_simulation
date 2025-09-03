/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"

/**
* @brief State_Passive构造函数
* @param ctrlComp 控制组件指针
* 
* 调用基类FSMState构造函数，设置：
* - 状态名称：FSMStateName::PASSIVE
* - 状态字符串："passive"（用于日志输出）
*/
State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

/**
* @brief 进入被动状态
* 
* 功能说明：
* 1. 根据控制平台（仿真/真实机器人）设置不同的阻尼参数
* 2. 将所有12个电机设置为阻尼模式（mode=10）
* 3. 清零所有控制量：位置、速度、刚度、力矩
* 4. 设置所有腿为摆动状态（脱离地面接触）
*/
void State_Passive::enter(){
    // 仿真环境下的电机设置
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;      // 电机模式10：阻尼模式
            _lowCmd->motorCmd[i].q = 0;          // 目标位置：0（不使用位置控制）
            _lowCmd->motorCmd[i].dq = 0;         // 目标速度：0
            _lowCmd->motorCmd[i].Kp = 0;         // 位置增益：0（关闭位置控制）
            _lowCmd->motorCmd[i].Kd = 8;         // 阻尼增益：8（仿真环境较高阻尼）
            _lowCmd->motorCmd[i].tau = 0;        // 力矩：0（无额外力矩输出）
        }
    }
    // 真实机器人环境下的电机设置
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;      // 电机模式10：阻尼模式
            _lowCmd->motorCmd[i].q = 0;          // 目标位置：0
            _lowCmd->motorCmd[i].dq = 0;         // 目标速度：0
            _lowCmd->motorCmd[i].Kp = 0;         // 位置增益：0
            _lowCmd->motorCmd[i].Kd = 3;         // 阻尼增益：3（真实机器人较低阻尼）
            _lowCmd->motorCmd[i].tau = 0;        // 力矩：0
        }
    }

    /**
    * 设置所有腿为摆动状态
    * - 摆动状态意味着腿部脱离地面接触
    * - 在被动状态下，机器人躺在地面，所有腿都不承重
    * - 这样设置有助于状态估计和控制逻辑的一致性
    */
    _ctrlComp->setAllSwing();
}

/**
* @brief 被动状态运行函数
* 
* 被动状态下无需执行任何控制逻辑：
* - 电机已在enter()中设置为阻尼模式
* - 机器人将在重力作用下自然垂下
* - 只需等待用户指令进行状态切换
*/
void State_Passive::run(){
    // 空实现 - 被动状态无需主动控制
}

/**
* @brief 退出被动状态
* 
* 无需特殊的清理操作：
* - 电机设置将由下一个状态的enter()函数重新配置
* - 状态数据将被下一个状态接管
*/
void State_Passive::exit(){
    // 空实现 - 无需特殊清理
}

/**
* @brief 检查状态切换条件
* @return FSMStateName 下一个状态
* 
* 状态切换逻辑：
* 1. 检测到L2_A用户命令（通常对应手柄的L2+A按键组合）
*    → 返回FIXEDSTAND状态，开始机器人站立流程
* 2. 其他情况
*    → 保持PASSIVE状态不变
* 
* 这是机器人控制的起点：
* PASSIVE → FIXEDSTAND → TROTTING → 运动控制
*/
FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;     // 切换到固定站立状态
    }
    else{
        return FSMStateName::PASSIVE;        // 保持被动状态
    }
}