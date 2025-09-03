/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

/**
* @brief State_FixedStand构造函数
* @param ctrlComp 控制组件指针
* 
* 调用基类FSMState构造函数，设置：
* - 状态名称：FSMStateName::FIXEDSTAND
* - 状态字符串："fixed stand"（用于日志输出和调试）
*/
State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

/**
* @brief 进入固定站立状态
* 
* 这是状态机切换到FIXEDSTAND时的初始化函数，负责：
* 1. 配置关节控制模式和参数
* 2. 记录当前位置作为插值起点
* 3. 设置足端接触状态
*/
void State_FixedStand::enter(){
    // 为四条腿（每条腿3个关节）设置控制参数
    for(int i=0; i<4; i++){
        // 根据控制平台设置不同的控制增益
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            /**
            * 仿真环境下的增益设置
            * - 使用较高的增益保证仿真中的响应速度
            * - setSimStanceGain()设置适合Gazebo仿真的PD参数
            */
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            /**
            * 真实机器人环境下的增益设置
            * - 使用较低的增益确保真实硬件的安全性
            * - setRealStanceGain()设置适合真实机器人的PD参数
            */
            _lowCmd->setRealStanceGain(i);
        }
        
        /**
        * 清零速度和力矩命令
        * - setZeroDq(i)：设置第i条腿的3个关节目标速度为0
        * - setZeroTau(i)：设置第i条腿的3个关节目标力矩为0
        * 
        * 这样做的原因：
        * 1. 在位置控制模式下，主要依靠位置误差产生控制力矩
        * 2. 清零速度命令避免额外的速度扰动
        * 3. 清零力矩命令确保纯PD位置控制
        */
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    
    /**
    * 记录当前关节位置作为插值起点
    * 这是插值算法的关键步骤：
    * 1. 获取机器人当前的实际关节角度
    * 2. 将目标位置命令初始化为当前位置（避免突跳）
    * 3. 记录起始位置供插值计算使用
    */
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;    // 设置初始目标位置
        _startPos[i] = _lowState->motorState[i].q;              // 记录起始位置
    }
    
    /**
    * 设置所有腿为支撑状态
    * - 在站立状态下，四条腿都应该接触地面
    * - 这个设置影响状态估计和平衡控制算法
    * - 与之前PASSIVE状态的setAllSwing()形成对比
    */
    _ctrlComp->setAllStance();
}

/**
* @brief 固定站立状态运行函数
* 
* 每个控制周期（500Hz）执行一次，实现平滑的插值控制
* 
* 插值算法原理：
* 当前位置 = (1-t) × 起始位置 + t × 目标位置
* 其中 t 从 0 线性增长到 1
*/
void State_FixedStand::run(){
    /**
    * 更新插值进度
    * _percent += 1/_duration 实现线性递增
    * 
    * 计算逻辑：
    * - 每个周期增加 1/1000 = 0.001
    * - 1000个周期后 _percent 达到 1.0
    * - 总时间：1000 × 0.002s = 2秒
    */
    _percent += (float)1/_duration;
    
    /**
    * 限制插值进度不超过1.0
    * 防止插值超调，确保最终稳定在目标位置
    */
    _percent = _percent > 1 ? 1 : _percent;
    
    /**
    * 对所有12个关节执行线性插值
    * 
    * 插值公式详解：
    * q_cmd = (1-percent) × q_start + percent × q_target
    * 
    * 当 percent = 0：q_cmd = q_start（起始位置）
    * 当 percent = 1：q_cmd = q_target（目标位置）
    * 当 0 < percent < 1：平滑过渡
    */
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }
}

/**
* @brief 退出固定站立状态
* 
* 当状态机切换到其他状态时调用
* 主要功能：重置插值进度，为下次进入该状态做准备
*/
void State_FixedStand::exit(){
    /**
     * 重置插值进度
     * 确保下次进入FIXEDSTAND状态时从头开始插值
     */
    _percent = 0;
}

/**
* @brief 检查状态切换条件
* @return FSMStateName 下一个状态
* 
* FIXEDSTAND是一个重要的枢纽状态，支持多种状态切换：
* 1. 可以回退到安全状态
* 2. 可以进入运动状态
* 3. 可以进入各种测试状态
* 
* 这种设计提供了丰富的控制选择和调试能力
*/
FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        /**
        * L2_B：回到被动状态
        * 这是安全退出命令，让机器人重新躺下
        */
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        /**
        * L2_X：进入自由站立状态
        * 允许调整身体姿态（俯仰、翻滚、高度等）
        */
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        /**
        * START：进入小跑运动状态
        * 这是最重要的状态切换，开始运动控制
        */
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        /**
        * L1_X：进入平衡测试状态
        * 用于测试和调试平衡控制算法
        */
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        /**
        * L1_A：进入摆动测试状态
        * 用于测试腿部摆动轨迹和步态生成
        */
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        /**
        * L1_Y：进入步伐测试状态
        * 用于测试完整的步伐协调
        */
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        /**
        * L2_Y：进入移动基座状态（可选编译）
        * 启用ROS导航栈集成，用于自主导航
        */
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else{
        /**
        * 其他情况：保持固定站立状态
        * 没有有效用户命令时维持当前状态
        */
        return FSMStateName::FIXEDSTAND;
    }
}