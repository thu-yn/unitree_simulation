/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

/**
* @class State_Passive
* @brief 被动状态类 - 机器人的初始状态（躺在地面）
* 
* 这是机器人启动时的默认状态，机器人在此状态下：
* - 所有关节松弛，没有力矩输出
* - 处于最低能耗状态
* - 等待用户指令进入其他状态
* - 机器人躺在地面，关节处于自然下垂位置
*/
class State_Passive : public FSMState{
public:
    /**
    * @brief 构造函数
    * @param ctrlComp 控制组件指针，包含所有控制相关的数据和接口
    */
    State_Passive(CtrlComponents *ctrlComp);
    
    /**
    * @brief 状态进入函数
    * 当从其他状态切换到PASSIVE状态时调用
    * 主要功能：设置所有电机为阻尼模式，关闭所有力矩输出
    */
    void enter();
    
    /**
    * @brief 状态运行函数  
    * 在PASSIVE状态下每个控制周期(500Hz)调用一次
    * 由于是被动状态，此函数为空实现
    */
    void run();
    
    /**
    * @brief 状态退出函数
    * 当从PASSIVE状态切换到其他状态时调用
    * 由于无需特殊清理，此函数为空实现
    */
    void exit();
    
    /**
    * @brief 状态切换检查函数
    * 检查是否满足状态切换条件
    * @return FSMStateName 下一个状态名称
    * 
    * 切换逻辑：
    * - 按下L2+A键(用户命令) → 切换到FIXEDSTAND状态
    * - 其他情况 → 保持PASSIVE状态
    */
    FSMStateName checkChange();
};

#endif  // PASSIVE_H