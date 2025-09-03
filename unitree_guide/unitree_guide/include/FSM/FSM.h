/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// ==================== 状态机相关头文件 ====================

// 状态基类和具体状态实现
#include "FSM/FSMState.h"            // 抽象状态基类
#include "FSM/State_FixedStand.h"    // 固定站立状态
#include "FSM/State_Passive.h"       // 被动状态（初始状态）
#include "FSM/State_FreeStand.h"     // 自由站立状态
#include "FSM/State_Trotting.h"      // 小跑运动状态
#include "FSM/State_BalanceTest.h"   // 平衡测试状态
#include "FSM/State_SwingTest.h"     // 摆动测试状态
#include "FSM/State_StepTest.h"      // 步态测试状态

// 系统依赖
#include "common/enumClass.h"        // 枚举类定义
#include "control/CtrlComponents.h"  // 控制组件集合

// 可选扩展：ROS导航集成
#ifdef COMPILE_WITH_MOVE_BASE
#include "FSM/State_move_base.h"  // ROS move_base导航状态
#endif  // COMPILE_WITH_MOVE_BASE

/**
* @brief 状态机状态列表结构体
* 
* 这个结构体包含了所有可能的机器人状态对象，是状态机的"状态池"。
* 每个状态都是一个独立的对象，负责处理特定的机器人行为。
* 
* 设计特点：
* - 所有状态对象集中管理，便于访问和维护
* - 使用指针存储，支持多态（所有状态都继承自FSMState）
* - 提供统一的内存清理接口
*/
struct FSMStateList{
    // ==================== 核心功能状态 ====================

    /**
    * @brief 无效状态指针
    * 通常设为nullptr，用作错误状态或占位符
    */
    FSMState *invalid;

    /**
    * @brief 被动状态（PASSIVE）
    * 机器人的初始状态，通常表示机器人躺在地面上
    * - 所有关节松弛，无力矩输出
    * - 系统启动后的默认状态
    * - 安全状态，紧急情况下可回到此状态
    */
    State_Passive *passive;

    /**
    * @brief 固定站立状态（FIXEDSTAND）
    * 机器人静态站立状态
    * - 关节位置控制到预设的站立姿态
    * - 为其他动态状态做准备
    * - 所有腿都支撑，重心稳定
    */
    State_FixedStand *fixedStand;

    /**
    * @brief 自由站立状态（FREESTAND）
    * 动态平衡站立状态
    * - 可以调整身体姿态（俯仰、横滚、偏航）
    * - 可以调整身体高度
    * - 具备一定的平衡调节能力
    */
    State_FreeStand *freeStand;

    /**
    * @brief 小跑运动状态（TROTTING）
    * 机器人的主要运动状态
    * - 实现各种步态的动态行走
    * - 支持前进、后退、转向、横移
    * - 包含复杂的步态生成和平衡控制
    */
    State_Trotting *trotting;

    // ==================== 测试和调试状态 ====================

    /**
    * @brief 平衡测试状态（BALANCETEST）
    * 用于测试机器人的平衡控制性能
    * - 测试在外力干扰下的平衡恢复能力
    * - 验证平衡控制算法的效果
    * - 调试和优化控制参数
    */
    State_BalanceTest *balanceTest;

    /**
    * @brief 摆动测试状态（SWINGTEST）
    * 用于测试单腿或多腿的摆动运动
    * - 验证足端轨迹生成算法
    * - 测试关节运动学正逆解
    * - 调试步态生成参数
    */
    State_SwingTest *swingTest;

    /**
    * @brief 步态测试状态（STEPTEST）
    * 用于测试步态相关的功能
    * - 验证步态切换的平滑性
    * - 测试不同步态模式
    * - 调试步态参数
    */
    State_StepTest *stepTest;

    // ==================== 可选扩展状态 ====================

#ifdef COMPILE_WITH_MOVE_BASE
    /**
    * @brief ROS导航状态（MOVE_BASE）
    * 与ROS导航栈集成的高级导航状态
    * - 接收ROS的cmd_vel消息控制机器人运动
    * - 与move_base节点配合实现自主导航
    * - 支持路径规划和避障功能
    */
    State_move_base *moveBase;
#endif  // COMPILE_WITH_MOVE_BASE

    /**
    * @brief 清理所有状态对象的内存
    * 在状态机析构时调用，确保没有内存泄露
    */
    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete balanceTest;
        delete swingTest;
        delete stepTest;
#ifdef COMPILE_WITH_MOVE_BASE
        delete moveBase;
#endif  // COMPILE_WITH_MOVE_BASE
    }
};

/**
* @brief 有限状态机（FSM）主控制器类
* 
* FSM是整个机器人行为控制的核心，负责：
* 1. 管理机器人的所有行为状态
* 2. 根据条件执行状态转换
* 3. 协调各个控制组件的工作
* 4. 确保系统安全性
* 
* 设计理念：
* - 状态模式（State Pattern）：每个状态封装特定行为
* - 集中控制：统一管理状态转换逻辑
* - 安全第一：内置安全检查机制
* - 实时性：在500Hz控制循环中高效运行
*/
class FSM{
public:
    /**
    * @brief 构造函数
    * @param ctrlComp 控制组件指针
    * 
    * 初始化状态机：
    * 1. 创建所有状态对象
    * 2. 设置初始状态为PASSIVE
    * 3. 配置状态机模式为NORMAL
    */
    FSM(CtrlComponents *ctrlComp);

    /**
    * @brief 析构函数
    * 清理所有状态对象，释放内存
    */
    ~FSM();

    /**
    * @brief 初始化状态机
    * 设置初始状态并进入PASSIVE状态
    */
    void initialize();

    /**
    * @brief 状态机主运行函数
    * 
    * 这是状态机的核心函数，每个500Hz控制周期调用一次。
    * 执行完整的控制流程：
    * 
    * 1. 数据通信：与机器人收发数据
    * 2. 步态更新：运行步态生成器
    * 3. 状态估计：更新机器人状态估计
    * 4. 安全检查：确保系统安全
    * 5. 状态执行：运行当前状态的控制逻辑
    * 6. 状态检查：检查是否需要状态转换
    * 7. 状态转换：执行状态切换（如果需要）
    * 8. 时间控制：等待下一个控制周期
    */
    void run();

private:
    // ==================== 私有方法 ====================

    /**
    * @brief 根据状态名获取状态对象
    * @param stateName 目标状态名
    * @return 对应的状态对象指针
    * 
    * 这是状态转换的核心方法，将状态枚举映射到具体状态对象
    */
    FSMState* getNextState(FSMStateName stateName);

    /**
    * @brief 安全检查函数
    * @return true表示系统安全，false表示存在安全风险
    * 
    * 检查系统是否处于安全状态：
    * - 通信是否正常
    * - 传感器数据是否有效
    * - 机器人姿态是否稳定
    * - 关节是否在安全范围内
    */
    bool checkSafty();

    // ==================== 私有成员变量 ====================

    /**
    * @brief 控制组件指针
    * 提供对所有控制组件的访问，包括：
    * - 命令和状态消息
    * - IO接口
    * - 步态生成器
    * - 状态估计器等
    */
    CtrlComponents *_ctrlComp;

    /**
    * @brief 当前活跃状态指针
    * 指向正在执行的状态对象
    */
    FSMState *_currentState;

    /**
    * @brief 下一个状态指针
    * 在状态转换过程中，临时存储目标状态
    */
    FSMState *_nextState;

    /**
    * @brief 下一个状态名
    * 存储状态转换的目标状态枚举值
    */
    FSMStateName _nextStateName;

    /**
    * @brief 状态列表
    * 包含所有状态对象的容器
    */
    FSMStateList _stateList;

    /**
    * @brief 状态机模式
    * - NORMAL: 正常运行模式，执行当前状态
    * - CHANGE: 状态转换模式，执行状态切换
    */
    FSMMode _mode;

    /**
    * @brief 控制周期开始时间
    * 用于精确控制500Hz的时序
    */
    long long _startTime;

    /**
    * @brief 计数器
    * 用于调试和统计目的
    */
    int count;
};

#endif  // FSM_H

/*
* ==================== 状态机设计分析 ====================
* 
* ## 🎯 核心设计理念
* 
* ### 1. 状态模式（State Pattern）
* - 每个状态都是独立的类，封装特定的行为逻辑
* - 状态转换通过多态实现，代码结构清晰
* - 便于添加新状态，扩展性强
* 
* ### 2. 集中式控制
* - FSM类统一管理所有状态和转换逻辑
* - 所有状态共享同一套控制组件
* - 便于全局协调和安全控制
* 
* ### 3. 双重状态机制
* - FSMMode控制状态机本身的状态（NORMAL/CHANGE）
* - FSMStateName控制机器人的行为状态
* - 确保状态转换的原子性和安全性
* 
* ## 🔄 状态转换流程
* 
* ```
* NORMAL模式：
* 1. currentState->run()           执行当前状态
* 2. checkChange()                 检查转换条件
* 3. 如果需要转换 → 设置CHANGE模式
* 
* CHANGE模式：
* 1. currentState->exit()          退出当前状态
* 2. currentState = nextState      切换状态指针
* 3. currentState->enter()         进入新状态
* 4. 设置NORMAL模式               恢复正常运行
* ```
* 
* ## 🛡️ 安全机制
* 
* ### 1. 安全检查
* - 每个控制周期都进行安全检查
* - 发现问题立即切换到被动模式
* - 多层次的安全保护
* 
* ### 2. 状态封装
* - 每个状态内部处理自己的安全逻辑
* - 状态转换条件明确，避免意外切换
* - 异常情况下可快速回到安全状态
* 
* ## 📊 典型状态转换图
* 
* ```
* PASSIVE (初始)
*    ↓ 按键'2'
* FIXEDSTAND (站立)
*    ↓ 按键'4'
* TROTTING (行走)
*    ↓ 'w''a''s''d'
* 各种运动控制
*    ↓ 紧急情况
* PASSIVE (安全状态)
* ```
* 
* ## 🎮 用户控制映射
* 
* - L2_B → PASSIVE     (被动状态)
* - L2_A → FIXEDSTAND  (固定站立)
* - L2_X → FREESTAND   (自由站立)
* - START → TROTTING   (开始行走)
* - L1_X → BALANCETEST (平衡测试)
* - L1_A → SWINGTEST   (摆动测试)
* - L1_Y → STEPTEST    (步态测试)
* - L2_Y → MOVE_BASE   (导航模式，可选)
* 
* ## 💡 关键技术特点
* 
* 1. **高频实时控制**：500Hz控制循环，确保实时性
* 2. **模块化设计**：状态独立，便于开发和调试
* 3. **安全优先**：多重安全检查，确保系统稳定
* 4. **可扩展性**：易于添加新状态和功能
* 5. **用户友好**：简单的按键控制界面
*/