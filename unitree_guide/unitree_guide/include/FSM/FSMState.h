/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

// ==================== 标准库头文件 ====================
#include <string>      // 字符串处理
#include <iostream>    // 输入输出流
#include <unistd.h>    // Unix标准函数定义

// ==================== 系统核心组件 ====================
#include "control/CtrlComponents.h"  // 控制组件集合
#include "message/LowlevelCmd.h"     // 底层控制命令
#include "message/LowlevelState.h"   // 底层状态反馈

// ==================== 通用工具和类型 ====================
#include "common/enumClass.h"        // 枚举类定义
#include "common/mathTools.h"        // 数学工具函数
#include "common/mathTypes.h"        // 数学类型定义（向量、矩阵等）
#include "common/timeMarker.h"       // 时间标记工具
#include "interface/CmdPanel.h"      // 命令面板（用户输入处理）

/**
* @brief 有限状态机状态抽象基类
* 
* 这是所有机器人行为状态的基类，定义了状态机的标准接口。
* 每个具体的机器人行为（如站立、行走、测试等）都需要继承这个类。
* 
* 设计理念：
* - 模板方法模式：定义状态执行的标准流程
* - 策略模式：每个状态封装一种特定的行为策略
* - 多态机制：通过虚函数实现不同状态的特定行为
* 
* 状态生命周期：
* 1. enter()  - 状态初始化：设置初始参数、重置变量
* 2. run()    - 状态执行：核心控制逻辑，每个控制周期调用
* 3. exit()   - 状态清理：保存数据、释放资源、清理工作
* 4. checkChange() - 转换检查：判断是否需要切换到其他状态
*/
class FSMState{
public:
    /**
    * @brief 构造函数
    * @param ctrlComp 控制组件指针，提供对整个控制系统的访问
    * @param stateName 状态名枚举值，用于状态识别和转换
    * @param stateNameString 状态名字符串，用于调试和日志输出
    * 
    * 构造函数负责：
    * 1. 保存控制组件引用，让状态可以访问所有系统资源
    * 2. 设置状态标识，便于状态机管理
    * 3. 初始化快捷访问指针，提高运行效率
    */
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    // ==================== 纯虚函数接口 ====================
    // 这些函数必须被所有子类实现，定义了状态的标准行为
    
    /**
    * @brief 进入状态时的初始化函数 [纯虚函数]
    * 
    * 当状态机切换到这个状态时调用，负责：
    * - 初始化状态相关的变量和参数
    * - 设置控制器的初始配置（增益、模式等）
    * - 记录进入状态时的机器人状态
    * - 准备状态执行所需的资源
    * 
    * 调用时机：状态转换的第一步
    * 调用频率：每次进入状态时调用一次
    */
    virtual void enter() = 0;
    
    /**
    * @brief 状态主执行函数 [纯虚函数]
    * 
    * 状态的核心函数，包含该状态的主要控制逻辑：
    * - 读取传感器数据和用户输入
    * - 执行状态特定的控制算法
    * - 计算并设置机器人控制命令
    * - 更新状态内部变量
    * 
    * 调用时机：状态正常运行期间
    * 调用频率：每个控制周期（500Hz）调用一次
    * 
    * 典型实现流程：
    * 1. 获取用户命令和传感器数据
    * 2. 执行状态特定的控制算法
    * 3. 计算关节控制命令
    * 4. 设置输出命令
    */
    virtual void run() = 0;
    
    /**
    * @brief 退出状态时的清理函数 [纯虚函数]
    * 
    * 当状态机准备离开这个状态时调用，负责：
    * - 保存重要的状态信息
    * - 清理临时资源和变量
    * - 为下一个状态做准备
    * - 执行安全性检查和设置
    * 
    * 调用时机：状态转换过程中，进入新状态之前
    * 调用频率：每次离开状态时调用一次
    */
    virtual void exit() = 0;
    
    /**
    * @brief 检查状态转换条件 [虚函数，可重写]
    * @return 下一个目标状态的枚举值
    * 
    * 检查是否满足状态转换的条件：
    * - 分析用户输入命令
    * - 检查安全条件
    * - 评估任务完成情况
    * - 判断异常情况
    * 
    * 调用时机：每个控制周期的状态执行后
    * 调用频率：每个控制周期（500Hz）调用一次
    * 
    * 返回值说明：
    * - 返回当前状态名：继续保持当前状态
    * - 返回其他状态名：请求切换到该状态
    * - 返回INVALID：异常情况或错误
    * 
    * 默认实现：返回INVALID，子类应该重写此函数
    */
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    // ==================== 公共成员变量 ====================
    
    /**
    * @brief 状态名枚举
    * 用于状态机内部的状态识别和转换判断
    * 例如：FSMStateName::PASSIVE, FSMStateName::TROTTING
    */
    FSMStateName _stateName;
    
    /**
    * @brief 状态名字符串
    * 用于调试输出、日志记录和用户界面显示
    * 例如："passive", "trotting", "fixed stand"
    */
    std::string _stateNameString;

protected:
    // ==================== 受保护成员变量 ====================
    // 这些变量只能被子类访问，提供了状态实现所需的基础资源
    
    /**
    * @brief 控制组件指针
    * 提供对整个控制系统的访问，包括：
    * - 机器人模型和参数
    * - 步态生成器
    * - 状态估计器
    * - 平衡控制器
    * - IO通信接口
    * - 所有传感器数据和控制命令
    */
    CtrlComponents *_ctrlComp;
    
    /**
    * @brief 下一个状态名
    * 临时变量，用于存储状态转换的目标状态
    * 在checkChange()函数中设置，由状态机使用
    */
    FSMStateName _nextStateName;

    // ==================== 快捷访问指针 ====================
    // 为了提高访问效率，直接保存常用数据的指针
    
    /**
    * @brief 底层控制命令指针
    * 直接指向控制组件中的lowCmd，方便快速访问
    * 用于设置：
    * - 12个关节的目标位置、速度、力矩
    * - 控制增益参数（Kp, Kd）
    * - 控制模式选择
    */
    LowlevelCmd *_lowCmd;
    
    /**
    * @brief 底层状态反馈指针
    * 直接指向控制组件中的lowState，方便快速访问
    * 用于读取：
    * - 12个关节的实际位置、速度、力矩
    * - IMU数据（姿态、角速度、加速度）
    * - 用户输入命令
    * - 足端接触信息
    */
    LowlevelState *_lowState;
    
    /**
    * @brief 用户数值输入
    * 存储用户通过操作界面输入的数值参数
    * 可能包含：
    * - 运动速度设定值
    * - 姿态调整参数
    * - 测试参数等
    */
    UserValue _userValue;
};

#endif  // FSMSTATE_H

/*
* ==================== 设计模式分析 ====================
* 
* ## 🎨 模板方法模式 (Template Method Pattern)
* 
* FSMState定义了状态执行的标准模板：
* ```
* 状态转换流程：
* 1. currentState->exit()    // 退出当前状态
* 2. currentState = newState // 切换状态指针  
* 3. currentState->enter()   // 进入新状态
* 
* 状态运行流程：
* 1. currentState->run()        // 执行状态逻辑
* 2. currentState->checkChange() // 检查转换条件
* ```
* 
* ## 🎯 策略模式 (Strategy Pattern)
* 
* 每个具体状态类实现不同的行为策略：
* - State_Passive: 被动策略（关节松弛）
* - State_FixedStand: 固定站立策略（位置控制）
* - State_Trotting: 动态行走策略（步态控制）
* - State_BalanceTest: 平衡测试策略（力控制）
* 
* ## 🔗 多态机制 (Polymorphism)
* 
* 通过虚函数实现运行时多态：
* ```cpp
* FSMState *currentState = new State_Trotting(ctrlComp);
* currentState->run();  // 调用State_Trotting::run()
* ```
* 
* ==================== 内存管理策略 ====================
* 
* ## 📦 对象生命周期
* 
* 1. **创建阶段**：
*    - 所有状态对象在FSM构造时创建
*    - 对象在整个程序运行期间常驻内存
*    - 避免运行时的动态分配开销
* 
* 2. **运行阶段**：
*    - 状态切换只是指针操作，无内存分配
*    - 通过enter()/exit()管理状态资源
*    - 高效的实时性能
* 
* 3. **销毁阶段**：
*    - FSM析构时统一清理所有状态对象
*    - 防止内存泄露
* 
* ## 🚀 性能优化特点
* 
* 1. **快捷指针**：
*    ```cpp
*    _lowCmd = _ctrlComp->lowCmd;    // 避免多层解引用
*    _lowState = _ctrlComp->lowState; // 提高访问效率
*    ```
* 
* 2. **对象复用**：
*    - 状态对象创建一次，使用整个生命周期
*    - 状态切换无内存分配开销
*    - 适合500Hz高频实时控制
* 
* 3. **接口简化**：
*    - 四个核心函数覆盖所有状态行为
*    - 清晰的职责分离
*    - 便于理解和实现
* 
* ==================== 扩展性设计 ====================
* 
* ## ➕ 添加新状态的步骤
* 
* 1. **创建状态类**：
*    ```cpp
*    class State_NewBehavior : public FSMState {
*    public:
*        State_NewBehavior(CtrlComponents *ctrlComp);
*        void enter() override;
*        void run() override;
*        void exit() override;
*        FSMStateName checkChange() override;
*    };
*    ```
* 
* 2. **添加状态枚举**：
*    ```cpp
*    enum class FSMStateName {
*        // ... 现有状态
*        NEWBEHAVIOR,  // 新状态
*    };
*    ```
* 
* 3. **更新状态列表**：
*    ```cpp
*    struct FSMStateList {
*        // ... 现有状态指针
*        State_NewBehavior *newBehavior;
*    };
*    ```
* 
* 4. **更新状态机**：
*    - 在FSM构造函数中创建状态对象
*    - 在getNextState()中添加映射
*    - 在deletePtr()中添加清理代码
* 
* ## 🔧 自定义控制逻辑
* 
* 每个状态可以：
* - 实现独特的控制算法
* - 使用不同的传感器数据
* - 设置特定的控制参数
* - 定义专门的转换条件
* 
* 这种设计使得系统具有极强的扩展能力！
*/