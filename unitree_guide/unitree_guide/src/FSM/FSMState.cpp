/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "FSM/FSMState.h"

/**
* @brief FSMState构造函数实现
* @param ctrlComp 控制组件指针，提供对整个控制系统的访问
* @param stateName 状态名枚举值，用于状态识别
* @param stateNameString 状态名字符串，用于调试输出
* 
* 这个构造函数虽然看起来简单，但它是整个状态系统的基础。
* 它建立了状态对象与控制系统之间的连接，使得每个状态都能：
* 1. 访问机器人的所有传感器数据
* 2. 发送控制命令到机器人
* 3. 使用所有控制算法组件
* 4. 与其他状态协调工作
*/
FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp),           // 保存控制组件指针
             _stateName(stateName),         // 设置状态名枚举
             _stateNameString(stateNameString) // 设置状态名字符串
{
    // ==================== 建立快捷访问通道 ====================
    
    /**
    * @brief 建立底层控制命令的快捷访问
    * 
    * 将控制组件中的lowCmd指针直接赋值给成员变量，这样做的好处：
    * 1. 性能优化：避免每次访问时的多层指针解引用
    *    原本需要：_ctrlComp->lowCmd->motorCmd[i].q = value;
    *    现在只需：_lowCmd->motorCmd[i].q = value;
    * 
    * 2. 代码简洁：减少代码复杂度，提高可读性
    * 
    * 3. 实时性保证：在500Hz高频控制中，每个CPU周期都很宝贵
    * 
    * lowCmd包含发送给机器人的所有控制命令：
    * - motorCmd[12]: 12个关节的控制命令（位置、速度、力矩、增益）
    * - 控制模式设置
    * - 安全参数设置
    */
    _lowCmd = _ctrlComp->lowCmd;
    
    /**
    * @brief 建立底层状态反馈的快捷访问
    * 
    * 同样建立对lowState的快捷访问，lowState包含：
    * 1. **关节状态信息**：
    *    - motorState[12]: 12个关节的实际位置、速度、力矩
    *    - 关节温度、错误状态等
    * 
    * 2. **IMU传感器数据**：
    *    - 四元数姿态表示
    *    - 角速度和线性加速度
    *    - 机体坐标系下的运动信息
    * 
    * 3. **用户输入命令**：
    *    - userCmd: 手柄或键盘输入的命令
    *    - 用于状态转换的触发条件
    * 
    * 4. **足端接触信息**：
    *    - 四条腿的接触力传感器数据
    *    - 用于步态控制和平衡调节
    * 
    * 5. **系统状态信息**：
    *    - 通信状态、安全状态等
    *    - 用于安全检查和故障处理
    */
    _lowState = _ctrlComp->lowState;
    
    // ==================== 构造函数设计分析 ====================
    
    /*
    * 🎯 为什么构造函数如此简洁？
    * 
    * 1. **单一职责原则**：
    *    构造函数只负责建立基础连接，具体的初始化工作留给enter()函数
    * 
    * 2. **生命周期分离**：
    *    - 对象创建：在程序启动时一次性完成
    *    - 状态初始化：在每次进入状态时执行
    *    这种分离使得状态对象可以被多次使用
    * 
    * 3. **性能考虑**：
    *    - 所有状态对象在程序启动时创建，运行时无动态分配
    *    - 状态切换只是指针操作，极其高效
    *    - 适合500Hz实时控制的要求
    * 
    * 4. **资源共享**：
    *    - 所有状态共享同一套控制组件
    *    - 避免资源重复和冲突
    *    - 便于状态间的数据传递
    */

    /*
    * 🔗 成员初始化列表 vs 构造函数体
    * 
    * 使用成员初始化列表的原因：
    * 1. **效率**：直接初始化，避免先默认构造再赋值
    * 2. **必要性**：const成员和引用成员必须使用初始化列表
    * 3. **清晰性**：明确表达初始化意图
    * 
    * 在构造函数体中的赋值：
    * - 这些是普通指针赋值操作
    * - 建立快捷访问通道
    * - 提高后续访问效率
    */
}

/*
* ==================== 构造函数的深层作用 ====================
* 
* ## 🏗️ 依赖注入模式
* 
* 这个构造函数实现了依赖注入模式：
* - 状态对象不主动创建依赖的控制组件
* - 而是通过构造函数接收外部传入的组件
* - 降低了耦合度，提高了可测试性
* 
* ## 🎭 多态基础建立
* 
* 虽然FSMState是抽象类，但它的构造函数为多态奠定基础：
* ```cpp
* FSMState *state = new State_Trotting(ctrlComp);
* // State_Trotting的构造函数会先调用FSMState的构造函数
* ```
* 
* ## 🔄 状态上下文建立
* 
* 构造函数建立了状态的执行上下文：
* - 状态知道自己是谁（_stateName）
* - 状态知道自己叫什么（_stateNameString）
* - 状态知道如何与系统交互（_ctrlComp, _lowCmd, _lowState）
* 
* ## 🚀 实时性能优化
* 
* 快捷指针的建立是性能优化的体现：
* ```cpp
* // 优化前：需要两次指针解引用
* _ctrlComp->lowCmd->motorCmd[0].q = targetPos;
* 
* // 优化后：只需要一次指针解引用
* _lowCmd->motorCmd[0].q = targetPos;
* ```
* 
* 在500Hz控制频率下，这种优化积少成多，显著提升性能。
* 
* ## 📊 内存布局优化
* 
* 状态对象的内存布局经过精心设计：
* - 常用的指针成员放在类的前面
* - 利用CPU缓存的局部性原理
* - 提高数据访问效率
* 
* ==================== 使用示例 ====================
* 
* ## 创建状态对象
* ```cpp
* CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
* State_Trotting *trottingState = new State_Trotting(ctrlComp);
* ```
* 
* ## 状态对象的典型使用流程
* ```cpp
* // 1. 状态切换时
* currentState->exit();           // 清理当前状态
* currentState = trottingState;   // 切换到新状态
* currentState->enter();          // 初始化新状态
* 
* // 2. 每个控制周期
* currentState->run();            // 执行状态逻辑
* nextState = currentState->checkChange(); // 检查转换条件
* ```
* 
* ## 状态内部的典型访问模式
* ```cpp
* void State_Trotting::run() {
*     // 读取用户输入
*     UserCommand cmd = _lowState->userCmd;
*     
*     // 读取传感器数据
*     float jointPos = _lowState->motorState[0].q;
*     
*     // 执行控制算法
*     float targetPos = calculateTarget();
*     
*     // 发送控制命令
*     _lowCmd->motorCmd[0].q = targetPos;
* }
* ```
* 
* 这种设计模式使得状态的实现既简洁又高效！
*/