/**
* @file FSM.cpp
* @brief 有限状态机（FSM）核心实现文件
* 
* 文件作用：
* 这是unitree_guide四足机器人控制框架中的核心文件，实现了有限状态机（FSM）的主控制器。
* FSM是整个机器人行为控制的中枢，负责：
* 1. 管理机器人的所有行为状态（被动、站立、行走等）
* 2. 根据条件执行状态转换，确保状态切换的安全性和连续性
* 3. 协调各个控制组件（步态生成、平衡控制、状态估计等）
* 4. 在500Hz高频控制循环中实时运行，确保机器人响应的实时性
* 5. 提供多重安全检查机制，保障系统稳定性
* 
* 该文件采用状态模式设计，将复杂的机器人控制逻辑分解为多个独立的状态类，
* 每个状态封装特定的行为逻辑，使代码结构清晰、易于维护和扩展。
* 
* @copyright Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
*/
#include "FSM/FSM.h"
#include <iostream>

/**
* @brief FSM构造函数
* @param ctrlComp 控制组件指针，包含所有必要的控制组件和接口
* 
* 构造函数的主要任务：
* 1. 保存控制组件指针，用于后续访问各种控制功能
* 2. 创建所有状态对象，建立完整的状态库
* 3. 调用initialize()进行状态机初始化
* 
* 状态创建说明：
* - invalid: 设为nullptr，作为错误状态的标识
* - passive: 被动状态，机器人的初始和安全状态
* - fixedStand: 固定站立状态，机器人保持固定姿态站立
* - freeStand: 自由站立状态，允许机器人调整平衡
* - trotting: 小跑步态状态，机器人的主要行走状态
* - balanceTest: 平衡测试状态，用于调试平衡控制算法
* - swingTest: 摆动测试状态，用于测试单腿摆动动作
* - stepTest: 步态测试状态，用于验证步态生成算法
* - moveBase: ROS导航状态（条件编译），与ROS导航栈集成
*/
FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    // 初始化状态列表中的所有状态对象
    _stateList.invalid = nullptr;  // 无效状态标识，用于错误处理
    _stateList.passive = new State_Passive(_ctrlComp);        // 被动状态：机器人躺下或关节松弛
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);  // 固定站立：保持预设站立姿态
    _stateList.freeStand = new State_FreeStand(_ctrlComp);    // 自由站立：带平衡控制的站立
    _stateList.trotting = new State_Trotting(_ctrlComp);      // 小跑步态：主要运动状态
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp); // 平衡测试：调试平衡算法
    _stateList.swingTest = new State_SwingTest(_ctrlComp);    // 摆动测试：单腿动作测试
    _stateList.stepTest = new State_StepTest(_ctrlComp);      // 步态测试：步态参数验证

// 条件编译：只有在定义了COMPILE_WITH_MOVE_BASE宏时才编译ROS导航功能
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);     // ROS导航状态：与move_base集成
#endif  // COMPILE_WITH_MOVE_BASE

    // 调用初始化函数，设置初始状态
    initialize();
}

/**
* @brief FSM析构函数
* 
* 析构函数的作用：
* 1. 清理所有动态分配的状态对象
* 2. 防止内存泄露
* 3. 确保程序正常退出时资源得到释放
* 
* 注意：使用了状态列表的deletePtr()方法来统一管理内存释放
*/
FSM::~FSM(){
    _stateList.deletePtr();  // 调用状态列表的清理方法，删除所有状态对象
}

/**
* @brief 初始化状态机
* 
* 初始化过程：
* 1. 设置当前状态为PASSIVE（被动状态）
* 2. 调用当前状态的enter()方法，执行状态进入逻辑
* 3. 设置下一状态也为当前状态（避免意外转换）
* 4. 设置状态机模式为NORMAL（正常运行模式）
* 
* PASSIVE状态是机器人的安全状态，在此状态下：
* - 所有关节处于松弛状态或保持安全位置
* - 不执行主动控制动作
* - 等待用户命令切换到其他状态
*/
void FSM::initialize(){
    _currentState = _stateList.passive;    // 设置初始状态为被动状态
    _currentState -> enter();              // 执行状态进入逻辑
    _nextState = _currentState;            // 初始化下一状态指针
    _mode = FSMMode::NORMAL;               // 设置为正常运行模式
}

/**
* @brief 状态机主运行函数
* 
* 这是整个状态机的核心函数，在每个500Hz控制周期中被调用。
* 执行完整的控制流程，确保机器人的实时响应和安全运行。
* 
* 运行流程详解：
* 
* 1. 数据通信阶段：
*    - sendRecv(): 与机器人进行数据收发
*    - 发送控制命令到机器人硬件
*    - 接收机器人的传感器数据和状态信息
* 
* 2. 步态更新阶段：
*    - runWaveGen(): 运行步态生成器
*    - 更新四条腿的相位信息
*    - 生成足端轨迹和接触状态
* 
* 3. 状态估计阶段：
*    - estimator->run(): 运行状态估计器
*    - 融合IMU、编码器等传感器数据
*    - 估计机器人的位置、速度、姿态等状态
* 
* 4. 安全检查阶段：
*    - checkSafty(): 检查系统安全性
*    - 如果发现安全问题，立即切换到被动模式
*    - 多重安全保护机制
* 
* 5. 状态执行阶段：
*    - 根据当前模式（NORMAL/CHANGE）执行不同逻辑
*    - NORMAL模式：执行当前状态的控制逻辑
*    - CHANGE模式：执行状态转换逻辑
* 
* 6. 时间控制阶段：
*    - absoluteWait(): 精确控制时序
*    - 确保每个控制周期严格为2ms（500Hz）
*/
void FSM::run(){
    // ==================== 1. 记录控制周期开始时间 ====================
    _startTime = getSystemTime();  // 获取当前系统时间，用于后续时序控制
    
    // ==================== 2. 数据通信阶段 ====================
    _ctrlComp->sendRecv();         // 与机器人硬件进行数据收发
    
    // ==================== 3. 步态更新阶段 ====================
    _ctrlComp->runWaveGen();       // 运行步态生成器，更新腿部相位和接触状态
    
    // ==================== 4. 状态估计阶段 ====================
    _ctrlComp->estimator->run();   // 运行状态估计器，更新机器人状态信息
    
    // ==================== 5. 安全检查阶段 ====================
    if(!checkSafty()){             // 如果安全检查失败
        _ctrlComp->ioInter->setPassive();  // 立即设置为被动模式，确保安全
    }

    // ==================== 6. 状态执行阶段 ====================
    if(_mode == FSMMode::NORMAL){  // 正常运行模式
        // 执行当前状态的主要控制逻辑
        _currentState->run();
        
        // 检查是否需要状态转换
        _nextStateName = _currentState->checkChange();
        
        // 如果检测到状态转换需求
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;                    // 切换到状态转换模式
            _nextState = getNextState(_nextStateName);  // 获取目标状态对象
            
            // 打印状态转换信息，便于调试和监控
            std::cout << "Switched from " << _currentState->_stateNameString
                    << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){  // 状态转换模式
        // ==================== 状态转换三步骤 ====================
        
        // 步骤1：退出当前状态
        _currentState->exit();    // 执行当前状态的清理和退出逻辑
        
        // 步骤2：切换状态指针
        _currentState = _nextState;  // 将当前状态指针指向新状态
        
        // 步骤3：进入新状态
        _currentState->enter();   // 执行新状态的初始化和进入逻辑
        
        // 状态转换完成，恢复正常运行模式
        _mode = FSMMode::NORMAL;
        
        // 立即执行一次新状态的控制逻辑，确保连续性
        _currentState->run();
    }

    // ==================== 7. 时间控制阶段 ====================
    // 精确等待，确保每个控制周期严格为dt时间（2ms，即500Hz）
    // 参数说明：_startTime是周期开始时间，dt*1000000将秒转换为微秒
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

/**
* @brief 根据状态名获取对应的状态对象
* @param stateName 目标状态的枚举名称
* @return FSMState* 对应的状态对象指针
* 
* 这是状态转换的核心映射函数，将状态枚举转换为具体的状态对象指针。
* 使用switch语句确保所有状态都有明确的映射关系。
* 
* 状态映射说明：
* - INVALID: 返回nullptr，用于错误处理
* - PASSIVE: 返回被动状态对象
* - FIXEDSTAND: 返回固定站立状态对象
* - FREESTAND: 返回自由站立状态对象
* - TROTTING: 返回小跑步态状态对象
* - BALANCETEST: 返回平衡测试状态对象
* - SWINGTEST: 返回摆动测试状态对象
* - STEPTEST: 返回步态测试状态对象
* - MOVE_BASE: 返回ROS导航状态对象（条件编译）
* - default: 返回invalid状态，处理未知状态名
*/
FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:      // 无效状态
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:      // 被动状态
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:   // 固定站立状态
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:    // 自由站立状态
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:     // 小跑步态状态
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:  // 平衡测试状态
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:    // 摆动测试状态
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:     // 步态测试状态
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE        // 条件编译：ROS导航功能
    case FSMStateName::MOVE_BASE:    // ROS导航状态
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:                         // 未知状态名，返回无效状态
        return _stateList.invalid;
        break;
    }
}

/**
* @brief 系统安全检查函数
* @return bool true表示系统安全，false表示存在安全风险
* 
* 安全检查是确保机器人稳定运行的关键机制。
* 该函数在每个控制周期都会被调用，实时监控系统状态。
* 
* 当前实现的安全检查项目：
* 1. 机器人姿态稳定性检查：
*    - 通过旋转矩阵的Z轴分量检查机器人的倾斜程度
*    - getRotMat()(2,2)表示机器人坐标系Z轴与世界坐标系Z轴的夹角余弦值
*    - 当该值小于0.5时，表示倾斜角度超过60度，存在倾倒风险
* 
* 可扩展的安全检查项目（建议添加）：
* - 关节角度限位检查：确保所有关节在安全范围内
* - 关节速度限制检查：防止关节运动过快
* - 通信状态检查：确保与机器人的通信正常
* - 传感器数据有效性检查：验证IMU、编码器等数据的合理性
* - 电池电压检查：防止低电压运行
* - 温度监控：防止关节电机过热
* 
* 安全机制的重要性：
* - 预防机器人损坏：及时发现异常情况，避免硬件损坏
* - 保护环境安全：防止机器人失控对周围环境造成影响
* - 确保实验安全：为研究人员提供安全的实验环境
*/
bool FSM::checkSafty(){
    // 检查机器人的倾斜程度
    // lowState->getRotMat()返回机器人当前的旋转矩阵
    // (2,2)元素表示机器人Z轴与世界坐标系Z轴的夹角余弦值
    // 当cos(angle) < 0.5时，倾斜角度 > 60°，认为存在安全风险
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;  // 机器人倾斜过度，不安全
    }else{
        return true;   // 机器人姿态稳定，安全
    }
}