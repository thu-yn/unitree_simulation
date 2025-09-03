/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

// 核心消息类型
#include "message/LowlevelCmd.h"      // 底层控制命令消息
#include "message/LowlevelState.h"    // 底层状态反馈消息

// 接口模块
#include "interface/IOInterface.h"    // 抽象IO接口基类
#include "interface/CmdPanel.h"       // 命令面板（用户输入处理）

// 机器人模型和控制组件
#include "common/unitreeRobot.h"      // 机器人模型定义（A1/Go1等）
#include "Gait/WaveGenerator.h"       // 步态波形生成器
#include "control/Estimator.h"        // 状态估计器
#include "control/BalanceCtrl.h"      // 平衡控制器

// 标准库
#include <string>
#include <iostream>

// 调试功能（可选编译）
#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"           // Python绘图接口，用于数据可视化
#endif  // COMPILE_DEBUG

/**
* @brief 控制组件集合结构体
* 
* 这是整个控制系统的核心数据结构，包含了所有必要的控制组件。
* 它负责组织和管理机器人控制所需的各个模块，是系统的"神经中枢"。
* 
* 设计理念：
* - 集中管理：所有控制组件统一管理，便于协调
* - 接口统一：通过IOInterface抽象，支持仿真和真实机器人
* - 模块化：各个组件职责清晰，便于维护和扩展
*/
struct CtrlComponents{
public:
/**
* @brief 构造函数
* @param ioInter IO接口指针（IOROS或IOSDK）
* 
* 构造函数负责初始化基础的数据结构和默认状态
*/
CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
    // 创建底层命令和状态消息对象
    lowCmd = new LowlevelCmd();       // 发送给机器人的控制命令
    lowState = new LowlevelState();   // 从机器人接收的状态信息
    
    // 创建步态相关的状态变量
    contact = new VecInt4;            // 四条腿的接触状态 [0,1,0,1]表示FR和RL腿接触地面
    phase = new Vec4;                 // 四条腿的步态相位 [0.0~1.0]表示步态周期中的位置
    
    // 初始化步态状态：所有腿都不接触地面，相位为0.5（中间位置）
    *contact = VecInt4(0, 0, 0, 0);           // 初始状态：所有腿都悬空
    *phase = Vec4(0.5, 0.5, 0.5, 0.5);       // 初始相位：所有腿都在步态周期中点
}

/**
* @brief 析构函数
* 负责清理所有动态分配的内存，防止内存泄露
*/
~CtrlComponents(){
    delete lowCmd;        // 清理底层命令
    delete lowState;      // 清理底层状态
    delete ioInter;       // 清理IO接口
    delete robotModel;    // 清理机器人模型
    delete waveGen;       // 清理步态生成器
    delete estimator;     // 清理状态估计器
    delete balCtrl;       // 清理平衡控制器
    
#ifdef COMPILE_DEBUG
    delete plot;          // 清理绘图对象（调试模式）
#endif  // COMPILE_DEBUG
}

// ==================== 核心控制组件 ====================

/**
* @brief 底层控制命令
* 包含发送给机器人的所有控制指令，如：
* - 12个关节的目标位置、速度、力矩
* - 控制模式选择
* - 安全参数等
*/
LowlevelCmd *lowCmd;

/**
* @brief 底层状态反馈
* 包含从机器人接收的所有状态信息，如：
* - 12个关节的实际位置、速度、力矩
* - IMU数据（姿态、角速度、加速度）
* - 足端接触力等
*/
LowlevelState *lowState;

/**
* @brief IO通信接口
* 抽象接口，实际可能是：
* - IOROS：与Gazebo仿真环境通信
* - IOSDK：与真实机器人硬件通信
*/
IOInterface *ioInter;

/**
* @brief 机器人运动学/动力学模型
* 包含机器人的物理参数，如：
* - 关节长度、质量分布
* - 运动学正逆解
* - 动力学参数等
* 支持不同型号：A1Robot, Go1Robot等
*/
QuadrupedRobot *robotModel;

/**
* @brief 步态波形生成器
* 负责生成四足机器人的步态模式：
* - 计算四条腿的相位关系
* - 确定支撑相和摆动相的时序
* - 支持多种步态：Trot、Crawl、Bound等
*/
WaveGenerator *waveGen;

/**
* @brief 状态估计器
* 融合多传感器数据估计机器人状态：
* - 机体位置、速度、加速度
* - 机体姿态（欧拉角）
* - 足端位置和速度
* - 滤波和预测
*/
Estimator *estimator;

/**
* @brief 平衡控制器
* 实现机器人的平衡和姿态控制：
* - 重心位置控制
* - 姿态稳定控制
* - 力分配算法
* - 虚拟力计算
*/
BalanceCtrl *balCtrl;

// ==================== 调试工具（可选） ====================

#ifdef COMPILE_DEBUG
/**
* @brief Python绘图接口
* 用于实时数据可视化和调试：
* - 绘制关节角度曲线
* - 显示足端轨迹
* - 监控控制器性能
*/
PyPlot *plot;
#endif  // COMPILE_DEBUG

// ==================== 步态状态变量 ====================

/**
* @brief 四条腿的接触状态
* VecInt4类型，包含4个整数值 [FR, FL, RR, RL]
* - 0: 该腿悬空（摆动相）
* - 1: 该腿接触地面（支撑相）
* 
* 例如：[1, 0, 0, 1] 表示前右腿和后左腿着地（对角小跑）
*/
VecInt4 *contact;

/**
* @brief 四条腿的步态相位
* Vec4类型，包含4个浮点值 [FR, FL, RR, RL]
* 范围：0.0 ~ 1.0，表示在步态周期中的位置
* - 0.0: 步态周期开始
* - 0.5: 步态周期中点
* - 1.0: 步态周期结束
* 
* 例如：[0.0, 0.5, 0.5, 0.0] 表示对角线腿同相
*/
Vec4 *phase;

// ==================== 系统参数 ====================

/**
* @brief 控制周期
* 单位：秒，通常设为0.002（对应500Hz控制频率）
* 这个值决定了控制循环的执行频率，影响系统的实时性
*/
double dt;

/**
* @brief 运行状态标志指针
* 指向main函数中的running变量
* 当接收到Ctrl+C信号时，该标志被设为false，控制循环退出
*/
bool *running;

/**
* @brief 控制平台类型
* 枚举值，可能为：
* - CtrlPlatform::GAZEBO: Gazebo仿真环境
* - CtrlPlatform::REALROBOT: 真实机器人硬件
*/
CtrlPlatform ctrlPlatform;

// ==================== 核心控制方法 ====================

/**
* @brief 发送命令并接收状态
* 这是与机器人通信的核心方法，每个控制周期都会调用
* 
* 执行流程：
* 1. 将lowCmd中的控制命令发送给机器人
* 2. 从机器人接收最新的状态信息到lowState
* 3. 底层处理通信协议和数据转换
*/
void sendRecv(){
    ioInter->sendRecv(lowCmd, lowState);
}

/**
* @brief 运行步态生成器
* 根据当前的步态状态计算四条腿的相位和接触状态
* 
* 参数说明：
* - *phase: 输出四条腿的相位值
* - *contact: 输出四条腿的接触状态
* - _waveStatus: 当前步态模式（STANCE_ALL/SWING_ALL/WAVE_ALL）
*/
void runWaveGen(){
    waveGen->calcContactPhase(*phase, *contact, _waveStatus);
}

/**
* @brief 设置所有腿为支撑相
* 强制让所有腿都接触地面，通常用于：
* - 站立状态
* - 紧急停止
* - 状态转换过程
*/
void setAllStance(){
    _waveStatus = WaveStatus::STANCE_ALL;
}

/**
* @brief 设置所有腿为摆动相
* 强制让所有腿都悬空，通常用于：
* - 躺下状态
* - 系统初始化
* - 测试模式
*/
void setAllSwing(){
    _waveStatus = WaveStatus::SWING_ALL;
}

/**
* @brief 开始正常步态
* 启动正常的步态生成，让机器人按照预设的步态模式运动
* 这是从静态状态转换到动态行走的关键方法
*/
void setStartWave(){
    _waveStatus = WaveStatus::WAVE_ALL;
}

/**
* @brief 生成控制对象
* 创建和初始化所有高级控制组件
* 
* 这个方法在main函数中被调用，负责：
* 1. 创建状态估计器（需要机器人模型、状态信息、步态参数）
* 2. 创建平衡控制器（需要机器人模型）
* 3. 如果启用调试模式，创建绘图对象并绑定到控制器
*/
void geneObj(){
    // 创建状态估计器：融合IMU、关节编码器、步态信息估计机器人状态
    estimator = new Estimator(robotModel, lowState, contact, phase, dt);
    
    // 创建平衡控制器：实现重心控制和力分配
    balCtrl = new BalanceCtrl(robotModel);

#ifdef COMPILE_DEBUG
    // 调试模式：创建可视化工具
    plot = new PyPlot();
    balCtrl->setPyPlot(plot);     // 为平衡控制器绑定绘图工具
    estimator->setPyPlot(plot);   // 为状态估计器绑定绘图工具
#endif  // COMPILE_DEBUG
}

private:
/**
* @brief 当前步态状态
* 内部状态变量，控制步态生成器的行为模式：
* - STANCE_ALL: 所有腿支撑
* - SWING_ALL: 所有腿摆动  
* - WAVE_ALL: 正常步态波形
* 
* 默认值为SWING_ALL，表示初始状态所有腿悬空
*/
WaveStatus _waveStatus = WaveStatus::SWING_ALL;
};

#endif  // CTRLCOMPONENTS_H

/*
* ==================== 设计思想总结 ====================
* 
* CtrlComponents是整个控制系统的"大脑中枢"，体现了以下设计理念：
* 
* 1. **组件化架构**：
*    - 每个功能模块独立封装（估计器、控制器、步态生成器等）
*    - 通过统一接口协调各模块工作
*    - 便于单独测试和替换各个组件
* 
* 2. **数据驱动**：
*    - lowCmd/lowState作为数据交换中心
*    - contact/phase作为步态状态共享变量
*    - 所有组件通过共享数据协同工作
* 
* 3. **平台抽象**：
*    - 通过IOInterface抽象硬件差异
*    - 同一套控制代码适用于仿真和真实机器人
*    - 便于开发、测试和部署
* 
* 4. **实时控制**：
*    - 500Hz高频控制循环
*    - 每个周期：接收状态 → 运行算法 → 发送命令
*    - 确保控制的实时性和稳定性
* 
* 5. **可扩展性**：
*    - 模块化设计便于添加新功能
*    - 调试接口支持开发调试
*    - 支持不同机器人型号和传感器配置
* 
* ==================== 数据流向 ====================
* 
* 输入数据流：
* 机器人硬件 → IOInterface → lowState → 各控制组件
* 
* 输出数据流：
* 各控制组件 → lowCmd → IOInterface → 机器人硬件
* 
* 控制循环：
* 1. sendRecv()         - 与机器人通信
* 2. runWaveGen()       - 更新步态状态
* 3. estimator.run()    - 状态估计
* 4. FSM.run()          - 状态机决策
* 5. 重复上述过程
*/