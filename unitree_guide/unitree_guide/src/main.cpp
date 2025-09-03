/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// ==================== 系统级头文件 ====================
#include <csignal>    // 信号处理（Ctrl+C中断）
#include <iostream>   // 输入输出流
#include <sched.h>    // 进程调度控制
#include <unistd.h>   // Unix标准函数定义

// ==================== 核心控制系统头文件 ====================
#include "Gait/WaveGenerator.h"      // 步态波形生成器
#include "control/BalanceCtrl.h"     // 平衡控制器
#include "control/ControlFrame.h"    // 控制框架主类
#include "control/CtrlComponents.h"  // 控制组件集合

// ==================== 条件编译：硬件接口 ====================
// 真实机器人接口：与实际硬件通信
#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"         // 真实机器人SDK接口
#endif// COMPILE_WITH_REAL_ROBOT

// ROS仿真接口：与Gazebo仿真环境通信
#ifdef COMPILE_WITH_ROS
#include "interface/IOROS.h"         // ROS/Gazebo仿真接口
#include "interface/KeyBoard.h"      // 键盘输入处理
#endif// COMPILE_WITH_ROS

// ==================== 全局控制变量 ====================

/**
* @brief 程序运行状态标志
* 控制主循环的执行：true = 继续运行，false = 退出程序
* 当接收到Ctrl+C信号时，该变量被设置为false
*/
bool running = true;

// ==================== 信号处理函数 ====================

/**
* @brief 信号处理函数：优雅地处理程序中断
* @param sig 信号类型（通常是SIGINT，即Ctrl+C）
* 
* 当用户按下Ctrl+C时，系统会调用此函数：
* 1. 输出提示信息告知用户控制器正在停止
* 2. 设置running为false，让主循环自然退出
* 3. 避免强制终止，确保资源得到正确清理
*/
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;  // 设置退出标志，主循环将自然结束
}

// ==================== 实时调度配置 ====================

/**
* @brief 设置进程为实时调度模式
* 
* 将当前进程设置为FIFO实时调度策略，确保：
* 1. 控制循环具有最高优先级
* 2. 减少操作系统调度延迟
* 3. 保证500Hz控制频率的时间精度
* 4. 提升机器人控制的实时性能
* 
* SCHED_FIFO特点：
* - 先进先出的实时调度策略
* - 比普通进程具有更高优先级
* - 不会被时间片轮转打断
* - 适合对时间要求严格的实时控制
*/
void setProcessScheduler()
{
    pid_t pid = getpid();  // 获取当前进程ID
    sched_param param;     // 调度参数结构

    // 设置为系统支持的最高实时优先级
    /**
    * SCHED_FIFO是Linux的实时调度策略，特点是：
    * 1. 抢占式调度：实时进程会立即抢占普通进程的CPU
    * 2. 先进先出：相同优先级的实时进程按FIFO顺序执行
    * 3. 无时间片：进程会一直运行直到主动让出CPU或被更高优先级进程抢占
    * 4. 确定性延迟：响应时间更可预测
    */
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    // 应用FIFO实时调度策略
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed."
                    << std::endl;
        // 注意：即使失败也继续运行，只是性能可能受影响
    }
}

// ==================== 主程序入口 ====================

/**
* @brief 主函数：四足机器人控制系统的入口点
* @param argc 命令行参数个数
* @param argv 命令行参数数组
* @return 程序退出状态码（0表示正常退出）
* 
* 程序执行流程：
* 1. 系统配置：设置实时调度和输出格式
* 2. 平台检测：根据编译选项选择仿真或真实机器人
* 3. 组件初始化：创建和配置所有控制组件
* 4. 机器人配置：设置机器人模型和步态参数
* 5. 控制循环：500Hz高频实时控制循环
* 6. 资源清理：程序退出时的内存释放
*/
int main(int argc, char **argv)
{
    // ==================== 第一步：系统级配置 ====================

    /* 设置实时进程调度，确保控制循环的时间精度 */
    setProcessScheduler();

    /* 设置浮点数输出格式：固定小数点，3位精度 */
    std::cout << std::fixed << std::setprecision(3);

    // ==================== 第二步：ROS初始化（可选） ====================

    /* 条件初始化ROS节点（仅在ROS编译环境下） */
#ifdef RUN_ROS
    ros::init(argc, argv, "unitree_gazebo_servo");  // 初始化ROS节点
#endif// RUN_ROS

    // ==================== 第三步：平台检测和接口选择 ====================

    /* 声明IO接口和控制平台类型 */
    IOInterface *ioInter;      // IO接口指针：抽象通信接口
    CtrlPlatform ctrlPlat;     // 控制平台类型：区分仿真/真实机器人

    /* 根据编译配置自动选择合适的IO接口 */

    // 仿真环境配置：使用ROS/Gazebo接口
#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();                    // 创建ROS通信接口
    ctrlPlat = CtrlPlatform::GAZEBO;          // 标记为Gazebo仿真平台
#endif// COMPILE_WITH_SIMULATION

    // 真实机器人配置：使用SDK接口
#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();                    // 创建SDK通信接口
    ctrlPlat = CtrlPlatform::REALROBOT;       // 标记为真实机器人平台
#endif// COMPILE_WITH_REAL_ROBOT

    // ==================== 第四步：核心控制组件创建 ====================

    /* 创建控制组件集合：系统的"大脑中枢" */
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);

    /* 配置控制组件的基本参数 */
    ctrlComp->ctrlPlatform = ctrlPlat;        // 设置控制平台类型
    ctrlComp->dt = 0.002;                     // 设置控制周期：2ms（500Hz频率）
    ctrlComp->running = &running;             // 绑定运行状态标志

    // ==================== 第五步：机器人模型配置 ====================

    /* 根据机器人型号创建对应的运动学模型 */

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();     // 创建A1机器人模型
#endif

#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();    // 创建Go1机器人模型
#endif

    // ==================== 第六步：步态生成器配置 ====================

    /* 创建步态生成器并设置默认的小跑步态 */

    // 小跑步态（Trot）参数详解：
    // - 参数1 (0.45): 步态周期 = 0.45秒，即完成一个完整步态循环的时间
    // - 参数2 (0.5):  支撑相比例 = 50%，即每条腿接触地面的时间占整个周期的一半
    // - 参数3 Vec4(0, 0.5, 0.5, 0): 四条腿的相位偏移
    //   * [前右腿, 前左腿, 后右腿, 后左腿] = [0, 0.5, 0.5, 0]
    //   * 前右腿和后左腿同相位（0），前左腿和后右腿同相位（0.5）
    //   * 这形成了对角线步态：对角线上的腿同时着地/抬起
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot 小跑步态

    // ==================== 其他可选步态（已注释） ====================
    // 以下步态仅适用于仿真环境，可根据需要启用：

    // 爬行步态（Crawl）：慢速但稳定，四条腿依次抬起
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim

    // 行走小跑（Walking Trot）：比标准小跑更慢更稳定
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim

    // 快跑小跑（Running Trot）：比标准小跑更快，支撑相时间更短
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim

    // 跳跃步态（Pronk）：四条腿同时动作，适合跳跃运动
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    // ==================== 第七步：高级控制对象生成 ====================

    /* 生成其他控制组件：状态估计器、平衡控制器等 */
    // 这个函数会创建：
    // - Estimator: 状态估计器（融合传感器数据）
    // - BalanceCtrl: 平衡控制器（重心和姿态控制）
    // - 调试工具: 如果启用DEBUG模式，还会创建可视化工具
    ctrlComp->geneObj();

    // ==================== 第八步：控制框架创建 ====================

    /* 创建控制框架：整合FSM状态机和控制循环 */
    // ControlFrame是系统的最高层控制器，包含：
    // - FSM状态机：决策机器人行为
    // - 时间控制：维护500Hz控制频率
    // - 安全监控：系统安全检查
    ControlFrame ctrlFrame(ctrlComp);

    // ==================== 第九步：信号处理注册 ====================

    /* 注册Ctrl+C信号处理函数 */
    // 当用户按下Ctrl+C时，调用ShutDown函数进行优雅退出
    signal(SIGINT, ShutDown);

    // ==================== 第十步：主控制循环 ====================

    /* 500Hz实时控制主循环 */
    // 循环执行直到running变为false（接收到退出信号）
    // 每次循环执行一次完整的控制流程：
    // 1. 数据收发：与机器人通信
    // 2. 状态估计：更新机器人状态
    // 3. 决策执行：运行状态机逻辑
    // 4. 命令输出：发送控制命令
    // 5. 时间同步：等待下一个2ms周期
    while (running)
    {
        ctrlFrame.run();  // 执行一次控制循环
    }

    // ==================== 第十一步：资源清理 ====================

    /* 程序退出时的内存清理 */
    // ctrlComp的析构函数会自动清理所有子组件：
    // - 删除IO接口、机器人模型、步态生成器
    // - 删除状态估计器、平衡控制器
    // - 删除所有状态机状态对象
    // - 防止内存泄露
    delete ctrlComp;

    return 0;  // 程序正常退出
}

/*
* ==================== 程序架构总结 ====================
* 
* ## 🏗️ 系统启动流程
* ```
* 系统配置 → 平台检测 → 组件创建 → 机器人配置 
*     ↓
* 步态设置 → 对象生成 → 控制框架 → 信号注册
*     ↓
* 500Hz主循环运行 → 接收退出信号 → 资源清理
* ```
* 
* ## ⚙️ 控制频率设计
* - **500Hz控制频率**：每2毫秒执行一次控制循环
* - **实时调度**：SCHED_FIFO确保时间精度
* - **高频优势**：
*   * 更平滑的运动控制
*   * 更快的响应速度
*   * 更好的稳定性
* 
* ## 🔧 平台适配机制
* 通过编译时宏定义实现一套代码多平台支持：
* - `COMPILE_WITH_SIMULATION` → Gazebo仿真
* - `COMPILE_WITH_REAL_ROBOT` → 真实机器人
* - `ROBOT_TYPE_A1` / `ROBOT_TYPE_Go1` → 不同机器人型号
* 
* ## 🎯 步态参数解析
* 小跑步态 `Vec4(0, 0.5, 0.5, 0)` 的含义：
* ```
* 时间轴:  0----0.25----0.5----0.75----1.0
* 前右腿:  着地----------抬起----------着地
* 前左腿:  ----抬起----------着地------
* 后右腿:  ----抬起----------着地------
* 后左腿:  着地----------抬起----------着地
* ```
* 
* ## 🛡️ 安全机制
* 1. **信号处理**：优雅响应Ctrl+C中断
* 2. **内存管理**：统一的资源分配和释放
* 3. **实时调度**：避免系统调度延迟
* 4. **错误检查**：关键操作的失败处理
* 
* ## 🚀 性能优化特点
* 1. **编译时优化**：条件编译减少运行时开销
* 2. **内存预分配**：避免运行时动态分配
* 3. **实时调度**：系统级性能保证
* 4. **高效数据结构**：针对实时控制优化
* 
* 这种设计使得系统既具有高性能的实时控制能力，
* 又具有良好的可维护性和扩展性！
*/