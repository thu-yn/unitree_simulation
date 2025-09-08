/**********************************************************************
 * 文件名: KeyBoard.h
 * 作用: 四足机器人键盘控制接口类，实现通过键盘按键控制机器人运动、状态切换和参数调节
 * 功能概述:
 *   - 继承自CmdPanel基类，提供键盘输入的人机交互接口
 *   - 实时监听键盘输入，将按键转换为机器人控制命令
 *   - 支持运动控制(wasd)、状态切换(数字键)、参数调节等功能
 *   - 采用多线程设计，独立的键盘监听线程避免阻塞主控制循环
 *   - 提供非阻塞式键盘输入检测，保证实时性要求
 * 
 * 在unitree_guide控制框架中的位置:
 *   - 作为interface层的一部分，属于人机交互接口模块
 *   - 为FSM状态机提供用户输入源，控制状态转换
 *   - 与IOInterface配合，实现用户指令到底层控制命令的转换
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

// 标准系统头文件
#include <stdio.h>          // 标准输入输出，用于键盘输入处理
#include <stdlib.h>         // 标准库函数
#include <sys/time.h>       // 时间相关函数，用于输入超时控制
#include <sys/types.h>      // 系统数据类型定义
#include <unistd.h>         // UNIX标准函数，文件描述符操作
#include <fcntl.h>          // 文件控制函数，用于设置非阻塞模式
#include <termios.h>        // 终端I/O控制，用于设置终端属性

// 项目内部头文件
#include "interface/CmdPanel.h"  // 命令面板基类，定义通用的用户命令接口
#include "common/mathTools.h"    // 数学工具库，提供向量、矩阵等数学运算

/**
 * @class KeyBoard
 * @brief 键盘控制接口类
 * @details 继承自CmdPanel基类，实现键盘输入到机器人控制命令的转换。
 *          该类采用多线程架构，通过独立的键盘监听线程实现实时按键检测，
 *          避免阻塞主控制循环，保证控制系统的实时性能。
 * 
 * 主要功能包括：
 * 1. 运动控制：w/a/s/d键控制前进/左转/后退/右转
 * 2. 状态切换：数字键切换机器人工作模式（被动/站立/行走等）
 * 3. 参数调节：通过特定按键组合调节运动参数
 * 4. 紧急停止：提供紧急停止功能保证安全
 * 
 * 控制键位映射（典型配置）：
 * - '2': 进入固定站立模式(FIXEDSTAND)
 * - '4': 进入小跑步态模式(TROTTING)  
 * - 'w': 前进运动
 * - 's': 后退运动
 * - 'a': 左转运动
 * - 'd': 右转运动
 * - 'q/e': 左右侧移
 * - 空格: 紧急停止
 */
class KeyBoard : public CmdPanel{
public:
    /**
     * @brief 构造函数
     * @details 初始化键盘控制器，设置终端属性为非缓冲模式，
     *          创建键盘监听线程，准备接收用户输入。
     *          配置终端为原始模式以实现按键的即时响应。
     */
    KeyBoard();
    
    /**
     * @brief 析构函数
     * @details 清理资源，恢复终端原始设置，
     *          停止键盘监听线程，确保程序正常退出。
     */
    ~KeyBoard();

private:
    /**
     * @brief 静态线程入口函数
     * @param arg 线程参数，指向KeyBoard实例的指针
     * @return void* 线程返回值（未使用）
     * @details 线程的静态入口点，用于创建pthread线程。
     *          该函数将调用转发给非静态的run()方法。
     */
    static void* runKeyBoard(void *arg);
    
    /**
     * @brief 键盘监听线程主循环
     * @param arg 线程参数（未使用）
     * @return void* 线程返回值（未使用）
     * @details 键盘监听的核心循环，持续监听键盘输入事件。
     *          使用select()系统调用实现非阻塞输入检测，
     *          避免CPU占用过高，同时保证响应及时性。
     */
    void* run(void *arg);
    
    /**
     * @brief 检查并处理按键命令
     * @return UserCommand 解析后的用户命令结构
     * @details 将检测到的按键转换为标准化的用户命令。
     *          根据当前按键状态和按键历史，生成相应的控制指令，
     *          包括运动方向、速度设置、模式切换等。
     */
    UserCommand checkCmd();
    
    /**
     * @brief 动态调节控制参数
     * @details 根据特定按键组合调节运动控制参数，
     *          如线速度、角速度的敏感度设置。
     *          允许用户在运行时微调机器人的响应特性。
     */
    void changeValue();

    // === 线程管理 ===
    /**
     * @brief 键盘监听线程ID
     * @details 用于管理键盘输入处理的独立线程，
     *          确保键盘输入不会阻塞主控制循环。
     */
    pthread_t _tid;
    
    // === 控制参数 ===
    /**
     * @brief 左转敏感度系数
     * @details 控制左转角速度的比例系数，值越大转向越快。
     *          典型值0.05，可通过changeValue()方法运行时调节。
     *          用于精细调节机器人的转向响应特性。
     */
    float sensitivityLeft = 0.05;
    
    /**
     * @brief 右转敏感度系数  
     * @details 控制右转角速度的比例系数，值越大转向越快。
     *          典型值0.05，可通过changeValue()方法运行时调节。
     *          与sensitivityLeft配合实现均衡的转向控制。
     */
    float sensitivityRight = 0.05;
    
    // === 终端控制 ===
    /**
     * @brief 终端原始设置备份
     * @details 保存程序启动前的终端配置，
     *          用于在程序退出时恢复终端的正常工作模式。
     */
    struct termios _oldSettings;
    
    /**
     * @brief 终端新设置配置
     * @details 配置为原始模式的终端属性，
     *          关闭回显和缓冲，实现按键的即时响应。
     *          设置为非阻塞模式以配合select()使用。
     */
    struct termios _newSettings;
    
    // === 输入检测 ===
    /**
     * @brief 文件描述符集合
     * @details 用于select()系统调用的文件描述符集合，
     *          监控标准输入(stdin)的可读状态。
     */
    fd_set set;
    
    /**
     * @brief select()调用结果
     * @details 存储select()系统调用的返回值，
     *          指示是否有输入事件发生。
     *          >0: 有输入可读; =0: 超时; <0: 错误
     */
    int res;
    
    /**
     * @brief 系统调用返回值
     * @details 通用的系统调用返回值存储变量，
     *          用于错误检测和状态判断。
     */
    int ret;
    
    /**
     * @brief 当前读取的字符
     * @details 存储从键盘读取的单个字符，
     *          用于按键识别和命令解析。
     */
    char _c;
};

#endif  // KEYBOARD_H