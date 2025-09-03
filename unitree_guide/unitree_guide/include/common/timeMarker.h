/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TIMEMARKER_H
#define TIMEMARKER_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

/**
 * @file timeMarker.h
 * @brief 实时控制系统的时间管理工具函数集合
 * 
 * 本文件提供了高精度时间测量和控制功能，专为机器人实时控制系统设计。
 * 这些函数在维持控制循环的精确定时方面起到关键作用。
 * 
 * 核心功能：
 * - 微秒级精度的系统时间获取
 * - 秒级时间戳转换
 * - 精确的延时等待函数
 * - 实时控制循环的时间同步
 * 
 * 设计理念：
 * - 高精度：使用微秒级时间测量，满足高频控制需求
 * - 轻量级：内联函数实现，最小化函数调用开销
 * - 跨平台：基于POSIX标准，支持Linux系统
 * - 实时友好：避免系统调用阻塞，保证控制循环时序
 * 
 * 在四足机器人控制中的应用：
 * - 500Hz控制循环的精确定时（每个周期2000微秒）
 * - 步态生成器的相位同步
 * - 传感器数据采样的时间戳记录
 * - 性能分析和调试的时间测量
 */

/**
 * @brief 获取系统时间戳函数（微秒精度）
 * @return long long 返回自Unix纪元以来的微秒数
 * 
 * 功能描述：
 * - 使用gettimeofday()系统调用获取当前时间
 * - 将秒和微秒部分合并为总微秒数
 * - 提供微秒级别的时间精度
 * 
 * 时间计算：
 * - 总微秒数 = 秒数 × 1,000,000 + 微秒数
 * - 精度：1微秒 = 10^-6秒
 * - 范围：可表示约292,000年的时间跨度
 * 
 * 应用场景：
 * - 控制循环的开始时间标记
 * - 算法执行时间的精确测量
 * - 事件时间戳的记录
 * - 系统性能分析和调试
 * 
 * 使用示例：
 * ```cpp
 * long long startTime = getSystemTime();
 * // ... 执行某些操作 ...
 * long long endTime = getSystemTime();
 * long long duration = endTime - startTime;  // 微秒
 * std::cout << "执行耗时: " << duration << " 微秒" << std::endl;
 * ```
 * 
 * 注意事项：
 * - 需要包含 <sys/time.h> 头文件
 * - 在某些系统上可能受到系统时钟调整的影响
 * - 对于相对时间测量比绝对时间更可靠
 */
//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}

/**
 * @brief 获取系统时间戳函数（秒精度）
 * @return double 返回自Unix纪元以来的秒数（带小数部分）
 * 
 * 功能描述：
 * - 基于getSystemTime()函数实现
 * - 将微秒时间戳转换为秒级浮点数
 * - 保留微秒级精度的小数部分
 * 
 * 转换公式：
 * - 秒数 = 微秒数 × 0.000001
 * - 小数部分保持微秒精度
 * 
 * 优势特点：
 * - 便于人类阅读的时间格式
 * - 便于与其他时间库兼容
 * - 适合需要秒级单位的计算
 * - 保持了微秒级的精度
 * 
 * 应用场景：
 * - 日志记录中的时间戳
 * - 与标准时间库的接口
 * - 科学计算中的时间变量
 * - 用户界面显示的时间信息
 * 
 * 使用示例：
 * ```cpp
 * double currentTime = getTimeSecond();
 * std::cout << "当前时间戳: " << std::fixed << std::setprecision(6) 
 *           << currentTime << " 秒" << std::endl;
 * 
 * // 计算时间间隔
 * double startTime = getTimeSecond();
 * // ... 执行操作 ...
 * double elapsed = getTimeSecond() - startTime;
 * std::cout << "耗时: " << elapsed << " 秒" << std::endl;
 * ```
 */
//时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}

/**
 * @brief 绝对时间等待函数
 * @param startTime 开始时间点（微秒时间戳）
 * @param waitTime 需要等待的时间长度（微秒）
 * 
 * 功能描述：
 * - 从指定的开始时间点开始，等待指定的时间长度
 * - 不是相对当前时间的等待，而是绝对时间的同步等待
 * - 如果已经超时则发出警告，但仍会完成等待逻辑
 * - 使用主动轮询方式实现精确等待
 * 
 * 工作原理：
 * 1. 首先检查是否已经超过了预期的等待时间
 * 2. 如果超时，输出警告信息但继续执行
 * 3. 使用while循环主动检查时间，直到达到目标时间
 * 4. 在循环中使用usleep(50)避免100%占用CPU
 * 
 * 超时处理：
 * - 当 (当前时间 - 开始时间) > 等待时间 时，认为超时
 * - 输出详细的警告信息，包括预期等待时间和实际已用时间
 * - 超时后仍继续执行等待逻辑，确保函数行为一致性
 * 
 * 精确等待策略：
 * - 使用50微秒的休眠间隔，平衡CPU占用和响应精度
 * - 主动轮询确保时间精度，避免系统调度延迟
 * - 适用于实时系统的硬实时要求
 * 
 * 典型应用场景：
 * - 控制循环的周期同步：确保每个控制周期准确执行
 * - 传感器采样的定时同步：保证固定频率采样
 * - 多线程任务的时间协调：同步不同处理线程
 * - 实时通信的时序控制：保证数据传输时序
 * 
 * 使用示例：
 * ```cpp
 * // 500Hz控制循环示例（周期2000微秒）
 * long long loopStartTime = getSystemTime();
 * 
 * while(running) {
 *     long long cycleStart = getSystemTime();
 *     
 *     // 执行控制算法
 *     runControlAlgorithm();
 *     
 *     // 等待到周期结束（确保精确的500Hz）
 *     absoluteWait(cycleStart, 2000);  // 2000微秒 = 2毫秒
 * }
 * ```
 * 
 * 性能考虑：
 * - 函数会消耗CPU资源进行主动等待
 * - 比被动等待（如sleep）更精确，但资源消耗更高
 * - 适合对时间精度要求极高的实时控制系统
 * - 不建议在非实时场景下使用
 * 
 * 注意事项：
 * - startTime必须是有效的时间戳，通常来自getSystemTime()
 * - waitTime单位为微秒，需要注意数值大小
 * - 在高负载系统中可能出现超时警告
 * - 长时间等待会持续占用CPU资源
 */
//等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long startTime, long long waitTime){
    if(getSystemTime() - startTime > waitTime){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while(getSystemTime() - startTime < waitTime){
        usleep(50);
    }
}

#endif //TIMEMARKER_H