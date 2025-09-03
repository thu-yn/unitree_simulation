/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
* @file LowPassFilter.cpp
* @brief 低通滤波器实现文件
* 
* 该文件实现了一个简单的一阶低通滤波器（Low Pass Filter），主要用于：
* 1. 平滑传感器数据，去除高频噪声
* 2. 对控制信号进行滤波，防止突变引起的震荡
* 3. 在机器人控制中，常用于IMU数据、关节角度、速度等信号的滤波
* 
* 低通滤波器的工作原理：
* - 允许低频信号通过，衰减高频信号
* - 使用指数加权移动平均的方式实现
* - 滤波后的输出 = 权重 * 当前输入 + (1-权重) * 历史输出
*/

#include "common/LowPassFilter.h"
#include <math.h>

/**
* @brief 低通滤波器构造函数
* @param samplePeriod 采样周期 (秒)，即两次采样之间的时间间隔
* @param cutFrequency 截止频率 (Hz)，超过此频率的信号将被大幅衰减
* 
* 构造函数主要完成：
* 1. 根据采样周期和截止频率计算滤波器权重系数
* 2. 初始化滤波器状态标志
* 
* 权重计算公式：
* weight = 1 / (1 + 1/(2π * T * fc))
* 其中：T = samplePeriod（采样周期），fc = cutFrequency（截止频率）
* 
* 权重特性：
* - 权重越大，滤波器响应越快，但滤波效果越弱
* - 权重越小，滤波效果越强，但响应越慢
* - 当截止频率很高时，权重接近1，滤波效果很弱
* - 当截止频率很低时，权重接近0，滤波效果很强
*/
LPFilter::LPFilter(double samplePeriod, double cutFrequency){
    // 计算滤波器权重系数
    // 公式基于一阶低通滤波器的离散化实现
    // 2π * samplePeriod * cutFrequency 是归一化频率参数
    _weight = 1.0 / ( 1.0 + 1.0/(2.0*M_PI * samplePeriod * cutFrequency) );
    
    // 初始化启动标志为false
    // 用于标识滤波器是否已经接收到第一个数据值
    _start  = false;
}

/**
* @brief 向滤波器添加新的数值
* @param newValue 新的输入数值
* 
* 该函数实现了低通滤波的核心算法：
* 1. 如果是第一次调用，直接将输入值作为初始的历史值
* 2. 否则，使用指数加权移动平均公式更新历史值
* 
* 滤波公式：
* output(k) = α * input(k) + (1-α) * output(k-1)
* 其中：α = _weight（权重系数）
*/
void LPFilter::addValue(double newValue){
    // 检查是否是滤波器启动后的第一个数据
    if(!_start){
        _start = true;         // 标记滤波器已启动
        _pastValue = newValue; // 将第一个输入值直接作为历史值，避免初值影响
    }
    else{
        // 执行低通滤波计算
        // 新的输出 = 权重 * 当前输入 + (1-权重) * 上次输出
        // 这是一阶IIR滤波器的标准递推公式
        _pastValue = _weight * newValue + (1 - _weight) * _pastValue;
    }
}

/**
* @brief 获取当前滤波后的数值
* @return 返回经过低通滤波处理后的数值
* 
* 该函数返回滤波器的当前输出值，即经过低通滤波处理后的结果。
* 如果滤波器尚未开始工作（未调用过addValue），返回值将是未定义的。
*/
double LPFilter::getValue(){
    return _pastValue;
}

/**
* @brief 清除滤波器状态，重新初始化
* 
* 该函数用于重置滤波器到初始状态：
* 1. 将启动标志设为false
* 2. 下次调用addValue时，新的输入值将作为初始的历史值
* 
* 使用场景：
* - 当传感器重新校准时
* - 当需要重新开始滤波过程时
* - 当检测到数据异常，需要重置滤波器时
*/
void LPFilter::clear(){
    _start = false;  // 重置启动标志，下次addValue将重新初始化
}