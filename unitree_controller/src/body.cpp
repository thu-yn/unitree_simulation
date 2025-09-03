/************************************************************************
* 文件名: body.cpp
* 功能描述: Unitree四足机器人本体控制核心实现文件
* 
* 主要功能:
* 1. 实现机器人基础运动控制算法
* 2. 管理12个关节的PD控制器参数
* 3. 提供平滑运动控制和站立控制功能
* 4. 处理关节控制命令的发送和状态更新
* 
* 在整个项目中的作用:
* - 🔴 核心控制库 - 为上层应用提供基础运动控制能力
* - 被编译为静态库，供其他可执行文件链接使用
* - 实现了机器人从倒地到站立的完整控制流程
* 
* 控制算法特点:
* - 采用PD控制器进行关节位置控制
* - 不同类型关节使用不同的控制参数
* - 支持平滑运动过渡，避免机械冲击
* 
* Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
* Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

/**
* @namespace unitree_model
* @brief Unitree机器人模型控制命名空间实现
*/
namespace unitree_model
{
    // ==================== 全局变量定义 ====================
    
    /**
    * @brief 12个关节的ROS发布器数组
    * 用于向Gazebo仿真环境发送关节控制命令
    */
    ros::Publisher servo_pub[12];
    
    /**
    * @brief 底层控制命令结构体
    * 包含所有关节的控制参数和目标值
    */
    unitree_legged_msgs::LowCmd lowCmd;
    
    /**
    * @brief 底层状态反馈结构体
    * 包含所有关节的实际状态和传感器数据
    */
    unitree_legged_msgs::LowState lowState;

    // ==================== 核心控制函数实现 ====================

    /**
    * @brief PD控制器参数初始化函数
    * 
    * 功能描述:
    * - 为12个关节设置PD控制器参数
    * - 配置关节控制模式
    * - 初始化关节目标位置为当前位置
    * 
    * PD参数设计理念:
    * - Hip关节: 相对较低的增益，保证系统稳定性
    * - Thigh关节: 中等增益，承载身体主要重量
    * - Calf关节: 最高增益，需要精确控制足端位置
    * 
    * 注意事项:
    * - 这些参数仅供仿真参考
    * - 在真实机器人上运行时需要重新调试参数
    */
    void paramInit()
    {
        // 遍历4条腿，每条腿有3个关节(Hip, Thigh, Calf)
        for (int i = 0; i < 4; i++)
        {
            // ========== Hip关节(髋关节)参数设置 ==========
            lowCmd.motorCmd[i * 3 + 0].mode = 0x0A;    // 设置为混合控制模式
            lowCmd.motorCmd[i * 3 + 0].Kp = 70;        // 位置增益: 较低值保证稳定
            lowCmd.motorCmd[i * 3 + 0].dq = 0;         // 目标速度: 初始化为0
            lowCmd.motorCmd[i * 3 + 0].Kd = 3;         // 速度增益: 提供阻尼
            lowCmd.motorCmd[i * 3 + 0].tau = 0;        // 前馈力矩: 初始化为0
            
            // ========== Thigh关节(大腿关节)参数设置 ==========
            lowCmd.motorCmd[i * 3 + 1].mode = 0x0A;    // 设置为混合控制模式
            lowCmd.motorCmd[i * 3 + 1].Kp = 180;       // 位置增益: 中等值，承载重量
            lowCmd.motorCmd[i * 3 + 1].dq = 0;         // 目标速度: 初始化为0
            lowCmd.motorCmd[i * 3 + 1].Kd = 8;         // 速度增益: 适中阻尼
            lowCmd.motorCmd[i * 3 + 1].tau = 0;        // 前馈力矩: 初始化为0
            
            // ========== Calf关节(小腿关节)参数设置 ==========
            lowCmd.motorCmd[i * 3 + 2].mode = 0x0A;    // 设置为混合控制模式
            lowCmd.motorCmd[i * 3 + 2].Kp = 300;       // 位置增益: 最高值，精确控制
            lowCmd.motorCmd[i * 3 + 2].dq = 0;         // 目标速度: 初始化为0
            lowCmd.motorCmd[i * 3 + 2].Kd = 15;        // 速度增益: 最高阻尼
            lowCmd.motorCmd[i * 3 + 2].tau = 0;        // 前馈力矩: 初始化为0
        }
        
        // 将所有关节的目标位置初始化为当前实际位置
        // 这样可以避免初始化时的突然运动
        for (int i = 0; i < 12; i++)
        {
            lowCmd.motorCmd[i].q = lowState.motorState[i].q;
        }
    }

    /**
    * @brief 机器人站立控制函数
    * 
    * 功能描述:
    * - 将机器人控制到标准站立姿态
    * - 使用经过测试的关节角度组合
    * - 通过平滑过渡避免机械冲击
    * 
    * 站立姿态详细说明:
    * 
    * 关节角度设计(弧度):
    * - FR腿: Hip=0.0, Thigh=0.67, Calf=-1.3
    * - FL腿: Hip=-0.0, Thigh=0.67, Calf=-1.3  
    * - RR腿: Hip=0.0, Thigh=0.67, Calf=-1.3
    * - RL腿: Hip=-0.0, Thigh=0.67, Calf=-1.3
    * 
    * 物理意义:
    * - Hip≈0°: 保持身体对称，不向左右倾斜
    * - Thigh≈38.4°: 大腿向前抬起，为身体提供支撑
    * - Calf≈-74.5°: 小腿向后弯曲，足端接触地面
    * 
    * 执行时间: 2秒(2000ms)的平滑过渡
    */
    void stand()
    {
        // 定义标准站立姿态的关节角度数组(12个关节)
        double pos[12] = {
            // 前右腿(FR): Hip, Thigh, Calf
            0.0, 0.67, -1.3,    
            // 前左腿(FL): Hip, Thigh, Calf
            -0.0, 0.67, -1.3,   
            // 后右腿(RR): Hip, Thigh, Calf
            0.0, 0.67, -1.3,    
            // 后左腿(RL): Hip, Thigh, Calf
            -0.0, 0.67, -1.3    
        };
        
        // 执行平滑运动，2秒内过渡到站立姿态
        moveAllPosition(pos, 2 * 1000);
    }

    /**
    * @brief 运动系统初始化函数
    * 
    * 功能描述:
    * - 执行机器人控制系统的完整初始化流程
    * - 确保机器人在后续操作前处于可控状态
    * 
    * 执行流程:
    * 1. paramInit(): 初始化PD控制器参数
    * 2. stand(): 执行站立动作
    * 
    * 调用时机:
    * - 在main函数中，机器人开始控制前调用
    * - 是整个控制系统的入口函数
    */
    void motion_init()
    {
        paramInit();    // 第一步: 初始化控制参数
        stand();        // 第二步: 执行站立动作
    }

    /**
    * @brief 发送伺服控制命令函数
    * 
    * 功能描述:
    * - 将lowCmd中的所有关节控制命令发送到Gazebo
    * - 处理ROS消息队列
    * - 控制发送频率以保证系统稳定性
    * 
    * 实现细节:
    * - 遍历所有12个关节发布器
    * - 发送对应的电机控制命令
    * - 调用ros::spinOnce()处理回调函数
    * - 添加1ms延时确保1000Hz控制频率
    * 
    * 性能特性:
    * - 控制频率: 1000Hz (工业级实时控制)
    * - 延迟: 1ms (满足四足机器人控制要求)
    */
    void sendServoCmd()
    {
        // 遍历所有12个关节，发送控制命令
        for (int m = 0; m < 12; m++)
        {
            servo_pub[m].publish(lowCmd.motorCmd[m]);
        }
        
        // 处理ROS消息队列，确保消息及时发送和接收
        ros::spinOnce();
        
        // 添加1ms延时，维持1000Hz的控制频率
        // 这个频率对于四足机器人的稳定控制是必要的
        usleep(1000);
    }

    /**
    * @brief 平滑运动控制函数
    * 
    * @param targetPos 目标关节位置数组(包含12个关节的目标角度)
    * @param duration 运动持续时间(毫秒)
    * 
    * 功能描述:
    * - 从当前关节位置平滑过渡到目标位置
    * - 使用线性插值算法确保运动平滑
    * - 支持运行时安全中断
    * 
    * 算法原理:
    * 1. 记录所有关节的当前位置作为起始点
    * 2. 在指定时间内进行线性插值
    * 3. 每1ms更新一次关节目标位置
    * 4. 实时检查ROS状态，支持安全停止
    * 
    * 线性插值公式:
    * current_pos = start_pos * (1 - percent) + target_pos * percent
    * 其中 percent = current_time / total_duration
    * 
    * 安全特性:
    * - 实时检查ros::ok()状态
    * - 支持Ctrl+C中断
    * - 避免机器人失控
    */
    void moveAllPosition(double *targetPos, double duration)
    {
        double pos[12], lastPos[12], percent;
        
        // 记录所有关节的当前位置作为运动起点
        for (int j = 0; j < 12; j++)
            lastPos[j] = lowState.motorState[j].q;
        
        // 执行平滑运动控制循环
        for (int i = 1; i <= duration; i++)
        {
            // 安全检查: 如果ROS被中断，立即退出
            if (!ros::ok())
                break;
            
            // 计算当前时刻的运动进度百分比
            percent = (double)i / duration;
            
            // 对每个关节进行线性插值计算
            for (int j = 0; j < 12; j++)
            {
                // 线性插值公式: 
                // 新位置 = 起始位置 * (1-进度) + 目标位置 * 进度
                lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
            }
            
            // 发送当前时刻的关节控制命令
            sendServoCmd();
        }
    }

} // namespace unitree_model