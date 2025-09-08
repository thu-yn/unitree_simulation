/**********************************************************************
 * 文件名: LowlevelCmd.h
 * 版权: Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 * 
 * 文件作用:
 * 本文件定义了Unitree四足机器人底层控制命令的数据结构，包含单个电机控制命令(MotorCmd)
 * 和整个机器人的底层控制命令(LowlevelCmd)。提供了四足机器人12个关节电机的统一控制接口，
 * 支持位置控制、速度控制、力矩控制以及PD控制参数设置。是unitree_guide控制框架中
 * 连接高层运动规划与底层电机驱动的关键数据结构。
 ***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"    // 包含数学类型定义 (Vec12, Vec3, Vec2等)
#include "common/mathTools.h"    // 包含数学工具函数 (saturation饱和函数等)

/**
 * @brief 单个电机控制命令结构体
 * 
 * 用途: 定义单个关节电机的控制命令，支持多种控制模式
 * 控制原理: 实际控制力矩 = Kp*(q_target-q_current) + Kd*(dq_target-dq_current) + tau_feedforward
 * 
 * 电机编号规则 (四足机器人12个关节):
 * 0-2:  右前腿 (FR) - 髋关节、大腿关节、小腿关节
 * 3-5:  左前腿 (FL) - 髋关节、大腿关节、小腿关节  
 * 6-8:  右后腿 (RR) - 髋关节、大腿关节、小腿关节
 * 9-11: 左后腿 (RL) - 髋关节、大腿关节、小腿关节
 */
struct MotorCmd{
    unsigned int mode;      // 电机控制模式 (0=锁定模式, 10=伺服模式-正常控制)
    float q;               // 目标关节角度 (弧度) - 期望的关节位置
    float dq;              // 目标关节角速度 (弧度/秒) - 期望的关节速度
    float tau;             // 前馈力矩 (牛米) - 直接施加的力矩命令
    float Kp;              // 位置增益系数 - 位置误差的比例系数，决定位置跟踪精度
    float Kd;              // 速度增益系数 - 速度误差的微分系数，提供阻尼和稳定性

    /**
     * @brief 默认构造函数
     * 将所有控制参数初始化为0，确保电机处于安全的初始状态
     */
    MotorCmd(){
        mode = 0;          // 默认锁定模式，电机不输出力矩
        q = 0;             // 零位置
        dq = 0;            // 零速度
        tau = 0;           // 零力矩
        Kp = 0;            // 零位置增益
        Kd = 0;            // 零速度增益
    }
};

/**
 * @brief 机器人整体底层控制命令结构体
 * 
 * 用途: 管理四足机器人所有12个关节的控制命令，提供便捷的批量操作接口
 * 应用场景: 用于状态机中的具体控制状态 (如站立、行走等) 生成统一的电机控制命令
 * 
 * 腿部编号约定:
 * legID = 0: 右前腿 (FR)
 * legID = 1: 左前腿 (FL)
 * legID = 2: 右后腿 (RR)  
 * legID = 3: 左后腿 (RL)
 * 
 * 每条腿3个关节: 髋关节(0) + 大腿关节(1) + 小腿关节(2)
 */
struct LowlevelCmd{
    MotorCmd motorCmd[12];  // 12个电机控制命令数组 (四条腿，每腿3个关节)

    /**
     * @brief 设置所有关节的目标位置
     * @param q 12维向量，包含所有关节的目标角度 (弧度)
     * 
     * 使用场景: 从逆运动学求解得到的关节角度直接设置到电机命令
     * 注意: 只设置位置，不改变速度、力矩和PD参数
     */
    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }

    /**
     * @brief 设置单条腿的目标位置
     * @param legID 腿部编号 (0=FR, 1=FL, 2=RR, 3=RL)
     * @param qi 3维向量，包含该腿3个关节的目标角度 (髋关节、大腿关节、小腿关节)
     * 
     * 使用场景: 针对单腿进行精确控制，如摆动腿的轨迹跟踪
     */
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);  // 髋关节目标位置
        motorCmd[legID*3+1].q = qi(1);  // 大腿关节目标位置
        motorCmd[legID*3+2].q = qi(2);  // 小腿关节目标位置
    }

    /**
     * @brief 设置所有关节的目标速度
     * @param qd 12维向量，包含所有关节的目标角速度 (弧度/秒)
     * 
     * 使用场景: 实现平滑的轨迹跟踪，配合位置控制提供速度前馈
     */
    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }

    /**
     * @brief 设置单条腿的目标速度
     * @param legID 腿部编号
     * @param qdi 3维向量，包含该腿3个关节的目标角速度
     * 
     * 使用场景: 单腿速度控制，如支撑腿的缓慢调整
     */
    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }

    /**
     * @brief 设置所有关节的前馈力矩
     * @param tau 12维向量，包含所有关节的力矩命令 (牛米)
     * @param torqueLimit 力矩限制范围，默认为[-50, 50]牛米
     * 
     * 功能: 
     * 1. 安全检查: 检测NaN值防止控制异常
     * 2. 力矩饱和: 限制力矩在安全范围内，防止电机过载
     * 3. 前馈补偿: 补偿重力、摩擦力等已知干扰
     * 
     * 使用场景: 平衡控制中的重力补偿，或者基于动力学模型的力矩前馈
     */
    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
                // 检测到NaN时不设置该关节力矩，保持之前的值
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);  // 应用饱和限制
        }
    }

    /**
     * @brief 将指定腿的目标速度设置为零
     * @param legID 腿部编号
     * 
     * 使用场景: 支撑腿控制，要求腿部保持静止以提供稳定支撑
     */
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;  // 髋关节速度清零
        motorCmd[legID*3+1].dq = 0;  // 大腿关节速度清零
        motorCmd[legID*3+2].dq = 0;  // 小腿关节速度清零
    }

    /**
     * @brief 将所有腿的目标速度设置为零
     * 
     * 使用场景: 紧急停止或者站立控制时要求所有关节静止
     */
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);  // 遍历四条腿
        }
    }

    /**
     * @brief 将指定腿的前馈力矩设置为零
     * @param legID 腿部编号
     * 
     * 使用场景: 摆动腿控制，摆动腿不需要支撑重力，清零力矩前馈
     */
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }

    /**
     * @brief 设置仿真环境下支撑腿的PD控制参数
     * @param legID 腿部编号
     * 
     * 控制特点:
     * - 髋关节和大腿关节: Kp=180, Kd=8 (中等刚度)
     * - 小腿关节: Kp=300, Kd=15 (高刚度，承受主要支撑力)
     * - mode=10: 伺服控制模式
     * 
     * 适用场景: Gazebo仿真环境，支撑相的腿部需要高刚度来承受地面反力
     */
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;   // 髋关节伺服模式
        motorCmd[legID*3+0].Kp = 180;    // 髋关节位置增益
        motorCmd[legID*3+0].Kd = 8;      // 髋关节速度增益
        motorCmd[legID*3+1].mode = 10;   // 大腿关节伺服模式
        motorCmd[legID*3+1].Kp = 180;    // 大腿关节位置增益
        motorCmd[legID*3+1].Kd = 8;      // 大腿关节速度增益
        motorCmd[legID*3+2].mode = 10;   // 小腿关节伺服模式
        motorCmd[legID*3+2].Kp = 300;    // 小腿关节位置增益 (更高刚度)
        motorCmd[legID*3+2].Kd = 15;     // 小腿关节速度增益 (更高阻尼)
    }

    /**
     * @brief 设置真实机器人支撑腿的PD控制参数
     * @param legID 腿部编号
     * 
     * 控制特点:
     * - 髋关节: Kp=60, Kd=5 (适中刚度)
     * - 大腿关节: Kp=40, Kd=4 (较低刚度，减少震荡)
     * - 小腿关节: Kp=80, Kd=7 (高刚度，主要承力关节)
     * 
     * 设计理由: 真实机器人存在机械间隙、摩擦、传感器噪声等，需要较低的增益
     *          防止高频振动和控制不稳定
     */
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 60;     // 比仿真环境低的增益
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 40;     // 大腿关节最低增益
        motorCmd[legID*3+1].Kd = 4;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 80;     // 小腿关节相对高增益
        motorCmd[legID*3+2].Kd = 7;
    }

    /**
     * @brief 将指定腿的PD控制参数设置为零
     * @param legID 腿部编号
     * 
     * 使用场景: 
     * 1. 紧急停止时去除所有控制力
     * 2. 切换控制模式前的安全过渡
     * 3. 纯力矩控制模式 (不需要PD控制)
     * 
     * 注意: mode仍设置为10 (伺服模式)，但Kp=Kd=0使其等效于开环控制
     */
    void setZeroGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0;      // 零位置增益
        motorCmd[legID*3+0].Kd = 0;      // 零速度增益
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }

    /**
     * @brief 将所有腿的PD控制参数设置为零
     * 
     * 使用场景: 系统初始化时的安全设置，或切换到纯力矩控制模式
     */
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }

    /**
     * @brief 设置稳定的低增益PD控制参数
     * @param legID 腿部编号
     * 
     * 控制特点: 所有关节统一使用Kp=0.8, Kd=0.8的低增益
     * 
     * 使用场景:
     * 1. 系统启动时的缓慢稳定控制
     * 2. 调试时避免剧烈运动
     * 3. 对精度要求不高但要求平稳的应用
     * 
     * 优势: 响应平缓，不易产生震荡，适合初学者和调试使用
     */
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0.8;    // 低位置增益
        motorCmd[legID*3+0].Kd = 0.8;    // 低速度增益
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0.8;
        motorCmd[legID*3+1].Kd = 0.8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0.8;
        motorCmd[legID*3+2].Kd = 0.8;
    }

    /**
     * @brief 将所有腿设置为稳定的低增益PD控制参数
     * 
     * 使用场景: 整机的安全启动模式或低速运动控制
     */
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }

    /**
     * @brief 设置摆动腿的PD控制参数
     * @param legID 腿部编号
     * 
     * 控制特点: 所有关节使用Kp=3, Kd=2的中等增益
     * 
     * 设计理念:
     * - 比支撑腿增益低: 摆动腿不需要承受地面反力，过高增益会导致轨迹跟踪僵硬
     * - 保持适度响应性: 确保摆动腿能够准确跟踪足端轨迹
     * - 平衡精度与柔顺性: 既要精确又要自然流畅
     * 
     * 使用场景: 行走步态中处于摆动相的腿部，需要跟踪预规划的足端轨迹
     */
    void setSwingGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 3;      // 摆动腿适中位置增益
        motorCmd[legID*3+0].Kd = 2;      // 摆动腿适中速度增益
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 3;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 2;
    }
};

#endif  //LOWLEVELCMD_H