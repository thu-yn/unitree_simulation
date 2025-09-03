/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

/**
* @class State_Trotting
* @brief 小跑步态状态类 - 四足机器人的核心运动状态
* 
* 这是机器人控制系统中最重要和最复杂的状态，负责：
* 1. 小跑步态的生成和执行（对角腿协调运动）
* 2. 用户运动命令的解析和执行
* 3. 机器人平衡控制和重心调节
* 4. 足端轨迹规划和力控制
* 5. 多传感器数据融合和状态估计
* 6. 实时步态参数调整和优化
* 
* 小跑步态特点：
* - 对角腿同时摆动（FR+RL，FL+RR）
* - 高速稳定的运动模式
* - 支持前进、后退、侧移、转向
* - 动态平衡控制
*/
class State_Trotting : public FSMState{
public:
    /**
    * @brief 构造函数
    * @param ctrlComp 控制组件指针，包含所有控制相关的数据和接口
    * 
    * 初始化步态生成器、控制参数，根据机器人类型设置PD增益
    */
    State_Trotting(CtrlComponents *ctrlComp);
    
    /**
    * @brief 析构函数
    * 释放步态生成器内存
    */
    ~State_Trotting();
    
    /**
    * @brief 状态进入函数
    * 当从其他状态切换到TROTTING状态时调用
    * 
    * 主要功能：
    * 1. 初始化机器人位置和朝向
    * 2. 清零速度命令
    * 3. 重启步态生成器
    * 4. 初始化控制面板
    */
    void enter();
    
    /**
    * @brief 状态运行函数 - 核心控制循环
    * 在TROTTING状态下每个控制周期(500Hz)调用一次
    * 
    * 主要流程：
    * 1. 获取机器人当前状态（位置、速度、姿态）
    * 2. 解析用户运动命令
    * 3. 计算运动指令和轨迹
    * 4. 运行步态生成器，生成足端轨迹
    * 5. 计算平衡控制力矩
    * 6. 计算关节角度和速度指令
    * 7. 设置接触状态和控制增益
    * 8. 输出最终控制命令
    */
    void run();
    
    /**
    * @brief 状态退出函数
    * 当从TROTTING状态切换到其他状态时调用
    * 
    * 主要功能：
    * 1. 清零控制面板
    * 2. 设置所有腿为摆动状态
    */
    void exit();
    
    /**
    * @brief 状态切换检查函数
    * @return FSMStateName 下一个状态名称
    * 
    * 支持的状态切换：
    * - L2_B → PASSIVE（紧急停止，回到被动状态）
    * - L2_A → FIXEDSTAND（停止运动，固定站立）
    * - 其他 → TROTTING（保持运动状态）
    */
    virtual FSMStateName checkChange();
    
    /**
    * @brief 设置高层运动命令（供外部调用）
    * @param vx 前向速度 (m/s)
    * @param vy 侧向速度 (m/s) 
    * @param wz 偏航角速度 (rad/s)
    * 
    * 这个接口供ROS导航或其他高层规划器调用
    */
    void setHighCmd(double vx, double vy, double wz);

private:
    // ==================== 核心算法函数 ====================
    
    /**
    * @brief 计算平衡控制力矩
    * 根据位置误差、速度误差和姿态误差计算机器人所需的力矩
    * 实现重心位置控制和姿态稳定
    */
    void calcTau();
    
    /**
    * @brief 计算关节角度和角速度指令
    * 根据足端目标位置，通过逆运动学计算关节角度
    * 同时计算关节角速度指令
    */
    void calcQQd();
    
    /**
    * @brief 计算运动指令
    * 将用户命令转换为全局坐标系下的速度和位置指令
    * 包括坐标变换、速度限制、积分计算等
    */
    void calcCmd();
    
    /**
    * @brief 获取用户运动命令（虚函数，可被子类重写）
    * 从手柄或键盘获取用户的运动意图
    * 包括前后、左右移动和转向命令
    */
    virtual void getUserCmd();
    
    /**
    * @brief 计算平衡控制增益（暂未实现）
    * 根据运动状态动态调整控制参数
    */
    void calcBalanceKp();
    
    /**
    * @brief 检查是否需要启动步态
    * @return bool true-需要步态运动，false-可以站立不动
    * 
    * 判断依据：
    * - 用户命令速度大小
    * - 位置误差大小
    * - 速度误差大小
    */
    bool checkStepOrNot();

    // ==================== 核心算法对象 ====================
    
    /**
    * @brief 步态生成器指针
    * 负责生成小跑步态的足端轨迹
    * 根据速度命令生成四条腿的协调运动
    */
    GaitGenerator *_gait;
    
    /**
    * @brief 状态估计器指针
    * 提供机器人的位置、速度、姿态等状态信息
    */
    Estimator *_est;
    
    /**
    * @brief 机器人模型指针
    * 提供运动学、动力学计算和机器人参数
    */
    QuadrupedRobot *_robModel;
    
    /**
    * @brief 平衡控制器指针
    * 实现机器人的平衡控制算法
    */
    BalanceCtrl *_balCtrl;

    // ==================== 机器人状态变量 ====================
    
    Vec3  _posBody, _velBody;                       ///< 机体位置和速度（全局坐标系）
    double _yaw, _dYaw;                             ///< 偏航角和偏航角速度
    Vec34 _posFeetGlobal, _velFeetGlobal;           ///< 足端位置和速度（全局坐标系）
    Vec34 _posFeet2BGlobal;                         ///< 足端相对机体位置（全局表示）
    RotMat _B2G_RotMat, _G2B_RotMat;                ///< 机体到全局/全局到机体的旋转矩阵
    Vec12 _q;                                       ///< 当前关节角度

    // ==================== 运动命令变量 ====================
    
    Vec3 _pcd;                                      ///< 期望机体位置（全局坐标系）
    Vec3 _vCmdGlobal, _vCmdBody;                    ///< 速度命令（全局和机体坐标系）
    double _yawCmd, _dYawCmd;                       ///< 期望偏航角和偏航角速度
    double _dYawCmdPast;                            ///< 上一步的偏航角速度（用于滤波）
    Vec3 _wCmdGlobal;                               ///< 角速度命令（全局坐标系）
    
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;   ///< 足端目标位置和速度（全局）
    Vec34 _posFeet2BGoal, _velFeet2BGoal;           ///< 足端目标位置和速度（机体）
    RotMat _Rd;                                     ///< 期望旋转矩阵
    Vec3 _ddPcd, _dWbd;                             ///< 期望加速度和角加速度
    Vec34 _forceFeetGlobal, _forceFeetBody;         ///< 足端力（全局和机体坐标系）
    Vec34 _qGoal, _qdGoal;                          ///< 目标关节角度和角速度
    Vec12 _tau;                                     ///< 关节力矩

    // ==================== 控制参数 ====================
    
    double _gaitHeight;                             ///< 步态抬腿高度 (默认0.08m)
    Vec3 _posError, _velError;                      ///< 位置误差和速度误差
    Mat3 _Kpp, _Kdp, _Kdw;                          ///< 位置、速度、角速度控制增益矩阵
    double _kpw;                                    ///< 角位置控制增益
    Mat3 _KpSwing, _KdSwing;                        ///< 摆动腿控制增益矩阵
    Vec2 _vxLim, _vyLim, _wyawLim;                  ///< 速度限制（前向、侧向、转向）
    
    Vec4 *_phase;                                   ///< 步态相位指针（4条腿的相位）
    VecInt4 *_contact;                              ///< 足端接触状态指针（4条腿的接触状态）

    // ==================== 数据分析工具 ====================
    
    /**
    * @brief 位置误差平均值计算器
    * 用于分析和调试位置控制性能
    * 参数：3维数据，1000点窗口，1倍缩放
    */
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    
    /**
    * @brief 角度误差平均值计算器  
    * 用于分析和调试姿态控制性能
    * 参数：3维数据，1000点窗口，1000倍缩放
    */
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // TROTTING_H