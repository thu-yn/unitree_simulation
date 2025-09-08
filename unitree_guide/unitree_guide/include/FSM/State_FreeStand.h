/**********************************************************************
 * 文件作用：定义自由站立状态类，负责机器人在自由站立模式下的姿态控制
 * 该状态允许机器人在保持站立的同时，通过遥控器输入调整机器人的横滚(Roll)、
 * 俯仰(Pitch)、偏航(Yaw)角度以及身体高度变化，实现灵活的姿态调整功能。
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"

/**
 * @brief 自由站立状态类
 * @details 继承自FSMState基类，实现机器人的自由站立功能
 *          在此状态下，机器人可以根据用户输入调整身体姿态角度和高度，
 *          同时保持四足接触地面的稳定站立状态
 */
class State_FreeStand : public FSMState{
public:
    /**
     * @brief 构造函数
     * @param ctrlComp 控制组件指针，包含机器人控制所需的各种组件和接口
     */
    State_FreeStand(CtrlComponents *ctrlComp);
    
    /**
     * @brief 析构函数
     * @details 使用默认析构函数，无需特殊清理操作
     */
    ~State_FreeStand(){}
    
    /**
     * @brief 状态进入函数
     * @details 当FSM切换到自由站立状态时调用，负责：
     *          - 设置电机参数和增益
     *          - 初始化当前姿态作为基准姿态
     *          - 设置所有腿为站立模式
     *          - 清零用户命令面板
     */
    void enter();
    
    /**
     * @brief 状态运行函数
     * @details 状态的主要执行逻辑，在状态激活期间持续调用
     *          读取用户输入值，计算目标姿态，并生成相应的关节角度指令
     */
    void run();
    
    /**
     * @brief 状态退出函数
     * @details 当FSM准备切换到其他状态时调用，负责清零用户命令面板
     */
    void exit();
    
    /**
     * @brief 状态切换检查函数
     * @details 检查用户命令，决定是否需要切换到其他状态
     * @return FSMStateName 返回下一个应该切换到的状态名称
     *         - L2_A: 切换到FIXEDSTAND(固定站立)
     *         - L2_B: 切换到PASSIVE(被动模式)
     *         - START: 切换到TROTTING(小跑步态)
     *         - 其他: 保持FREESTAND状态
     */
    FSMStateName checkChange();

private:
    /**
     * @brief 初始机体坐标系X轴方向向量
     * @details 3维向量，存储进入状态时机器人的X轴方向向量
     *          用作姿态控制的参考基准，通过robotModel->getX()获得
     */
    Vec3 _initVecOX;
    
    /**
     * @brief 初始足端位置矩阵
     * @details 3x4矩阵，每一列代表一条腿的足端位置(3D坐标)
     *          存储进入状态时四个足端相对于机体的位置向量
     *          通过robotModel->getVecXP()获得
     */
    Vec34 _initVecXP;
    
    // 各轴角度和高度的限制参数
    /**
     * @brief 横滚角(Roll)的最大值
     * @details 机器人绕机体X轴旋转的最大允许角度(弧度)
     *          默认设为20°(20 * M_PI / 180)
     */
    float _rowMax;
    
    /**
     * @brief 横滚角(Roll)的最小值  
     * @details 机器人绕机体X轴旋转的最小允许角度(弧度)
     *          默认设为-20°(-_rowMax)
     */
    float _rowMin;
    
    /**
     * @brief 俯仰角(Pitch)的最大值
     * @details 机器人绕机体Y轴旋转的最大允许角度(弧度)
     *          默认设为15°(15 * M_PI / 180)
     */
    float _pitchMax;
    
    /**
     * @brief 俯仰角(Pitch)的最小值
     * @details 机器人绕机体Y轴旋转的最小允许角度(弧度)
     *          默认设为-15°(-_pitchMax)
     */
    float _pitchMin;
    
    /**
     * @brief 偏航角(Yaw)的最大值
     * @details 机器人绕机体Z轴旋转的最大允许角度(弧度)
     *          默认设为20°(20 * M_PI / 180)
     */
    float _yawMax;
    
    /**
     * @brief 偏航角(Yaw)的最小值
     * @details 机器人绕机体Z轴旋转的最小允许角度(弧度)
     *          默认设为-20°(-_yawMax)
     */
    float _yawMin;
    
    /**
     * @brief 机器人身体高度变化的最大值
     * @details 相对于初始高度的最大上升量(米)
     *          默认设为0.04m，即最大可上升4cm
     */
    float _heightMax;
    
    /**
     * @brief 机器人身体高度变化的最小值
     * @details 相对于初始高度的最大下降量(米)
     *          默认设为-0.04m，即最大可下降4cm
     */
    float _heightMin;

    /**
     * @brief 计算目标足端位置矩阵
     * @param row 目标横滚角(弧度)，由遥控器lx输入映射得到
     * @param pitch 目标俯仰角(弧度)，由遥控器ly输入映射得到
     * @param yaw 目标偏航角(弧度)，由遥控器rx输入映射得到(取负值)
     * @param height 目标身体高度变化(米)，由遥控器ry输入映射得到
     * @return Vec34 返回3x4的目标足端位置矩阵，每列代表一个足端的目标位置
     * @details 根据输入的姿态参数，通过齐次变换矩阵计算各足端的新位置，
     *          使机器人能够达到指定的姿态和高度
     */
    Vec34 _calcOP(float row, float pitch, float yaw, float height);
    
    /**
     * @brief 计算并设置关节角度指令
     * @param vecOP 目标足端位置矩阵(3x4)
     * @details 根据目标足端位置，通过逆运动学计算对应的12个关节角度，
     *          并将计算结果设置到底层控制指令中，驱动机器人执行姿态调整
     */
    void _calcCmd(Vec34 vecOP);
};

#endif  // FREESTAND_H