/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/*
* ================================================================
* 文件名：BalanceCtrl.h
* 作用：四足机器人平衡控制器
* 
* 核心功能：
* - 实现机器人的重心位置和姿态控制
* - 通过二次规划(QP)算法分配四条腿的接触力
* - 确保机器人在运动过程中保持稳定的平衡状态
* - 处理足端摩擦约束，防止足端滑动
* 
* 控制原理：
* 根据期望的机体加速度和角加速度，通过求解约束优化问题，
* 计算出四条腿需要施加的最优接触力，使机器人能够准确
* 跟踪期望的运动轨迹并保持稳定。
* ================================================================
*/

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/mathTypes.h"           // 数学类型定义(向量、矩阵等)
#include "thirdParty/quadProgpp/QuadProg++.hh"  // 二次规划求解器
#include "common/unitreeRobot.h"        // 机器人模型定义

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"          // Python绘图接口(调试用)
#endif  // COMPILE_DEBUG

class BalanceCtrl{
public:
    /**
    * @brief 构造函数1 - 手动指定机器人参数
    * @param mass 机器人总质量 (kg)
    * @param Ib 机体惯性张量矩阵 (3x3, 相对于机体质心坐标系)
    * @param S 选择矩阵 (6x6, 用于选择控制的自由度)
    * @param alpha 摩擦锥约束权重参数
    * @param beta 力连续性权重参数
    * 
    * 适用于：自定义机器人参数或测试特殊配置
    */
    BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta);
    
    /**
    * @brief 构造函数2 - 从机器人模型获取参数
    * @param robModel 机器人模型对象指针，包含质量、惯性等物理参数
    * 
    * 适用于：使用标准机器人模型(A1或Go1)的常规情况
    */
    BalanceCtrl(QuadrupedRobot *robModel);

    /**
    * @brief 计算四条腿的接触力
    * 这是平衡控制器的核心方法，通过求解二次规划问题计算最优接触力
    * 
    * @param ddPcd 期望的机体质心加速度 (3x1向量，机体坐标系)
    *              [ddx, ddy, ddz] - x前向、y左向、z上向的加速度
    * @param dWbd 期望的机体角加速度 (3x1向量，机体坐标系)  
    *             [dwx, dwy, dwz] - 绕x、y、z轴的角加速度
    * @param rotM 当前机体到世界坐标系的旋转矩阵 (3x3)
    * @param feetPos2B 四条腿足端相对于机体质心的位置 (3x4矩阵)
    *                  每列代表一条腿: [FR, FL, RR, RL]
    * @param contact 四条腿的接触状态 (4x1整数向量)
    *                1表示接触地面，0表示悬空
    * @return Vec34 四条腿的接触力 (3x4矩阵)
    *               每列为一条腿的xyz向接触力: [FR, FL, RR, RL]
    */
    Vec34 calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact);

#ifdef COMPILE_DEBUG
    /**
    * @brief 设置Python绘图接口(仅调试模式)
    * @param plot Python绘图对象指针，用于实时可视化控制数据
    */
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG

private:
    // ==================== 核心计算方法 ====================
    
    /**
    * @brief 计算系统矩阵A
    * 根据足端位置和接触状态构建力到机体运动的映射关系
    * A * F = [合力, 合力矩]T，其中F为12维接触力向量
    * 
    * @param feetPos2B 足端位置矩阵
    * @param rotM 机体姿态旋转矩阵  
    * @param contact 接触状态向量
    */
    void calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact);
    
    /**
    * @brief 计算期望向量bd
    * 将期望加速度转换为期望的合力和合力矩
    * 
    * @param ddPcd 期望质心加速度
    * @param dWbd 期望角加速度
    * @param rotM 机体姿态矩阵
    */
    void calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM);
    
    /**
    * @brief 计算约束条件
    * 设置二次规划的等式和不等式约束：
    * - 摩擦锥约束：防止足端滑动
    * - 单向力约束：只能施加压力不能施加拉力
    * 
    * @param contact 接触状态，只对接触腿施加约束
    */
    void calConstraints(VecInt4 contact);
    
    /**
    * @brief 求解二次规划问题
    * 最小化目标函数：||A*F - bd||² + α*||摩擦约束||² + β*||F-F_prev||²
    * 得到最优的12维接触力向量
    */
    void solveQP();

    // ==================== 二次规划相关矩阵 ====================
    
    Mat12 _G;           ///< QP目标函数的Hessian矩阵 (12x12)
    Mat12 _W;           ///< 权重矩阵，用于平衡不同目标 (12x12)  
    Mat12 _U;           ///< 单位矩阵，用于正则化 (12x12)
    Mat6 _S;            ///< 自由度选择矩阵 (6x6)
    Mat3 _Ib;           ///< 机体惯性张量 (3x3)
    Vec6 _bd;           ///< 期望的合力和合力矩 (6x1)
    Vec3 _g;            ///< 重力加速度向量 [0, 0, -9.81] (3x1)
    Vec3 _pcb;          ///< 机体质心位置（通常为零点） (3x1)
    Vec12 _F;           ///< 当前求解的接触力 (12x1)
    Vec12 _Fprev;       ///< 上一时刻的接触力，用于平滑 (12x1)
    Vec12 _g0T;         ///< QP目标函数的线性项 (12x1)

    // ==================== 物理参数 ====================
    
    double _mass;       ///< 机器人总质量 (kg)
    double _alpha;      ///< 摩擦约束权重系数
    double _beta;       ///< 力连续性权重系数  
    double _fricRatio;  ///< 摩擦系数，通常取0.4-0.6

    // ==================== Eigen矩阵(用于计算) ====================
    
    Eigen::MatrixXd _CE;                ///< 等式约束矩阵
    Eigen::MatrixXd _CI;                ///< 不等式约束矩阵
    Eigen::VectorXd _ce0;               ///< 等式约束向量
    Eigen::VectorXd _ci0;               ///< 不等式约束向量
    Eigen::Matrix<double, 6 , 12> _A;   ///< 力到运动的映射矩阵 (6x12)
    Eigen::Matrix<double, 5 , 3 > _fricMat;  ///< 摩擦锥矩阵 (5x3)

    // ==================== 二次规划求解器接口 ====================
    
    quadprogpp::Matrix<double> G;       ///< QP求解器的Hessian矩阵
    quadprogpp::Matrix<double> CE;      ///< QP求解器的等式约束矩阵
    quadprogpp::Matrix<double> CI;      ///< QP求解器的不等式约束矩阵
    quadprogpp::Vector<double> g0;      ///< QP求解器的线性项向量
    quadprogpp::Vector<double> ce0;     ///< QP求解器的等式约束向量
    quadprogpp::Vector<double> ci0;     ///< QP求解器的不等式约束向量
    quadprogpp::Vector<double> x;       ///< QP求解器的解向量(接触力)

#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;                  ///< Python绘图对象(调试用)
#endif  // COMPILE_DEBUG
};

#endif  // BALANCECTRL_H

/*
* ================================================================
* 算法核心思想：
* 
* 1. 动力学建模：
*    机体运动方程：M*ddq = S^T * A^T * F + G
*    其中 F 为12维接触力向量 [FR_xyz, FL_xyz, RR_xyz, RL_xyz]
* 
* 2. 二次规划目标：
*    minimize: ||A*F - bd||² + α*||摩擦项||² + β*||F-F_prev||²
*    约束条件：摩擦锥约束 + 单向力约束
* 
* 3. 物理约束：
*    - 摩擦锥：|Fx|, |Fy| ≤ μ*Fz （防止滑动）
*    - 单向力：Fz ≥ 0 （只能压不能拉）
* 
* 4. 控制流程：
*    输入期望运动 → 构建QP问题 → 求解最优力 → 输出接触力
* ================================================================
*/