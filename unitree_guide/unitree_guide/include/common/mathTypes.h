/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <eigen3/Eigen/Dense>

/**
 * @file mathTypes.h
 * @brief 四足机器人控制系统的数学类型定义
 * 
 * 本文件定义了整个机器人控制系统中使用的数学类型，主要基于Eigen库。
 * 这些类型定义统一了项目中的数学表示，确保代码的一致性和可维护性。
 * 
 * 设计原则：
 * - 使用Eigen库作为底层数学库，提供高效的线性代数运算
 * - 通过using别名简化复杂的Eigen模板类型声明
 * - 为机器人学中常用的数据结构提供直观的类型名
 * - 区分固定大小和动态大小的矩阵向量类型
 * 
 * 主要用途：
 * - 机器人姿态表示（旋转矩阵、四元数）
 * - 关节空间和操作空间的状态向量
 * - 控制系统中的增益矩阵和雅可比矩阵
 * - 轨迹规划中的路径点和速度加速度
 */

/************************/
/******** Vector ********/
/************************/

/**
 * @brief 二维向量类型
 * 
 * 用途示例：
 * - 平面坐标表示 (x, y)
 * - 二维控制参数的上下限
 * - 图像处理中的像素坐标
 */
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

/**
 * @brief 三维向量类型
 * 
 * 这是机器人学中最常用的向量类型，用途包括：
 * - 三维空间位置坐标 (x, y, z)
 * - 角度表示 (roll, pitch, yaw)
 * - 力和力矩向量
 * - 线速度和角速度
 * - 加速度向量（线性和角加速度）
 * 
 * 使用示例：
 * Vec3 position{0.0, 0.0, 0.5}; // 机器人重心位置
 * Vec3 rpy{0.1, 0.0, 0.2};      // 机身姿态角
 */
using Vec3 = typename Eigen::Matrix<double, 3, 1>;

/**
 * @brief 四维向量类型
 * 
 * 在四足机器人中的主要用途：
 * - 四条腿的状态表示（每条腿一个标量值）
 * - 四元数表示（虽然也有专门的Quat类型）
 * - 四个关节的控制参数
 * 
 * 使用示例：
 * Vec4 legPhases{0.0, 0.5, 0.5, 0.0}; // 四条腿的步态相位
 * Vec4 footForces{100, 120, 110, 115}; // 四个足端的垂直力
 */
using Vec4 = typename Eigen::Matrix<double, 4, 1>;

/**
 * @brief 六维向量类型
 * 
 * 在机器人学中通常表示：
 * - 机器人的完整空间速度（3D线速度 + 3D角速度）
 * - 广义力向量（3D力 + 3D力矩）
 * - 机器人基座的完整状态（位置 + 姿态，当使用欧拉角时）
 * 
 * 向量结构通常为：[vx, vy, vz, wx, wy, wz]
 * 其中前三项为线性分量，后三项为角度分量
 */
using Vec6 = typename Eigen::Matrix<double, 6, 1>;

/**
 * @brief 四元数类型
 * 
 * 四元数是表示三维旋转的一种数学方法，具有以下优势：
 * - 避免万向锁问题
 * - 插值平滑
 * - 计算效率高
 * 
 * 四元数格式：[w, x, y, z] 或 [x, y, z, w]（取决于约定）
 * 在Unitree系统中，通常使用[w, x, y, z]格式
 * 
 * 使用场景：
 * - 机器人基座姿态表示
 * - 关节旋转表示
 * - 目标姿态指定
 */
using Quat = typename Eigen::Matrix<double, 4, 1>;

/**
 * @brief 四维整型向量
 * 
 * 主要用于表示离散状态或标志位：
 * - 足端接触状态（0=离地，1=接触）
 * - 步态状态标识
 * - 控制模式选择
 * - 错误状态标志
 * 
 * 使用示例：
 * VecInt4 contact{1, 1, 1, 1}; // 四足都接触地面
 * VecInt4 gaitMode{1, 0, 0, 1}; // 对角线腿同步的小跑步态
 */
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

/**
 * @brief 十二维向量类型
 * 
 * 四足机器人的关节空间表示，每条腿3个关节：
 * - 关节角度向量：[hip_aa, hip_fe, knee] × 4 legs
 * - 关节速度向量
 * - 关节力矩向量
 * - 足端位置向量：[x, y, z] × 4 legs
 * 
 * 腿部顺序通常为：FL(前左), FR(前右), HL(后左), HR(后右)
 * 
 * 使用示例：
 * Vec12 jointAngles;    // 12个关节的角度
 * Vec12 jointTorques;   // 12个关节的力矩控制量
 */
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

/**
 * @brief 十八维向量类型
 * 
 * 表示机器人的完整状态向量，包括：
 * - 基座状态：位置(3) + 姿态(3) + 线速度(3) + 角速度(3) = 12维
 * - 关节状态：关节角度(12) + 关节速度(12) = 24维
 * - 或其他18维的复合状态表示
 * 
 * 这种高维向量通常用于：
 * - 状态估计器的状态向量
 * - 运动规划的路径点
 * - 全身动力学建模
 */
using Vec18 = typename Eigen::Matrix<double, 18, 1>;

/**
 * @brief 动态长度向量类型
 * 
 * 当向量维度在编译时不确定时使用，例如：
 * - 优化问题的变量向量
 * - 传感器数据的缓存
 * - 可配置参数的存储
 * - 不同机器人型号的适配
 * 
 * 注意：动态大小的矩阵运算效率通常低于固定大小
 */
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/

/**
 * @brief 旋转矩阵类型
 * 
 * 3×3旋转矩阵是表示三维空间旋转的标准方法：
 * - 正交矩阵，行列式为1
 * - 描述坐标系之间的旋转关系
 * - 可以与向量相乘实现坐标变换
 * 
 * 使用场景：
 * - 机器人基座姿态表示
 * - 坐标系转换（世界系↔机体系↔腿部坐标系）
 * - 传感器数据的坐标变换
 * 
 * 与四元数的关系：可以相互转换，各有优缺点
 */
using RotMat = typename Eigen::Matrix<double, 3, 3>;

/**
 * @brief 齐次变换矩阵类型
 * 
 * 4×4齐次变换矩阵同时表示旋转和平移：
 * [R t]  其中R是3×3旋转矩阵，t是3×1平移向量
 * [0 1]
 * 
 * 用途：
 * - 完整的刚体变换表示
 * - 正向运动学计算
 * - 坐标系链式变换
 * - 机器人工具坐标系定义
 */
using HomoMat = typename Eigen::Matrix<double, 4, 4>;

/**
 * @brief 二阶方阵类型
 * 
 * 常用于：
 * - 二维系统的状态转移矩阵
 * - 简单控制系统的增益矩阵
 * - 平面运动的变换矩阵
 */
using Mat2 = typename Eigen::Matrix<double, 2, 2>;

/**
 * @brief 三阶方阵类型
 * 
 * 最常用的矩阵类型之一，用途包括：
 * - 旋转矩阵（RotMat是它的别名）
 * - 惯性张量矩阵
 * - 三维控制系统的增益矩阵
 * - 协方差矩阵
 */
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

/**
 * @brief 三阶单位矩阵宏定义
 * 
 * 预定义的3×3单位矩阵，避免重复创建：
 * [1 0 0]
 * [0 1 0]
 * [0 0 1]
 * 
 * 使用场景：
 * - 旋转矩阵的初始化
 * - 坐标变换的恒等变换
 * - 控制系统的初始增益矩阵
 */
#define I3 Eigen::MatrixXd::Identity(3, 3)

/**
 * @brief 3×4矩阵类型（四个三维向量的集合）
 * 
 * 专门为四足机器人设计的数据结构：
 * - 每一列代表一条腿的三维信息
 * - 例如四个足端的位置坐标
 * - 四个腿部的力向量
 * - 四个关节组的参考位置
 * 
 * 矩阵结构：
 * [FL_x FR_x HL_x HR_x]  // x坐标
 * [FL_y FR_y HL_y HR_y]  // y坐标
 * [FL_z FR_z HL_z HR_z]  // z坐标
 */
using Vec34 = typename Eigen::Matrix<double, 3, 4>;

/**
 * @brief 六阶方阵类型
 * 
 * 用于六自由度系统的表示：
 * - 空间机器人的雅可比矩阵
 * - 6D控制系统的增益矩阵
 * - 广义坐标的质量矩阵
 * - 状态估计的协方差矩阵
 */
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

/**
 * @brief 十二阶方阵类型
 * 
 * 四足机器人关节空间的系统矩阵：
 * - 关节空间的质量矩阵
 * - 关节控制的增益矩阵
 * - 雅可比矩阵的相关运算
 * - 全腿动力学的系统矩阵
 */
using Mat12 = typename Eigen::Matrix<double, 12, 12>;

/**
 * @brief 十二阶单位矩阵宏定义
 * 
 * 预定义的12×12单位矩阵，用于：
 * - 关节控制系统初始化
 * - 矩阵运算的恒等变换
 * - 对角占主导的矩阵初始化
 */
#define I12 Eigen::MatrixXd::Identity(12, 12)

/**
 * @brief 十八阶单位矩阵宏定义
 * 
 * 预定义的18×18单位矩阵，用于：
 * - 全身动力学系统
 * - 扩展卡尔曼滤波的协方差矩阵
 * - 大规模控制系统的初始化
 */
#define I18 Eigen::MatrixXd::Identity(18, 18)

/**
 * @brief 动态大小矩阵类型
 * 
 * 当矩阵尺寸在运行时确定时使用：
 * - 不同配置的机器人适配
 * - 优化问题的海塞矩阵
 * - 可变长度的数据处理
 * - 算法的通用实现
 */
using MatX = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/************************/
/****** Functions *******/
/************************/

/**
 * @brief 将12维向量重新组织为3×4矩阵
 * @param vec12 输入的12维向量
 * @return Vec34 输出的3×4矩阵，每列代表一条腿的3维信息
 * 
 * 转换规律：
 * vec12 = [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, HL_x, HL_y, HL_z, HR_x, HR_y, HR_z]
 * 转换为：
 * vec34 = [FL_x FR_x HL_x HR_x]
 *         [FL_y FR_y HL_y HR_y]
 *         [FL_z FR_z HL_z HR_z]
 * 
 * 这种转换便于按腿进行数据处理和可视化
 */
inline Vec34 vec12ToVec34(Vec12 vec12){
    Vec34 vec34;
    for(int i(0); i < 4; ++i){
        vec34.col(i) = vec12.segment(3*i, 3);  // 提取第i条腿的xyz三维数据
    }
    return vec34;
}

/**
 * @brief 将3×4矩阵展开为12维向量
 * @param vec34 输入的3×4矩阵，每列代表一条腿的3维信息
 * @return Vec12 输出的12维向量
 * 
 * 转换规律（与vec12ToVec34相反）：
 * vec34 = [FL_x FR_x HL_x HR_x]
 *         [FL_y FR_y HL_y HR_y]
 *         [FL_z FR_z HL_z HR_z]
 * 转换为：
 * vec12 = [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, HL_x, HL_y, HL_z, HR_x, HR_y, HR_z]
 * 
 * 这种转换便于进行向量运算和存储
 */
inline Vec12 vec34ToVec12(Vec34 vec34){
    Vec12 vec12;
    for(int i(0); i < 4; ++i){
        vec12.segment(3*i, 3) = vec34.col(i);  // 将第i条腿的数据存入向量对应位置
    }
    return vec12;
}

#endif  // MATHTYPES_H