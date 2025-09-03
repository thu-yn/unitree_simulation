/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <stdio.h>
#include <iostream>
#include "common/mathTypes.h"

/**
 * @file mathTools.h
 * @brief 四足机器人控制系统的数学工具函数集合
 * 
 * 本文件提供了机器人控制系统中常用的数学工具函数，包括：
 * - 基础数学运算（最值、饱和、归一化等）
 * - 控制系统专用函数（零点消除、窗口函数等）
 * - 旋转变换函数（欧拉角、旋转矩阵、四元数转换）
 * - 齐次坐标变换工具
 * - 统计分析工具（均值、协方差更新等）
 */

/************************/
/***** 基础数学函数 *****/
/************************/

/**
 * @brief 模板化最大值函数
 * @tparam T1 第一个参数的类型
 * @tparam T2 第二个参数的类型
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @return T1 返回两个值中的最大值
 */
template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
	return (a > b ? a : b);
}

/**
 * @brief 模板化最小值函数
 * @tparam T1 第一个参数的类型
 * @tparam T2 第二个参数的类型
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @return T1 返回两个值中的最小值
 */
template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
	return (a < b ? a : b);
}

/**
 * @brief 饱和函数（限幅函数）
 * @tparam T 输入值的类型
 * @param a 待限幅的输入值
 * @param limits 限制范围，Vec2类型 [下限, 上限]
 * @return T 限幅后的输出值
 * 
 * 应用场景：
 * - 关节角度限制：防止机械结构损坏
 * - 力矩输出限制：防止电机过载
 * - 速度限制：确保运动安全
 */
template<typename T>
inline T saturation(const T a, Vec2 limits){
    T lowLim, highLim;
    if(limits(0) > limits(1)){
        lowLim = limits(1);
        highLim= limits(0);
    }else{
        lowLim = limits(0);
        highLim= limits(1);
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}

/**
 * @brief 零点偏移消除函数
 * @tparam T0 输入值的类型
 * @tparam T1 阈值的类型
 * @param a 输入值
 * @param limit 零点阈值
 * @return T0 处理后的值
 * 
 * 应用场景：
 * - 遥控器摇杆的死区处理
 * - IMU角速度的零点校正
 * - 关节编码器的微小抖动消除
 */
template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit){
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

/**
 * @brief 反向归一化函数
 * @tparam T0 输入值的类型
 * @tparam T1 输出最小值的类型
 * @tparam T2 输出最大值的类型
 * @param value 待转换的归一化值
 * @param min 目标范围的最小值
 * @param max 目标范围的最大值
 * @param minLim 输入归一化范围的最小值（默认-1）
 * @param maxLim 输入归一化范围的最大值（默认+1）
 * @return T1 转换后的实际值
 * 
 * 将归一化的值（通常在[-1,1]范围）转换回实际物理量
 */
template<typename T0, typename T1, typename T2>
inline T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

/**
 * @brief 窗口函数（梯形过渡函数）
 * @tparam T 数值类型
 * @param x 输入变量（当前位置）
 * @param windowRatio 过渡区间比例（0-0.5之间）
 * @param xRange 输入变量的总范围（默认1.0）
 * @param yRange 输出变量的目标值（默认1.0）
 * @return T 窗口函数的输出值
 * 
 * 生成梯形窗口函数，用于步态规划中的足端轨迹平滑
 */
template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange=1.0, const T yRange=1.0){
    if((x < 0)||(x > xRange)){
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5)){
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
    }

    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }
    else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }
    else{
        return yRange;
    }
}

/************************/
/***** 统计分析函数 *****/
/************************/

/**
 * @brief 递推均值更新函数
 * @tparam T1 均值变量的类型
 * @tparam T2 新测量值的类型
 * @param exp 当前均值估计（引用传递，会被更新）
 * @param newValue 新的测量值
 * @param n 累计测量次数
 * 
 * 使用递推公式更新均值，避免存储所有历史数据
 */
template<typename T1, typename T2>
inline void updateAverage(T1 &exp, T2 newValue, double n){
    if(exp.rows()!=newValue.rows()){
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.001){
        exp = newValue;
    }else{
        exp = exp + (newValue - exp)/n;
    }
}

/**
 * @brief 递推协方差更新函数
 * @tparam T1 协方差矩阵的类型
 * @tparam T2 过去均值的类型
 * @tparam T3 新测量值的类型
 * @param cov 当前协方差估计（引用传递，会被更新）
 * @param expPast 更新前的均值估计
 * @param newValue 新的测量值
 * @param n 累计测量次数
 * 
 * 递推更新协方差矩阵，用于状态估计中的不确定性量化
 */
template<typename T1, typename T2, typename T3>
inline void updateCovariance(T1 &cov, T2 expPast, T3 newValue, double n){
    if( (cov.rows()!=cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows()!=newValue.rows())){
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.1){
        cov.setZero();
    }else{
        cov = cov*(n-1)/n + (newValue-expPast)*(newValue-expPast).transpose()*(n-1)/(n*n);
    }
}

/**
 * @brief 同时更新均值和协方差的函数
 * @tparam T1 协方差矩阵的类型
 * @tparam T2 均值变量的类型
 * @tparam T3 新测量值的类型
 * @param cov 协方差矩阵（引用传递，会被更新）
 * @param exp 均值估计（引用传递，会被更新）
 * @param newValue 新的测量值
 * @param n 累计测量次数
 * 
 * 注意：协方差必须在均值更新之前计算！
 */
template<typename T1, typename T2, typename T3>
inline void updateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n){
    // The order matters!!! covariance first!!!
    updateCovariance(cov, exp, newValue, n);
    updateAverage(exp, newValue, n);
}

/************************/
/***** 旋转变换函数 *****/
/************************/

/**
 * @brief 绕X轴旋转矩阵生成函数
 * @param theta 旋转角度（弧度）
 * @return RotMat 3x3旋转矩阵
 * 
 * 生成绕X轴旋转theta角度的旋转矩阵：
 * [1   0      0   ]
 * [0  cos  -sin  ]
 * [0  sin   cos  ]
 */
inline RotMat rotx(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

/**
 * @brief 绕Y轴旋转矩阵生成函数
 * @param theta 旋转角度（弧度）
 * @return RotMat 3x3旋转矩阵
 * 
 * 生成绕Y轴旋转theta角度的旋转矩阵：
 * [cos   0   sin ]
 * [ 0    1    0  ]
 * [-sin  0   cos ]
 */
inline RotMat roty(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

/**
 * @brief 绕Z轴旋转矩阵生成函数
 * @param theta 旋转角度（弧度）
 * @return RotMat 3x3旋转矩阵
 * 
 * 生成绕Z轴旋转theta角度的旋转矩阵：
 * [cos  -sin  0 ]
 * [sin   cos  0 ]
 * [ 0     0   1 ]
 */
inline RotMat rotz(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

/**
 * @brief 二维反对称矩阵生成函数
 * @param w 标量值
 * @return Mat2 2x2反对称矩阵
 * 
 * 生成二维反对称矩阵：
 * [ 0  -w]
 * [ w   0]
 * 
 * 应用场景：
 * - 二维空间中的旋转变换
 * - 角速度的矩阵表示
 */
inline Mat2 skew(const double& w){
    Mat2 mat; mat.setZero();
    mat(0, 1) = -w;
    mat(1, 0) =  w;
    return mat;
}

/**
 * @brief 三维反对称矩阵生成函数（向量叉积的矩阵形式）
 * @param v 三维向量
 * @return Mat3 3x3反对称矩阵
 * 
 * 将三维向量转换为反对称矩阵，使得：
 * skew(v) * u = v × u （向量叉积）
 * 
 * 生成的矩阵形式：
 * [  0   -vz   vy ]
 * [ vz    0   -vx ]
 * [-vy   vx    0  ]
 * 
 * 应用场景：
 * - 角速度矩阵表示
 * - 刚体运动学计算
 * - 李代数到李群的映射
 * - 机器人动力学建模
 */
inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

/**
 * @brief 欧拉角转旋转矩阵函数
 * @param row Roll角（绕X轴旋转，弧度）
 * @param pitch Pitch角（绕Y轴旋转，弧度）
 * @param yaw Yaw角（绕Z轴旋转，弧度）
 * @return RotMat 3x3旋转矩阵
 * 
 * 按照ZYX顺序（外旋）进行复合旋转：
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * 
 * 欧拉角定义：
 * - Roll：机体绕X轴（前进方向）的转动，正值表示右倾
 * - Pitch：机体绕Y轴（右侧方向）的转动，正值表示抬头
 * - Yaw：机体绕Z轴（垂直向上）的转动，正值表示左转
 * 
 * 应用场景：
 * - 机器人姿态控制
 * - IMU数据处理
 * - 坐标系变换
 * - 运动轨迹规划
 */
inline RotMat rpyToRotMat(const double& row, const double& pitch, const double& yaw) {
    RotMat m = rotz(yaw) * roty(pitch) * rotx(row);
    return m;
}

/**
 * @brief 旋转矩阵转欧拉角函数
 * @param R 3x3旋转矩阵
 * @return Vec3 欧拉角向量 [roll, pitch, yaw]
 * 
 * 从旋转矩阵中提取ZYX欧拉角：
 * - roll = atan2(R(2,1), R(2,2))
 * - pitch = asin(-R(2,0))
 * - yaw = atan2(R(1,0), R(0,0))
 * 
 * 注意事项：
 * - 该方法在pitch接近±π/2时可能出现奇异性（万向锁）
 * - 欧拉角表示不唯一，存在多解情况
 * 
 * 应用场景：
 * - 姿态反馈控制
 * - 传感器数据解析
 * - 姿态可视化显示
 */
inline Vec3 rotMatToRPY(const Mat3& R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}

/**
 * @brief 四元数转旋转矩阵函数
 * @param q 四元数 [w, x, y, z] (w为实部，x,y,z为虚部)
 * @return RotMat 3x3旋转矩阵
 * 
 * 将单位四元数转换为对应的旋转矩阵。
 * 四元数格式：q = [e0, e1, e2, e3] = [w, x, y, z]
 * 其中：e0=w为实部，e1=x, e2=y, e3=z为虚部
 * 
 * 转换公式基于四元数的旋转矩阵标准形式：
 * R = I + 2*sin(θ/2)*[v]× + 2*sin²(θ/2)*[v]×²
 * 
 * 优势：
 * - 避免万向锁问题
 * - 插值平滑
 * - 计算效率高
 * - 数值稳定性好
 * 
 * 应用场景：
 * - 姿态控制系统
 * - 3D图形渲染
 * - 机器人运动学
 * - 惯导系统
 */
inline RotMat quatToRotMat(const Quat& q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

/**
 * @brief 旋转矩阵转旋转向量（指数映射）函数
 * @param rm 3x3旋转矩阵
 * @return Vec3 旋转向量（轴角表示）
 * 
 * 将旋转矩阵转换为旋转向量（也称为轴角表示或指数坐标）。
 * 旋转向量的方向表示旋转轴，模长表示旋转角度。
 * 
 * 算法步骤：
 * 1. 通过矩阵的迹计算旋转角度：angle = acos((trace(R)-1)/2)
 * 2. 根据角度大小选择不同的计算策略：
 *    - 小角度：近似为零向量
 *    - 大角度(π)：特殊处理避免奇异性
 *    - 一般角度：使用标准公式
 * 
 * 特殊情况处理：
 * - angle≈0：返回零向量
 * - angle≈π：使用特殊公式避免奇异性
 * 
 * 应用场景：
 * - 姿态控制中的误差计算
 * - 优化问题中的旋转参数化
 * - 李群李代数理论应用
 * - 姿态插值和规划
 */
inline Vec3 rotMatToExp(const RotMat& rm){
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f){
        cosValue = 1.0f;
    }else if(cosValue < -1.0f){
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Vec3 exp;
    if (fabs(angle) < 1e-5){
        exp=Vec3(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5){
        exp = angle * Vec3(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else{
        exp=angle/(2.0f*sin(angle))*Vec3(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }
    return exp;
}

/************************/
/***** 齐次变换函数 *****/
/************************/

/**
 * @brief 构造齐次变换矩阵（位置+旋转矩阵）
 * @param p 三维位置向量
 * @param m 3x3旋转矩阵
 * @return HomoMat 4x4齐次变换矩阵
 * 
 * 构造4x4齐次变换矩阵：
 * [R  p]   其中R为旋转矩阵，p为位置向量
 * [0  1]
 * 
 * 齐次变换矩阵同时表示旋转和平移，是刚体变换的标准表示方法。
 * 
 * 应用场景：
 * - 正向运动学计算
 * - 坐标系变换
 * - 机器人工具坐标系定义
 * - 多刚体系统建模
 */
inline HomoMat homoMatrix(Vec3 p, RotMat m){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = m;
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

/**
 * @brief 构造齐次变换矩阵（位置+四元数）
 * @param p 三维位置向量
 * @param q 四元数姿态
 * @return HomoMat 4x4齐次变换矩阵
 * 
 * 使用四元数表示姿态的齐次变换矩阵构造函数。
 * 内部先将四元数转换为旋转矩阵，然后构造齐次变换矩阵。
 * 
 * 应用场景：
 * - 基于四元数的位姿表示
 * - 传感器数据处理（IMU通常输出四元数）
 * - 姿态控制系统
 * - 3D可视化和动画
 */
inline HomoMat homoMatrix(Vec3 p, Quat q){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = quatToRotMat(q);
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

/**
 * @brief 齐次变换矩阵求逆函数
 * @param homoM 输入的4x4齐次变换矩阵
 * @return HomoMat 逆齐次变换矩阵
 * 
 * 高效计算齐次变换矩阵的逆，利用旋转矩阵的正交性：
 * 
 * 对于变换矩阵 T = [R p; 0 1]
 * 其逆矩阵为 T⁻¹ = [Rᵀ -Rᵀp; 0 1]
 * 
 * 计算步骤：
 * 1. 旋转部分：R⁻¹ = Rᵀ（旋转矩阵的逆等于其转置）
 * 2. 平移部分：p_inv = -Rᵀ * p
 * 
 * 优势：
 * - 计算效率高，避免4x4矩阵的通用求逆
 * - 数值稳定性好
 * - 保持齐次变换矩阵的结构特性
 * 
 * 应用场景：
 * - 坐标系的逆变换
 * - 运动学逆解
 * - 传感器标定
 * - 相对位姿计算
 */
inline HomoMat homoMatrixInverse(HomoMat homoM){
    HomoMat homoInv;
    homoInv.setZero();
    homoInv.topLeftCorner(3, 3) = homoM.topLeftCorner(3, 3).transpose();
    homoInv.topRightCorner(3, 1) = -homoM.topLeftCorner(3, 3).transpose() * homoM.topRightCorner(3, 1);
    homoInv(3, 3) = 1;
    return homoInv;
}

/**
 * @brief 三维向量转齐次向量
 * @param v3 三维向量
 * @return Vec4 四维齐次向量
 * 
 * 在三维向量末尾添加1，转换为齐次坐标表示：
 * [x, y, z] → [x, y, z, 1]
 * 
 * 齐次坐标的优势：
 * - 统一表示平移和旋转变换
 * - 便于矩阵运算
 * - 支持透视投影
 * 
 * 应用场景：
 * - 齐次变换矩阵运算
 * - 计算机图形学
 * - 机器人运动学
 * - 坐标变换链式计算
 */
//  add 1 at the end of Vec3
inline Vec4 homoVec(Vec3 v3){
    Vec4 v4;
    v4.block(0, 0, 3, 1) = v3;
    v4(3) = 1;
    return v4;
}

/**
 * @brief 齐次向量转三维向量
 * @param v4 四维齐次向量
 * @return Vec3 三维向量
 * 
 * 移除齐次向量的最后一个元素（通常为1），恢复三维向量：
 * [x, y, z, w] → [x, y, z]
 * 
 * 注意：
 * - 通常假设w=1，如果w≠1需要先进行归一化：[x/w, y/w, z/w]
 * - 本函数直接取前三个元素，不进行归一化检查
 * 
 * 应用场景：
 * - 齐次变换后的坐标提取
 * - 计算机图形学中的投影计算
 * - 机器人学中的位置提取
 */
//  remove 1 at the end of Vec4
inline Vec3 noHomoVec(Vec4 v4){
    Vec3 v3;
    v3 = v4.block(0, 0, 3, 1);
    return v3;
}

/************************/
/***** 统计分析类 *****/
/************************/

/**
 * @brief 统计分析类：用于实时计算均值和协方差
 * 
 * 这是一个完整的统计分析工具类，提供：
 * - 实时均值和协方差计算
 * - 可配置的显示周期和预热期
 * - 数值放大显示功能
 * - 统计信息的格式化输出
 * 
 * 设计特点：
 * - 递推算法：内存占用小，适合实时系统
 * - 数值稳定：使用Welford算法变种
 * - 可配置：支持多种显示和统计选项
 * - 调试友好：提供清晰的输出格式
 * 
 * 主要用途：
 * - 控制系统性能监控
 * - 传感器特性分析  
 * - 算法收敛性验证
 * - 系统调试和优化
 */
// Calculate average value and covariance
class AvgCov{
public:
    /**
     * @brief 构造函数
     * @param size 数据向量的维度
     * @param name 统计变量的名称（用于输出显示）
     * @param avgOnly 是否只计算均值（默认false，同时计算协方差）
     * @param showPeriod 显示统计结果的周期（每多少次测量显示一次，默认1000）
     * @param waitCount 开始统计前的等待测量次数（数据预热期，默认5000）
     * @param zoomFactor 显示时的放大因子（便于观察小数值，默认10000）
     * 
     * 参数说明：
     * - size: 被统计向量的维度，必须与后续输入数据维度一致
     * - name: 变量名称，用于输出标识，便于多个统计对象的区分
     * - avgOnly: 仅计算均值可提高计算效率，适合高频数据
     * - showPeriod: 输出周期，避免过于频繁的屏幕输出
     * - waitCount: 预热期长度，让系统稳定后再开始统计
     * - zoomFactor: 放大系数，便于观察微小的数值变化
     */
    AvgCov(unsigned int size, std::string name, bool avgOnly=false, unsigned int showPeriod=1000, unsigned int waitCount=5000, double zoomFactor=10000)
            :_size(size), _showPeriod(showPeriod), _waitCount(waitCount), _zoomFactor(zoomFactor), _valueName(name), _avgOnly(avgOnly) {
        _exp.resize(size);
        _cov.resize(size, size);
        _defaultWeight.resize(size, size);
        _defaultWeight.setIdentity();
        _measureCount = 0;
    }
    
    /**
     * @brief 添加新的测量数据
     * @param newValue 新的测量值向量
     * 
     * 核心统计函数，每次新数据到达时调用：
     * 1. 增加测量计数器
     * 2. 检查是否过了预热期
     * 3. 更新统计量（均值和协方差）
     * 4. 定期输出统计结果
     * 
     * 工作流程：
     * - 预热期内：只计数，不统计，让系统达到稳态
     * - 正式统计期：使用递推算法更新统计量
     * - 输出期：按设定周期显示当前统计结果
     * 
     * 输出格式：
     * - 显示有效统计次数
     * - 显示放大后的均值（横向显示）
     * - 显示放大后的协方差矩阵（如果启用）
     */
    void measure(VecX newValue){
        ++_measureCount;

        if(_measureCount > _waitCount){
            updateAvgCov(_cov, _exp, newValue, _measureCount-_waitCount);
            if(_measureCount % _showPeriod == 0){
                std::cout << "******" << _valueName << " measured count: " << _measureCount-_waitCount << "******" << std::endl;
                std::cout << _zoomFactor << " Times Average of " << _valueName << std::endl << (_zoomFactor*_exp).transpose() << std::endl;
                if(!_avgOnly){
                    std::cout << _zoomFactor << " Times Covariance of " << _valueName << std::endl << _zoomFactor*_cov << std::endl;
                }
            }
        }
    }
    
private:
    VecX _exp;                    // 当前均值估计向量
    MatX _cov;                    // 当前协方差矩阵估计
    MatX _defaultWeight;          // 默认权重矩阵（单位矩阵）
    bool _avgOnly;                // 是否仅计算均值标志
    unsigned int _size;           // 数据向量维度
    unsigned int _measureCount;   // 累计测量次数计数器
    unsigned int _showPeriod;     // 显示周期设置
    unsigned int _waitCount;      // 预热期等待次数设置
    double _zoomFactor;           // 显示放大因子
    std::string _valueName;       // 统计变量名称
};

#endif  // MATHTOOLS_H