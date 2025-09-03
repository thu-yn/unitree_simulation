/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/*
* ================================================================
* 文件名：Estimator.h
* 作用：四足机器人状态估计器
* 
* 核心功能：
* - 融合多传感器数据估计机器人完整状态信息
* - 通过扩展卡尔曼滤波(EKF)算法实现状态估计和预测
* - 结合IMU、关节编码器、足端接触信息进行数据融合
* - 提供机器人位置、速度、足端状态的精确估计
* - 支持ROS导航栈集成，发布里程计信息
* 
* 估计原理：
* 使用线性系统状态空间模型，通过卡尔曼滤波算法融合：
* - IMU提供的姿态和加速度信息
* - 关节编码器提供的足端运动学信息  
* - 步态信息提供的接触约束
* - 先验运动学模型提供的物理约束
* 
* 状态向量维度：18维
* [位置(3) + 速度(3) + 四个足端位置(12)] = 18维状态
* ================================================================
*/

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <vector>
#include "common/unitreeRobot.h"        // 机器人模型
#include "common/LowPassFilter.h"       // 低通滤波器
#include "Gait/WaveGenerator.h"         // 步态生成器
#include "message/LowlevelState.h"      // 底层状态消息
#include "string"

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"          // Python绘图接口(调试用)
#endif  // COMPILE_DEBUG

#ifdef COMPILE_WITH_MOVE_BASE
    #include <ros/ros.h>                // ROS核心库
    #include <ros/time.h>               // ROS时间库
    #include <geometry_msgs/TransformStamped.h>  // TF变换消息
    #include <tf/transform_broadcaster.h>        // TF广播器
    #include <nav_msgs/Odometry.h>               // 里程计消息
    #include <geometry_msgs/Twist.h>             // 速度消息
    #include <boost/array.hpp>                   // Boost数组库
#endif  // COMPILE_WITH_MOVE_BASE

class Estimator{
public:
    /**
    * @brief 构造函数1 - 标准初始化
    * 使用默认的过程噪声协方差参数创建状态估计器
    * 
    * @param robotModel 机器人模型指针，提供运动学参数
    * @param lowState 底层状态指针，提供传感器原始数据
    * @param contact 足端接触状态指针，提供接触约束信息
    * @param phase 步态相位指针，提供步态时序信息
    * @param dt 控制周期(秒)，通常为0.002s(500Hz)
    */
    Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, VecInt4 *contact, Vec4 *phase, double dt);
    
    /**
    * @brief 构造函数2 - 自定义噪声参数初始化
    * 允许用户自定义卡尔曼滤波器的过程噪声协方差参数
    * 
    * @param robotModel 机器人模型指针
    * @param lowState 底层状态指针  
    * @param contact 足端接触状态指针
    * @param phase 步态相位指针
    * @param dt 控制周期
    * @param Qdig 18维对角噪声协方差向量，用于调节滤波器性能
    * @param testName 测试名称字符串，用于调试和数据记录
    */
    Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, VecInt4 *contact, Vec4 *phase, double dt, Vec18 Qdig, std::string testName);
    
    /**
    * @brief 析构函数
    * 清理动态分配的资源，包括低通滤波器对象
    */
    ~Estimator();
    
    /**
    * @brief 获取机器人当前位置估计
    * @return Vec3 机器人质心在全局坐标系中的位置 [x, y, z]
    */
    Vec3  getPosition();
    
    /**
    * @brief 获取机器人当前速度估计
    * @return Vec3 机器人质心在全局坐标系中的速度 [vx, vy, vz]
    */
    Vec3  getVelocity();
    
    /**
    * @brief 获取指定足端的位置估计
    * @param i 足端索引 (0:FR, 1:FL, 2:RR, 3:RL)
    * @return Vec3 足端在全局坐标系中的位置
    */
    Vec3  getFootPos(int i);
    
    /**
    * @brief 获取所有足端的位置估计
    * @return Vec34 4个足端在全局坐标系中的位置矩阵
    */
    Vec34 getFeetPos();
    
    /**
    * @brief 获取所有足端的速度估计
    * @return Vec34 4个足端在全局坐标系中的速度矩阵
    */
    Vec34 getFeetVel();
    
    /**
    * @brief 获取足端相对于机体的全局位置
    * @return Vec34 4个足端相对于机体质心的位置向量(全局坐标系)
    */
    Vec34 getPosFeet2BGlobal();
    
    /**
    * @brief 执行状态估计主循环
    * 这是状态估计器的核心方法，每个控制周期调用一次
    * 执行卡尔曼滤波的预测和更新步骤
    */
    void run();

#ifdef COMPILE_DEBUG
    /**
    * @brief 设置Python绘图接口(仅调试模式)
    * @param plot Python绘图对象指针，用于实时可视化估计数据
    */
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG

private:
    /**
    * @brief 初始化估计器系统
    * 设置卡尔曼滤波器的系统矩阵、噪声参数、初始状态等
    */
    void _initSystem();
    
    // ==================== 线性系统状态空间模型 ====================
    
    /**
    * @brief 状态向量 (18维)
    * 估计器的状态，包含：位置(3) + 速度(3) + 足端位置(3x4)
    */
    Eigen::Matrix<double, 18, 1>  _xhat;
    
    /**
    * @brief 系统输入向量 (3维)
    * 估计器的输入，通常为IMU测得的加速度
    */
    Vec3 _u;
    
    /**
    * @brief 观测向量 (28维)
    * 输出y的测量值
    */
    Eigen::Matrix<double, 28,  1> _y;
    
    /**
    * @brief 观测预测向量 (28维)
    * 输出y的预测值
    */
    Eigen::Matrix<double, 28,  1> _yhat;
    
    /**
    * @brief 系统状态转移矩阵 (18x18)
    * 估计器的转移矩阵
    */
    Eigen::Matrix<double, 18, 18> _A;
    
    /**
    * @brief 控制输入矩阵 (18x3)
    * 输入矩阵
    */
    Eigen::Matrix<double, 18, 3>  _B;
    
    /**
    * @brief 观测矩阵 (28x18)
    * 输出矩阵
    */
    Eigen::Matrix<double, 28, 18> _C;
    
    // ==================== 协方差矩阵 ====================
    
    /**
    * @brief 状态协方差矩阵 (18x18)
    * 预测协方差
    */
    Eigen::Matrix<double, 18, 18> _P;
    
    /**
    * @brief 先验状态协方差矩阵 (18x18)
    * 先验预测协方差
    */
    Eigen::Matrix<double, 18, 18> _Ppriori;
    
    /**
    * @brief 过程噪声协方差矩阵 (18x18)
    * 动力学仿真协方差
    */
    Eigen::Matrix<double, 18, 18> _Q;
    
    /**
    * @brief 观测噪声协方差矩阵 (28x28)
    * 测量协方差
    */
    Eigen::Matrix<double, 28, 28> _R;
    
    /**
    * @brief 初始过程噪声协方差矩阵 (18x18)
    * 动力学仿真协方差的初始值
    */
    Eigen::Matrix<double, 18, 18> _QInit;
    
    /**
    * @brief 初始观测噪声协方差矩阵 (28x28)
    * 测量协方差的初始值
    */
    Eigen::Matrix<double, 28, 28> _RInit;
    
    /**
    * @brief 可调过程噪声对角元素 (18维)
    * 可调节的过程噪声协方差
    */
    Vec18 _Qdig;
    
    /**
    * @brief 系统输入协方差矩阵 (3x3)
    * 系统输入u的协方差
    */
    Mat3 _Cu;
    
    // ==================== 输出测量 ====================
    
    /**
    * @brief 足端相对机体位置 (12维)
    * 足端相对机体的位置，全局坐标系表示
    */
    Eigen::Matrix<double, 12, 1>  _feetPos2Body;
    
    /**
    * @brief 足端相对机体速度 (12维)
    * 足端相对机体的速度，全局坐标系表示
    */
    Eigen::Matrix<double, 12, 1>  _feetVel2Body;
    
    /**
    * @brief 足端高度 (4维)
    * 每个足端的高度，全局坐标系表示
    */
    Eigen::Matrix<double,  4, 1>  _feetH;
    
    /**
    * @brief 新息协方差矩阵 (28x28)
    * _S = C*P*C.T + R
    */
    Eigen::Matrix<double, 28, 28> _S;
    
    /**
    * @brief S矩阵的LU分解
    * _S.lu()，用于高效求解线性方程组
    */
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _Slu;
    
    /**
    * @brief 加权观测残差 (28维)
    * _Sy = _S.inv() * (y - yhat)
    */
    Eigen::Matrix<double, 28,  1> _Sy;
    
    /**
    * @brief 预计算矩阵 (28x18)
    * _Sc = _S.inv() * C
    */
    Eigen::Matrix<double, 28, 18> _Sc;
    
    /**
    * @brief 预计算矩阵 (28x28)
    * _SR = _S.inv() * R
    */
    Eigen::Matrix<double, 28, 28> _SR;
    
    /**
    * @brief 预计算矩阵 (28x18)
    * _STC = (_S.transpose()).inv() * C
    */
    Eigen::Matrix<double, 28, 18> _STC;
    
    /**
    * @brief I - KC矩阵 (18x18)
    * _IKC = I - KC，用于协方差更新
    */
    Eigen::Matrix<double, 18, 18> _IKC;

    /**
    * @brief 机体到全局坐标系旋转矩阵
    * 旋转矩阵：从机体坐标系到全局坐标系
    */
    RotMat _rotMatB2G;
    
    /**
    * @brief 重力加速度向量
    */
    Vec3 _g;
    
    /**
    * @brief 足端位置和速度(运动学计算)
    * 通过运动学计算得到的足端全局位置和速度
    */
    Vec34 _feetPosGlobalKine, _feetVelGlobalKine;

    /**
    * @brief 底层状态指针
    * 指向底层状态数据的指针
    */
    LowlevelState* _lowState;
    
    /**
    * @brief 机器人模型指针
    * 指向机器人模型对象的指针
    */
    QuadrupedRobot *_robModel;
    
    /**
    * @brief 步态相位指针
    * 指向步态相位数组的指针
    */
    Vec4 *_phase;
    
    /**
    * @brief 足端接触状态指针
    * 指向足端接触状态数组的指针
    */
    VecInt4 *_contact;
    
    /**
    * @brief 控制周期
    * 系统控制周期，单位秒
    */
    double _dt;
    
    /**
    * @brief 信任度参数
    * 用于调节滤波器性能的信任度参数
    */
    double _trust;
    
    /**
    * @brief 大方差值
    * 用于初始化的大方差数值
    */
    double _largeVariance;

    // ==================== 低通滤波器 ====================
    
    /**
    * @brief 速度低通滤波器
    * 分别对应x、y、z方向速度的低通滤波器
    */
    LPFilter *_vxFilter, *_vyFilter, *_vzFilter;

    // ==================== 调参工具 ====================
    
    /**
    * @brief R矩阵检查工具
    * 用于检查和调节观测噪声协方差矩阵R
    */
    AvgCov *_RCheck;
    
    /**
    * @brief 输入检查工具
    * 用于检查和调节系统输入u
    */
    AvgCov *_uCheck;
    
    /**
    * @brief 估计器名称
    * 用于标识和调试的估计器名称
    */
    std::string _estName;

#ifdef COMPILE_DEBUG
    /**
    * @brief Python绘图接口
    * 调试模式下的Python绘图对象指针
    */
    PyPlot *_testPlot;
#endif  // COMPILE_DEBUG

#ifdef COMPILE_WITH_MOVE_BASE
    /**
    * @brief ROS节点句柄
    * ROS节点句柄，用于ROS通信
    */
    ros::NodeHandle _nh;
    
    /**
    * @brief ROS发布者
    * 用于发布里程计消息的ROS发布者
    */
    ros::Publisher _pub;
    
    /**
    * @brief TF变换广播器
    * 用于广播坐标变换的TF广播器
    */
    tf::TransformBroadcaster _odomBroadcaster;
    
    /**
    * @brief 当前时间戳
    * ROS时间戳，用于消息时间同步
    */
    ros::Time _currentTime;
    
    /**
    * @brief 里程计TF变换消息
    * 用于发布里程计坐标变换的消息
    */
    geometry_msgs::TransformStamped _odomTF;
    
    /**
    * @brief 里程计消息
    * 用于发布里程计信息的ROS消息
    */
    nav_msgs::Odometry _odomMsg;
    
    /**
    * @brief 发布计数器
    * 用于控制消息发布频率的计数器，初始值为0
    */
    int _count = 0;
    
    /**
    * @brief 发布频率
    * 里程计消息的发布频率，默认为10Hz
    */
    double _pubFreq = 10;

    /**
    * @brief 机体坐标系速度和角速度
    * 机体坐标系下的线速度和角速度
    */
    Vec3 _velBody, _wBody;
    
    /**
    * @brief 里程计姿态协方差 (6x6展开为36维数组)
    * 用于ROS里程计消息的姿态协方差矩阵
    * 描述位置[x,y,z]和姿态[roll,pitch,yaw]的不确定性
    */
    boost::array<double, 36> _odom_pose_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
    
    /**
    * @brief 里程计速度协方差 (6x6展开为36维数组)
    * 用于ROS里程计消息的速度协方差矩阵
    * 描述线速度[vx,vy,vz]和角速度[wx,wy,wz]的不确定性
    */
    boost::array<double, 36> _odom_twist_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0, 
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
#endif  // COMPILE_WITH_MOVE_BASE

};

#endif  // ESTIMATOR_H

/*
* ================================================================
* 算法核心原理：
* 
* 1. 状态空间模型：
*    x(k+1) = A*x(k) + B*u(k) + w(k)  // 状态方程
*    y(k) = C*x(k) + v(k)             // 观测方程
*    其中 w(k)~N(0,Q), v(k)~N(0,R)    // 噪声假设
* 
* 2. 卡尔曼滤波递推：
*    预测步骤：
*      x_pred = A*x + B*u
*      P_pred = A*P*A' + Q
*    更新步骤：  
*      K = P_pred*C'*(C*P_pred*C' + R)^(-1)
*      x = x_pred + K*(y - C*x_pred)
*      P = (I - K*C)*P_pred
* 
* 3. 多传感器融合策略：
*    - IMU数据：提供系统输入u(加速度)和姿态约束
*    - 关节编码器：通过正向运动学计算足端位置观测
*    - 接触状态：动态调整观测噪声权重
*    - 低通滤波：平滑速度估计结果
* 
* 4. ROS集成：
*    - 发布nav_msgs/Odometry消息供导航使用
*    - 广播odom->base_link的TF变换
*    - 提供标准的里程计接口
* ================================================================
*/