/**********************************************************************
 * 文件作用：
 * IOROS.cpp - ROS/Gazebo仿真环境接口实现
 * 
 * 这个文件是unitree_guide项目中的核心通信接口，专门用于ROS/Gazebo仿真环境。
 * 它继承自IOInterface抽象基类，实现了与Gazebo仿真器的双向通信功能：
 * 
 * 主要功能：
 * 1. 向Gazebo发送关节控制命令（位置、速度、力矩）
 * 2. 从Gazebo接收关节状态反馈（位置、速度、力矩估计）
 * 3. 接收IMU传感器数据（姿态、角速度、加速度）
 * 4. 处理键盘用户输入命令
 * 
 * 控制架构：
 * unitree_guide → IOROS → ROS Topics → Gazebo → 机器人仿真模型
 * 
 * 支持的机器人型号：A1、Go1、Laikago等Unitree四足机器人
 * 
 * 注意：此文件仅在COMPILE_WITH_ROS宏定义时编译，用于仿真环境
 ***********************************************************************/

#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"     // IOROS类定义
#include "interface/KeyBoard.h"  // 键盘输入处理
#include <iostream>
#include <unistd.h>              // usleep函数
#include <csignal>               // 信号处理

/**
 * ROS关闭信号处理函数
 * 当接收到SIGINT信号（Ctrl+C）时，优雅地关闭ROS节点
 * @param sig 信号编号
 */
void RosShutDown(int sig){
    ROS_INFO("ROS interface shutting down!");  // 打印关闭信息
    ros::shutdown();                          // 关闭ROS节点
}

/**
 * IOROS构造函数
 * 初始化ROS接口，建立与Gazebo仿真环境的通信连接
 * 执行流程：
 * 1. 获取机器人名称参数
 * 2. 初始化订阅器（接收状态）
 * 3. 启动异步处理线程
 * 4. 初始化发布器（发送命令）
 * 5. 设置信号处理和键盘控制
 */
IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    
    // 从ROS参数服务器获取机器人名称
    // 支持的机器人: go1, a1, laikago等
    ros::param::get("/robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // 第一步：启动订阅器，用于接收机器人状态
    initRecv();
    
    // 启动异步处理线程，处理ROS消息回调
    ros::AsyncSpinner subSpinner(1); // 使用单线程处理回调
    subSpinner.start();
    
    // 等待300ms确保所有订阅器正常启动
    usleep(300000);     
    
    // 第二步：初始化发布器，用于发送控制命令
    initSend();   

    // 设置SIGINT信号处理器，支持Ctrl+C优雅退出
    signal(SIGINT, RosShutDown);

    // 初始化键盘命令面板，支持用户交互控制
    cmdPanel = new KeyBoard();
}

/**
 * IOROS析构函数
 * 清理资源并关闭ROS节点
 */
IOROS::~IOROS(){
    delete cmdPanel;    // 释放键盘控制面板
    ros::shutdown();    // 关闭ROS节点
}

/**
 * 核心通信函数 - 发送命令并接收状态
 * 这是控制循环的核心函数，每个控制周期（500Hz）调用一次
 * 
 * @param cmd 待发送的底层控制命令（关节位置、速度、力矩）
 * @param state 接收的底层状态反馈（关节状态、IMU数据、用户命令）
 */
void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    // 发送关节控制命令到Gazebo
    sendCmd(cmd);
    
    // 接收机器人状态反馈
    recvState(state);

    // 获取键盘用户输入命令
    state->userCmd = cmdPanel->getUserCmd();      // 获取用户按键命令
    state->userValue = cmdPanel->getUserValue();  // 获取用户输入数值
}

/**
 * 发送控制命令到Gazebo
 * 将unitree_guide的控制命令转换为ROS消息并发布到相应话题
 * 
 * @param lowCmd 包含12个关节控制参数的命令结构体
 * 
 * 控制参数说明：
 * - mode: 控制模式（位置/速度/力矩控制）
 * - q: 目标关节位置（弧度）
 * - dq: 目标关节速度（弧度/秒）
 * - tau: 目标关节力矩（N·m）
 * - Kp: 位置增益
 * - Kd: 微分增益
 */
void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    // 复制12个关节的控制参数到内部消息结构
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;  // 控制模式
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;        // 目标位置
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;      // 目标速度
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;    // 目标力矩
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;      // 微分增益
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;      // 位置增益
    }
    
    // 向每个关节控制器发布命令
    for(int m(0); m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    
    // 处理一次ROS事件循环，确保消息发送
    ros::spinOnce();
}

/**
 * 接收机器人状态反馈
 * 从内部状态缓存中读取最新的机器人状态数据
 * 
 * @param state 输出参数，用于存储接收到的状态信息
 * 
 * 状态信息包括：
 * - 12个关节的位置、速度、加速度、估计力矩
 * - IMU传感器的四元数、陀螺仪、加速度计数据
 */
void IOROS::recvState(LowlevelState *state){
    // 复制12个关节的状态信息
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;         // 当前关节位置
        state->motorState[i].dq = _lowState.motorState[i].dq;       // 当前关节速度
        state->motorState[i].ddq = _lowState.motorState[i].ddq;     // 当前关节加速度
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst; // 估计关节力矩
    }
    
    // 复制IMU姿态数据（四元数：x, y, z, w）
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];        // x, y, z分量
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];  // 加速度计x, y, z
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];          // 陀螺仪x, y, z
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];            // w分量
}

/**
 * 初始化ROS发布器
 * 为每个关节控制器创建发布器，用于发送控制命令
 * 
 * 话题命名规则：/{robot_name}_gazebo/{joint_name}_controller/command
 * 
 * 关节编号对应关系：
 * 0-2:  前右腿（FR: hip, thigh, calf）
 * 3-5:  前左腿（FL: hip, thigh, calf）  
 * 6-8:  后右腿（RR: hip, thigh, calf）
 * 9-11: 后左腿（RL: hip, thigh, calf）
 */
void IOROS::initSend(){
    // 前右腿（Front Right）关节发布器
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    
    // 前左腿（Front Left）关节发布器
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    
    // 后右腿（Rear Right）关节发布器
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    
    // 后左腿（Rear Left）关节发布器
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
}

/**
 * 初始化ROS订阅器
 * 为IMU传感器和每个关节状态创建订阅器，用于接收反馈数据
 * 
 * 订阅的话题：
 * - IMU数据：/trunk_imu
 * - 关节状态：/{robot_name}_gazebo/{joint_name}_controller/state
 */
void IOROS::initRecv(){
    // 订阅IMU传感器数据（机身姿态、角速度、线加速度）
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
    
    // 前右腿（Front Right）关节状态订阅器
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    
    // 前左腿（Front Left）关节状态订阅器
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    
    // 后右腿（Rear Right）关节状态订阅器
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    
    // 后左腿（Rear Left）关节状态订阅器
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);
}

/**
 * IMU传感器数据回调函数
 * 当接收到新的IMU数据时被调用，更新内部状态缓存
 * 
 * @param msg 来自ROS的IMU消息，包含姿态、角速度、线加速度
 * 
 * 数据转换说明：
 * - 四元数存储顺序：[w, x, y, z]
 * - 角速度单位：rad/s
 * - 线加速度单位：m/s²
 */ 
void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    // 姿态四元数（w, x, y, z顺序）
    _lowState.imu.quaternion[0] = msg.orientation.w;  // w分量
    _lowState.imu.quaternion[1] = msg.orientation.x;  // x分量
    _lowState.imu.quaternion[2] = msg.orientation.y;  // y分量
    _lowState.imu.quaternion[3] = msg.orientation.z;  // z分量

    // 角速度（rad/s）
    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;  // 绕x轴角速度
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;  // 绕y轴角速度
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;  // 绕z轴角速度
    
    // 线加速度（m/s²）
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;  // x方向加速度
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;  // y方向加速度
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;  // z方向加速度
}

// ==================== 关节状态回调函数 ====================
// 以下12个回调函数分别处理每个关节的状态反馈
// 每个函数将接收到的关节状态数据存储到对应的数组位置

/**
 * 前右髋关节状态回调函数
 * 关节编号：0，对应FR_hip_controller
 */
void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;        // 当前控制模式
    _lowState.motorState[0].q = msg.q;              // 当前关节位置（弧度）
    _lowState.motorState[0].dq = msg.dq;            // 当前关节速度（弧度/秒）
    _lowState.motorState[0].tauEst = msg.tauEst;    // 估计关节力矩（N·m）
}

/**
 * 前右大腿关节状态回调函数
 * 关节编号：1，对应FR_thigh_controller
 */
void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

/**
 * 前右小腿关节状态回调函数
 * 关节编号：2，对应FR_calf_controller
 */
void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

/**
 * 前左髋关节状态回调函数
 * 关节编号：3，对应FL_hip_controller
 */
void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

/**
 * 前左大腿关节状态回调函数
 * 关节编号：4，对应FL_thigh_controller
 */
void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

/**
 * 前左小腿关节状态回调函数
 * 关节编号：5，对应FL_calf_controller
 */
void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

/**
 * 后右髋关节状态回调函数
 * 关节编号：6，对应RR_hip_controller
 */
void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

/**
 * 后右大腿关节状态回调函数
 * 关节编号：7，对应RR_thigh_controller
 */
void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

/**
 * 后右小腿关节状态回调函数
 * 关节编号：8，对应RR_calf_controller
 */
void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

/**
 * 后左髋关节状态回调函数
 * 关节编号：9，对应RL_hip_controller
 */
void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

/**
 * 后左大腿关节状态回调函数
 * 关节编号：10，对应RL_thigh_controller
 */
void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

/**
 * 后左小腿关节状态回调函数
 * 关节编号：11，对应RL_calf_controller
 */
void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

#endif  // COMPILE_WITH_ROS