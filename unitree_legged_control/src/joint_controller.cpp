/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// 包含控制器头文件
// #include "unitree_legged_control/joint_controller.h"  // 原始包含路径
#include "joint_controller.h"

// pluginlib类列表宏 - 用于ROS插件系统注册
#include <pluginlib/class_list_macros.h>

// rqt调优功能开关 - 取消注释可启用图形化参数调优
// #define rqtTune // use rqt or not

namespace unitree_legged_control
{

/**
 * @brief 构造函数 - 初始化控制器状态
 * 
 * 将所有消息结构体和控制状态清零，确保控制器启动时处于安全状态
 */
UnitreeJointController::UnitreeJointController()
{
    // 清零上次控制命令 - 确保启动时无残留命令
    memset(&lastCmd, 0, sizeof(unitree_legged_msgs::MotorCmd));
    
    // 清零上次状态信息 - 确保状态反馈从零开始
    memset(&lastState, 0, sizeof(unitree_legged_msgs::MotorState));
    
    // 清零伺服命令结构 - 确保控制算法从安全状态开始
    memset(&servoCmd, 0, sizeof(ServoCmd));
}

/**
 * @brief 析构函数 - 清理资源
 * 
 * 关闭订阅器，释放通信资源
 */
UnitreeJointController::~UnitreeJointController()
{
    sub_ft.shutdown();   // 关闭力传感器反馈订阅器
    sub_cmd.shutdown();  // 关闭控制命令订阅器
}

/**
 * @brief 力传感器反馈回调函数
 * 
 * 处理来自力传感器的力矩反馈数据
 * 根据关节类型选择不同的力矩分量
 * 
 * @param msg 力/力矩反馈消息 - 包含六维力/力矩信息
 */
void UnitreeJointController::setTorqueCB(
        const geometry_msgs::WrenchStampedConstPtr &msg)
{
    // 根据关节类型选择对应的力矩分量
    if (isHip)
        sensor_torque = msg->wrench.torque.x;  // 髋关节使用X轴力矩
    else
        sensor_torque = msg->wrench.torque.y;  // 其他关节使用Y轴力矩
    
    // printf("sensor torque%f\n", sensor_torque);  // 调试输出（已注释）
}

/**
 * @brief 控制命令回调函数
 * 
 * 接收并缓存来自上层控制器的电机控制命令
 * 使用实时缓冲区确保实时线程安全
 * 
 * @param msg 电机控制命令消息
 */
void UnitreeJointController::setCommandCB(
        const unitree_legged_msgs::MotorCmdConstPtr &msg)
{
    // 缓存控制命令到本地变量
    lastCmd.mode = msg->mode;   // 控制模式
    lastCmd.q = msg->q;         // 目标位置
    lastCmd.Kp = msg->Kp;       // 位置增益
    lastCmd.dq = msg->dq;       // 目标速度
    lastCmd.Kd = msg->Kd;       // 速度增益
    lastCmd.tau = msg->tau;     // 前馈力矩
    
    // 将命令写入实时缓冲区
    // writeFromNonRT可以在实时线程中使用，前提是：
    //  * 没有非实时线程同时调用相同函数（我们不订阅ros回调）
    //  * 只有一个实时线程
    command.writeFromNonRT(lastCmd);
}

/**
 * @brief 控制器初始化函数 - 非实时环境下的初始化
 * 
 * 设置控制器参数、订阅器、发布器等
 * 这是ROS Control框架的标准初始化接口
 * 
 * @param robot 硬件接口指针
 * @param n ROS节点句柄
 * @return 初始化成功返回true，失败返回false
 */
bool UnitreeJointController::init(
        hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    // 初始化关节类型标志
    isHip = false;
    isThigh = false;
    isCalf = false;
    // rqtTune = false;  // rqt调优标志
    sensor_torque = 0;   // 传感器力矩初始化
    
    // 获取命名空间
    name_space = n.getNamespace();
    
    // 从参数服务器获取关节名称
    if (!n.getParam("joint", joint_name))
    {
        ROS_ERROR("No joint given in namespace: '%s')",
                  n.getNamespace().c_str());
        return false;
    }

    // 仅在需要rqt调优时加载PID参数
    // if(rqtTune) {
#ifdef rqtTune
    // 从参数服务器加载PID控制器参数
    if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
        return false;
#endif
    // }

    // 从URDF获取关节信息
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", n))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }
    
    // 获取指定关节的URDF描述信息
    joint_urdf = urdf.getJoint(joint_name);
    if (!joint_urdf)
    {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
    }
    
    // 根据关节名称自动识别关节类型
    // 髋关节识别 - 用于区分控制策略
    if (joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" ||
        joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint")
    {
        isHip = true;
    }
    
    // 小腿关节识别 - 用于足端精确控制
    if (joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" ||
        joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint")
    {
        isCalf = true;
    }
    
    // 获取关节硬件句柄 - 连接到Gazebo仿真环境
    joint = robot->getHandle(joint_name);

    // 启动控制命令订阅器
    sub_ft = n.subscribe(name_space + "/" + "joint_wrench", 1,
                         &UnitreeJointController::setTorqueCB, this);
    sub_cmd = n.subscribe("command", 20, &UnitreeJointController::setCommandCB,
                          this);

    // 启动实时状态发布器 - 替代普通发布器以保证实时性
    // pub_state = n.advertise<unitree_legged_msgs::MotorState>(name_space + "/state", 20);
    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<
                                      unitree_legged_msgs::MotorState>(
            n, name_space + "/state", 1));

    return true;
}

/**
 * @brief 设置PID控制器参数
 * 
 * @param p 比例增益
 * @param i 积分增益  
 * @param d 微分增益
 * @param i_max 积分上限
 * @param i_min 积分下限
 * @param antiwindup 抗积分饱和标志
 */
void UnitreeJointController::setGains(const double &p, const double &i,
                                      const double &d, const double &i_max,
                                      const double &i_min,
                                      const bool &antiwindup)
{
    pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

/**
 * @brief 获取PID控制器参数 - 完整版本
 */
void UnitreeJointController::getGains(double &p, double &i, double &d,
                                      double &i_max, double &i_min,
                                      bool &antiwindup)
{
    pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

/**
 * @brief 获取PID控制器参数 - 简化版本
 */
void UnitreeJointController::getGains(double &p, double &i, double &d,
                                      double &i_max, double &i_min)
{
    bool dummy;  // 抗积分饱和标志的占位变量
    pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

/**
 * @brief 控制器启动函数 - 实时环境下的启动
 * 
 * 初始化控制状态，准备开始控制循环
 * 
 * @param time 启动时间戳
 */
void UnitreeJointController::starting(const ros::Time &time)
{
    // 设置初始控制参数
    // lastCmd.Kp = 0;  // 初始增益为0（已注释）
    // lastCmd.Kd = 0;
    
    // 获取关节当前位置作为初始目标
    double init_pos = joint.getPosition();
    lastCmd.q = init_pos;      // 目标位置设为当前位置
    lastState.q = init_pos;    // 状态位置记录当前位置
    
    // 初始化速度和力矩为零
    lastCmd.dq = 0;            // 目标速度为0
    lastState.dq = 0;          // 当前速度为0
    lastCmd.tau = 0;           // 前馈力矩为0
    lastState.tauEst = 0;      // 估计力矩为0
    
    // 初始化实时命令缓冲区
    command.initRT(lastCmd);

    // 重置PID控制器状态
    pid_controller_.reset();
}

/**
 * @brief 控制器更新函数 - 实时控制循环的核心
 * 
 * 这是控制器的主要工作函数，以1000Hz频率运行
 * 实现位置-速度-力矩复合控制算法
 * 
 * @param time 当前时间戳
 * @param period 控制周期
 */
void UnitreeJointController::update(const ros::Time &time,
                                    const ros::Duration &period)
{
    double currentPos, currentVel, calcTorque;
    
    // 从实时缓冲区读取最新控制命令
    lastCmd = *(command.readFromRT());

    // 设置控制命令数据 - 根据控制模式进行不同处理
    if (lastCmd.mode == PMSM)  // PMSM模式 - 正常控制模式
    {
        // 位置控制设置
        servoCmd.pos = lastCmd.q;           // 设置目标位置
        positionLimits(servoCmd.pos);       // 应用位置安全限制
        servoCmd.posStiffness = lastCmd.Kp; // 设置位置刚度(比例增益)
        
        // 位置控制停止检查 - 当目标位置为特殊值时停止位置控制
        if (fabs(lastCmd.q - PosStopF) < 0.00001)
        {
            servoCmd.posStiffness = 0;      // 停止位置控制
        }
        
        // 速度控制设置
        servoCmd.vel = lastCmd.dq;          // 设置目标速度
        velocityLimits(servoCmd.vel);       // 应用速度安全限制
        servoCmd.velStiffness = lastCmd.Kd; // 设置速度刚度(阻尼增益)
        
        // 速度控制停止检查 - 当目标速度为特殊值时停止速度控制
        if (fabs(lastCmd.dq - VelStopF) < 0.00001)
        {
            servoCmd.velStiffness = 0;      // 停止速度控制
        }
        
        // 力矩控制设置
        servoCmd.torque = lastCmd.tau;      // 设置前馈力矩
        effortLimits(servoCmd.torque);      // 应用力矩安全限制
    }
    if (lastCmd.mode == BRAKE)  // BRAKE模式 - 制动模式
    {
        servoCmd.posStiffness = 0;          // 关闭位置控制
        servoCmd.vel = 0;                   // 目标速度设为0
        servoCmd.velStiffness = 20;         // 设置制动阻尼
        servoCmd.torque = 0;                // 前馈力矩设为0
        effortLimits(servoCmd.torque);      // 应用力矩安全限制
    }

    // } else {
    //     servoCmd.posStiffness = 0;      // 其他模式的默认设置（已注释）
    //     servoCmd.velStiffness = 5;
    //     servoCmd.torque = 0;
    // }

    // rqt参数调优 - 使用图形界面设置的PID参数
    // if(rqtTune) {
#ifdef rqtTune
    double i, i_max, i_min;
    // 从rqt界面获取P和D增益，覆盖消息中的增益设置
    getGains(servoCmd.posStiffness, i, servoCmd.velStiffness, i_max, i_min);
#endif
    // }

    // 获取关节当前状态
    currentPos = joint.getPosition();      // 获取当前位置
    
    // 计算当前速度 - 使用滤波算法平滑速度估计
    currentVel = computeVel(currentPos, (double) lastState.q,
                            (double) lastState.dq, period.toSec());
    
    // 计算控制力矩 - 核心控制算法
    calcTorque = computeTorque(currentPos, currentVel, servoCmd);
    effortLimits(calcTorque);              // 应用最终力矩限制

    // 输出控制命令到硬件接口
    joint.setCommand(calcTorque);

    // 更新状态缓存 - 为下次控制循环准备
    lastState.q = currentPos;              // 更新位置状态
    lastState.dq = currentVel;             // 更新速度状态
    // lastState.tauEst = calcTorque;      // 使用计算力矩（已注释）
    // lastState.tauEst = sensor_torque;   // 使用传感器力矩（已注释）
    lastState.tauEst = joint.getEffort();  // 使用硬件接口返回的力矩

    // 发布状态反馈 - 非阻塞实时发布
    // pub_state.publish(lastState);       // 普通发布器（已注释）
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
        // 设置发布消息内容
        controller_state_publisher_->msg_.q = lastState.q;           // 当前位置
        controller_state_publisher_->msg_.dq = lastState.dq;         // 当前速度
        controller_state_publisher_->msg_.tauEst = lastState.tauEst; // 估计力矩
        
        // 非阻塞发布 - 保证实时性
        controller_state_publisher_->unlockAndPublish();
    }

    // printf("sensor torque%f\n", sensor_torque);  // 调试输出（已注释）

    // 调试输出示例（已注释）
    // if(joint_name == "wrist1_joint") printf("wrist1 setp:%f  getp:%f t:%f\n", servoCmd.pos, currentPos, calcTorque);
}

/**
 * @brief 控制器停止函数 - 实时环境下的停止
 * 
 * 控制器停止时的清理工作，当前为空实现
 */
void UnitreeJointController::stopping()
{
    // 当前无特殊停止处理逻辑
}

// 速度计算函数的原始实现（已注释）
// 这里展示了另一种可能的速度计算方法，使用了互斥锁保护
// double UnitreeJointController::computeVel
// (double &currentPos, double &lastPos, double &lastVel, double &period)
// {
//     pthread_mutex_clocklock(&clock_lock);
//     lastPos = currentPos;
//     lastVel = currentVel;
//     double currentVel;
//     if (fabs(lastPos - currentPos) < 0.00001)
//     {
//         currentVel = lastVel;
//     }
//     else
//     {
//         currentVel = (currentPos - lastPos) / period;
//     }
//
//     pthread_mutex_unclocklock(&clock_lock);
//     return currentVel;
//
// }

/**
 * @brief 位置安全限制函数
 * 
 * 基于URDF中定义的关节限制对位置进行约束
 * 
 * @param position 待限制的位置值（引用传递，会被修改）
 */
void UnitreeJointController::positionLimits(double &position)
{
    // 仅对旋转关节和直线关节应用限制
    if (joint_urdf->type == urdf::Joint::REVOLUTE ||
        joint_urdf->type == urdf::Joint::PRISMATIC)
        clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
}

/**
 * @brief 速度安全限制函数
 * 
 * 基于URDF中定义的关节限制对速度进行约束
 * 
 * @param velocity 待限制的速度值（引用传递，会被修改）
 */
void UnitreeJointController::velocityLimits(double &velocity)
{
    // 仅对旋转关节和直线关节应用限制
    if (joint_urdf->type == urdf::Joint::REVOLUTE ||
        joint_urdf->type == urdf::Joint::PRISMATIC)
        clamp(velocity, -joint_urdf->limits->velocity,
              joint_urdf->limits->velocity);
}

/**
 * @brief 力矩安全限制函数
 * 
 * 基于URDF中定义的关节限制对力矩进行约束
 * 
 * @param effort 待限制的力矩值（引用传递，会被修改）
 */
void UnitreeJointController::effortLimits(double &effort)
{
    // 仅对旋转关节和直线关节应用限制
    if (joint_urdf->type == urdf::Joint::REVOLUTE ||
        joint_urdf->type == urdf::Joint::PRISMATIC)
        clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
}

}// namespace unitree_legged_control

// 向pluginlib注册控制器 - 使控制器可以被ROS Control框架动态加载
// 这个宏将UnitreeJointController注册为controller_interface::ControllerBase的实现
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController,
                       controller_interface::ControllerBase);