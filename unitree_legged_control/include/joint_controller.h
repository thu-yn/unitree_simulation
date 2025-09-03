/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _UNITREE_ROS_JOINT_CONTROLLER_H_
#define _UNITREE_ROS_JOINT_CONTROLLER_H_

// ROS核心功能
#include <ros/node_handle.h>
#include <urdf/model.h>

// 控制工具库
#include <control_toolbox/pid.h>

// Boost库工具
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

// 实时工具库 - 保证实时性能
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

// ROS Control框架接口
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

// 标准ROS消息
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

// Unitree机器人专用消息
#include "unitree_legged_msgs/MotorCmd.h"      // 电机控制命令消息
#include "unitree_legged_msgs/MotorState.h"    // 电机状态反馈消息

// Unitree关节控制工具 - 包含控制算法实现
#include "unitree_joint_control_tool.h"

// 控制器模式定义
#define PMSM      (0x0A)        // 永磁同步电机模式 - 正常运行模式
#define BRAKE     (0x00)        // 制动模式 - 关节制动状态

// 控制停止标志值
#define PosStopF  (2.146E+9f)   // 位置控制停止标志 - 当目标位置为此值时停止位置控制
#define VelStopF  (16000.0f)    // 速度控制停止标志 - 当目标速度为此值时停止速度控制

namespace unitree_legged_control
{
    /**
     * @brief Unitree关节控制器类
     * 
     * 继承自ROS Control框架的Controller基类，专门为Unitree四足机器人设计
     * 支持位置、速度、力矩的复合控制模式
     * 实现了标准的ROS Control插件接口，可通过YAML配置文件动态加载
     */
    class UnitreeJointController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
private:
        // 硬件接口层
        hardware_interface::JointHandle joint;    // 关节硬件句柄 - 与Gazebo仿真环境的接口
        
        // ROS通信接口
        ros::Subscriber sub_cmd, sub_ft;          // 订阅器：控制命令和力反馈
        // ros::Publisher pub_state;             // 状态发布器（已注释，使用实时发布器替代）
        
        // 控制算法组件
        control_toolbox::Pid pid_controller_;    // PID控制器 - 用于rqt参数调优时的控制
        
        // 实时状态发布器 - 保证实时性能的状态反馈
        boost::scoped_ptr<realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState> > controller_state_publisher_ ;

public:
        // 配置参数
        // bool start_up;                         // 启动标志（已注释）
        std::string name_space;                   // 命名空间 - 用于多机器人或多关节区分
        std::string joint_name;                   // 关节名称 - 如"FL_hip_joint"等
        
        // 传感器数据
        float sensor_torque;                      // 传感器测得的力矩值
        
        // 关节类型标识 - 用于不同关节的差异化控制
        bool isHip, isThigh, isCalf;             // 分别标识髋关节、大腿关节、小腿关节
        bool rqtTune;                            // rqt调参标志 - 是否启用图形化参数调优
        
        // URDF模型信息
        urdf::JointConstSharedPtr joint_urdf;    // URDF关节信息 - 包含关节限制等参数
        
        // 实时通信缓冲区 - 确保实时线程安全
        realtime_tools::RealtimeBuffer<unitree_legged_msgs::MotorCmd> command;  // 命令缓冲区
        
        // 状态缓存
        unitree_legged_msgs::MotorCmd lastCmd;      // 上次接收的控制命令
        unitree_legged_msgs::MotorState lastState;  // 上次的关节状态
        
        // 伺服控制命令结构
        ServoCmd servoCmd;                        // 转换后的伺服控制命令

        // 构造函数和析构函数
        UnitreeJointController();
        ~UnitreeJointController();
        
        // ROS Control框架标准接口 - 控制器生命周期管理
        virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        virtual void starting(const ros::Time& time);     // 控制器启动时调用
        virtual void update(const ros::Time& time, const ros::Duration& period);  // 实时控制循环
        virtual void stopping();                          // 控制器停止时调用
        
        // 回调函数 - 处理外部输入
        void setTorqueCB(const geometry_msgs::WrenchStampedConstPtr& msg);    // 力传感器反馈回调
        void setCommandCB(const unitree_legged_msgs::MotorCmdConstPtr& msg);  // 控制命令回调
        
        // 安全限制函数 - 保护硬件安全
        void positionLimits(double &position);    // 位置限制 - 基于URDF定义的关节限制
        void velocityLimits(double &velocity);    // 速度限制 - 防止过速运动
        void effortLimits(double &effort);        // 力矩限制 - 防止过载

        // PID参数配置接口 - 支持运行时参数调整
        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    };
}

#endif