#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class UnitreeRobotController
{
public:
    UnitreeRobotController() : 
        safe(LeggedType::B1),
        udp(HIGHLEVEL, 8090, "192.168.123.220", 8082)
    {
        udp.InitCmdData(cmd);
        
        // 初始化命令参数
        cmd.mode = 0;
        cmd.gaitType = 0;
        cmd.speedLevel = 0;
        cmd.footRaiseHeight = 0;
        cmd.bodyHeight = 0;
        cmd.euler[0] = 0;
        cmd.euler[1] = 0;
        cmd.euler[2] = 0;
        cmd.velocity[0] = 0.0f;
        cmd.velocity[1] = 0.0f;
        cmd.yawSpeed = 0.0f;
        cmd.reserve = 0;
        
        // 创建ROS订阅者
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &UnitreeRobotController::cmdVelCallback, this);
        
        // 初始化控制循环
        control_loop = nh.createTimer(ros::Duration(0.002), &UnitreeRobotController::controlCallback, this);
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 将ROS Twist消息转换为Unitree命令
        if (msg->linear.x != 0 || msg->linear.y != 0 || msg->angular.z != 0) {
            // 如果有速度命令，切换到行走模式
            cmd.mode = 2;  // 行走模式
            cmd.velocity[0] = msg->linear.x;  // 前进/后退速度
            cmd.velocity[1] = msg->linear.y;  // 左右速度
            cmd.yawSpeed = msg->angular.z;    // 转向速度
        } else {
            // 如果没有速度命令，切换到站立模式
            cmd.mode = 1;  // 站立模式
            cmd.velocity[0] = 0;
            cmd.velocity[1] = 0;
            cmd.yawSpeed = 0;
        }
        
        last_cmd_time = ros::Time::now();
    }
    
    void controlCallback(const ros::TimerEvent& event)
    {
        // 接收机器人状态
        udp.Recv();
        udp.GetRecv(state);
        
        // 安全检查：如果长时间没有收到命令，则切换到站立模式
        if ((ros::Time::now() - last_cmd_time).toSec() > 0.5) {
            cmd.mode = 1;  // 站立模式
            cmd.velocity[0] = 0;
            cmd.velocity[1] = 0;
            cmd.yawSpeed = 0;
        }
        
        // 发送命令到机器人
        udp.SetSend(cmd);
        udp.Send();
        
        // 打印机器人状态信息（可选）
        ROS_DEBUG("Robot state - Roll: %f, Pitch: %f, Yaw: %f", 
                 state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Timer control_loop;
    ros::Time last_cmd_time;
    
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_cmd_vel_controller");
    
    std::cout << "通信级别设置为HIGH-level." << std::endl
              << "警告: 确保机器人站立在地面上." << std::endl
              << "按Enter键继续..." << std::endl;
    std::cin.ignore();
    
    UnitreeRobotController controller;
    
    ros::spin();
    
    return 0;
}