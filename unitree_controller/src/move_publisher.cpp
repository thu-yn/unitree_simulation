/*
* move_publisher.cpp - Unitree四足机器人位置控制发布器
* 
* 文件作用：
* 这个文件是unitree_controller模块中的位置控制工具，主要用于在Gazebo仿真环境中
* 控制机器人的位置和姿态运动。它提供了两种控制模式：
* 
* 1. 世界坐标系模式 (WORLD)：
*    - 让机器人在世界坐标系下执行圆周运动
*    - 适用于测试全局定位、SLAM算法等需要固定轨迹的场景
*    - 机器人会以原点为中心，半径1.5米做圆周运动
* 
* 2. 机器人坐标系模式 (ROBOT)：
*    - 在机器人本体坐标系下控制运动
*    - 适用于测试机器人的局部运动控制
*    - 机器人会持续前进并上升
* 
* 应用场景：
* - SLAM算法开发：提供可重复的运动轨迹用于建图测试
* - 视觉开发：提供稳定的运动模式用于视觉算法调试
* - 导航算法测试：模拟机器人的预期运动轨迹
* - 传感器标定：通过已知运动轨迹标定传感器
* 
* 注意：这是一个仿真工具，直接通过Gazebo的ModelState接口控制机器人位置，
* 绕过了机器人的运动学和动力学控制，主要用于开发和测试阶段。
*/

#include <stdio.h>
#include <string>
#include <gazebo_msgs/ModelState.h>      // Gazebo模型状态消息
#include <gazebo_msgs/SetModelState.h>   // Gazebo设置模型状态服务
#include <ros/ros.h>                     // ROS核心功能
#include <tf/transform_datatypes.h>      // TF坐标变换数据类型
#include <iostream>
#include <math.h>                        // 数学函数库

int main(int argc, char **argv)
{
    // 定义坐标系枚举类型
    enum coord
    {
        WORLD,    // 世界坐标系模式
        ROBOT     // 机器人坐标系模式
    };
    
    // 设置默认控制模式为世界坐标系
    // 可以修改这个值来切换控制模式：
    // WORLD: 圆周运动模式，适合SLAM和全局定位测试
    // ROBOT: 相对运动模式，适合局部运动控制测试
    coord def_frame = coord::WORLD;

    // 初始化ROS节点
    ros::init(argc, argv, "move_publisher");
    ros::NodeHandle nh;
    
    // 创建发布器，向Gazebo发送模型状态命令
    // "/gazebo/set_model_state" 是Gazebo提供的标准话题，用于设置模型的位置和速度
    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>(
            "/gazebo/set_model_state", 1000);

    // 创建模型状态消息对象
    gazebo_msgs::ModelState model_state_pub;

    // 从ROS参数服务器获取机器人名称
    // 这个参数通常在launch文件中设置，支持多种机器人型号(go1, a1, laikago等)
    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;

    // 设置要控制的模型名称
    // 在Gazebo中，机器人模型的名称格式为 "{robot_name}_gazebo"
    model_state_pub.model_name = robot_name + "_gazebo";
    
    // 设置控制循环频率为1000Hz，保证平滑的运动控制
    ros::Rate loop_rate(1000);

    // ============ 世界坐标系控制模式 ============
    if (def_frame == coord::WORLD)
    {
        // 设置机器人初始位置和姿态
        model_state_pub.pose.position.x = 0.0;    // X轴位置：原点
        model_state_pub.pose.position.y = 0.0;    // Y轴位置：原点  
        model_state_pub.pose.position.z = 0.5;    // Z轴位置：0.5米高度(避免与地面碰撞)

        // 设置初始姿态为标准姿态(无旋转)
        model_state_pub.pose.orientation.x = 0.0;
        model_state_pub.pose.orientation.y = 0.0;
        model_state_pub.pose.orientation.z = 0.0;
        model_state_pub.pose.orientation.w = 1.0;  // 四元数的w分量为1表示无旋转

        // 指定参考坐标系为世界坐标系
        model_state_pub.reference_frame = "world";

        // 圆周运动参数设置
        long long time_ms = 0;           // 时间计数器，单位：毫秒
        const double period = 5000;      // 圆周运动周期：5000毫秒 = 5秒一圈
        const double radius = 1.5;       // 圆周运动半径：1.5米
        
        tf::Quaternion q;                // 四元数对象，用于姿态计算

        // 主控制循环：实现圆周运动
        while (ros::ok())
        {
            // 计算圆周运动的X坐标
            // 使用sin函数实现圆周运动的X分量
            model_state_pub.pose.position.x =
                    radius * sin(2 * M_PI * (double) time_ms / period);
            
            // 计算圆周运动的Y坐标  
            // 使用cos函数实现圆周运动的Y分量
            model_state_pub.pose.position.y =
                    radius * cos(2 * M_PI * (double) time_ms / period);
            
            // 计算机器人的朝向角度
            // 机器人的朝向随着运动方向改变，实现更真实的运动效果
            // 负号表示逆时针旋转
            model_state_pub.pose.orientation =
                    tf::createQuaternionMsgFromRollPitchYaw(
                            0, 0, -2 * M_PI * (double) time_ms / period);
            
            // 发布模型状态消息到Gazebo
            move_publisher.publish(model_state_pub);
            
            // 按设定频率休眠，保证1000Hz的控制频率
            loop_rate.sleep();
            
            // 时间递增1毫秒
            time_ms += 1;
        }
    }
    // ============ 机器人坐标系控制模式 ============
    else if (def_frame == coord::ROBOT)
    {
        // 设置机器人在本体坐标系下的运动速度
        // 这种模式下直接设置速度，而不是位置
        
        model_state_pub.twist.linear.x = 0.02;   // X轴线速度：0.02 m/s = 2 cm/s (向前运动)
        model_state_pub.twist.linear.y = 0.0;    // Y轴线速度：0 (不进行侧向运动)
        model_state_pub.twist.linear.z = 0.08;   // Z轴线速度：0.08 m/s = 8 cm/s (向上运动)

        // 设置角速度为0(不旋转)
        model_state_pub.twist.angular.x = 0.0;   // 绕X轴角速度
        model_state_pub.twist.angular.y = 0.0;   // 绕Y轴角速度  
        model_state_pub.twist.angular.z = 0.0;   // 绕Z轴角速度

        // 指定参考坐标系为机器人本体坐标系
        // "base" 通常指机器人的基础坐标系(base_link)
        model_state_pub.reference_frame = "base";

        // 主控制循环：持续发送速度命令
        while (ros::ok())
        {
            // 持续发布速度命令
            // 在这种模式下，机器人会以恒定速度前进并上升
            move_publisher.publish(model_state_pub);
            
            // 按设定频率休眠
            loop_rate.sleep();
        }
    }
    
    // 程序正常结束
    return 0;
}