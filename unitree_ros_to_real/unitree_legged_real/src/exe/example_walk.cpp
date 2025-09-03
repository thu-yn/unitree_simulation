/**
* @file example_walk.cpp
* @brief Unitree四足机器人高层运动控制示例程序
* 
* 本程序演示如何使用高层控制模式对机器人进行运动控制。高层控制提供了
* 用户友好的接口，可以直接控制机器人的步态、姿态、速度和运动方向，
* 无需关心底层的关节控制细节。
* 
* 程序包含以下运动序列：
* 1. 起立准备
* 2. 姿态调整(roll, pitch, yaw方向的倾斜)
* 3. 前进行走和转向
* 4. 侧向移动
* 5. 不同步态展示
* 
* ⚠️ 注意：高层控制模式要求机器人站立在地面上
* 
* @author Unitree Robotics
* @date 2018-2019
*/

#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>     // 高层控制命令消息
#include <unitree_legged_msgs/HighState.h>   // 高层状态反馈消息
#include "../unitree_legged_sdk/include/unitree_legged_sdk.h"  // Unitree SDK
#include "convert.h"                         // ROS消息与SDK数据转换函数

using namespace UNITREE_LEGGED_SDK;

// 全局变量：存储高层状态反馈数据
unitree_legged_msgs::HighState high_state_ros;

/**
* @brief 高层状态回调函数
* @param state 接收到的高层状态消息
* 
* 此函数在每次接收到机器人高层状态反馈时被调用。
* 高层状态包含机器人的整体运动信息，如位置、速度、姿态、
* 足端接触力、IMU数据等。
*/
void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &state)
{
    static long count = 0;
    ROS_INFO("highStateCallback %ld", count++);
    
    // 保存最新的状态信息到全局变量
    high_state_ros = *state;
}

/**
* @brief 主函数 - 高层运动控制示例
*/
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "example_walk_without_lcm");

    // 显示控制模式信息和安全提示
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
            << "Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();  // 等待用户确认

    // 创建ROS节点句柄和设置循环频率
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);  // 500Hz控制频率

    // =============================================================================
    // 控制参数初始化
    // =============================================================================
    
    long motiontime = 0;  // 运动时间计数器(单位：2ms)

    // 创建高层控制命令消息
    unitree_legged_msgs::HighCmd high_cmd_ros;

    // =============================================================================
    // ROS发布者和订阅者设置
    // =============================================================================
    
    // 创建发布者：发送高层控制命令到机器人
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    
    // 创建订阅者：接收机器人高层状态反馈
    ros::Subscriber sub = nh.subscribe("high_state", 1000, highStateCallback);

    // =============================================================================
    // 主控制循环
    // =============================================================================
    
    while (ros::ok())
    {
        // 打印当前IMU的欧拉角信息(roll, pitch, yaw)，用于监控机器人姿态
        printf("rpy: %f %f %f\n", 
            high_state_ros.imu.rpy[0],  // Roll角 (绕X轴旋转)
            high_state_ros.imu.rpy[1],  // Pitch角(绕Y轴旋转)
            high_state_ros.imu.rpy[2]); // Yaw角  (绕Z轴旋转)

        // 时间计数器递增(每个循环周期2ms)
        motiontime += 2;

        // =============================================================================
        // 高层控制命令基础参数设置(每次循环都需要设置)
        // =============================================================================
        
        // 设置数据包头标识(固定值，用于通信协议识别)
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        
        // 设置控制级别标志为高层控制
        high_cmd_ros.levelFlag = HIGHLEVEL;
        
        // 初始化控制参数为默认值
        high_cmd_ros.mode = 0;              // 运动模式(0=空闲)
        high_cmd_ros.gaitType = 0;          // 步态类型(0=空闲)
        high_cmd_ros.speedLevel = 0;        // 速度等级
        high_cmd_ros.footRaiseHeight = 0;   // 足部抬起高度
        high_cmd_ros.bodyHeight = 0;        // 身体高度调整
        high_cmd_ros.euler[0] = 0;          // Roll角目标值
        high_cmd_ros.euler[1] = 0;          // Pitch角目标值
        high_cmd_ros.euler[2] = 0;          // Yaw角目标值
        high_cmd_ros.velocity[0] = 0.0f;    // X方向速度(前进/后退)
        high_cmd_ros.velocity[1] = 0.0f;    // Y方向速度(左右平移)
        high_cmd_ros.yawSpeed = 0.0f;       // 偏航角速度(转向)
        high_cmd_ros.reserve = 0;           // 保留字段

        // =============================================================================
        // 运动序列控制 - 基于时间的状态机
        // =============================================================================
        
        // 阶段1：起立准备阶段 (0-4000ms)
        // 功能：让机器人从任意状态转换到站立准备状态
        if (motiontime > 0 && motiontime < 2000)
        {
            high_cmd_ros.mode = 6;  // 模式6：起立模式，机器人会自动调整到站立姿态
        }
        // 阶段2：站立保持阶段 (4000ms-6000ms)
        // 功能：保持基本站立姿态，为后续运动做准备
        else if(motiontime >= 2000 && motiontime < 3000)
        {
            high_cmd_ros.mode = 1;  // 模式1：站立模式，机器人保持站立状态
        }
        // 阶段3：Roll方向倾斜测试 (6000ms-8000ms)
        // 功能：测试机器人绕X轴的倾斜能力(左右倾斜)
        else if(motiontime >= 3000 && motiontime < 4000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[0] = 0.3;  // Roll角设为0.3弧度(约17度)，机器人向右倾斜
        }
        // 阶段4：反向Roll倾斜 (8000ms-12000ms)
        // 功能：向相反方向倾斜，展示平衡控制能力
        else if(motiontime >= 4000 && motiontime < 6000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[0] = -0.3; // Roll角设为-0.3弧度，机器人向左倾斜
        }
        // 阶段5：Pitch方向倾斜测试 (12000ms-16000ms)
        // 功能：测试机器人绕Y轴的倾斜能力(前后倾斜)
        else if(motiontime >= 6000 && motiontime < 8000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[1] = 0.3;  // Pitch角设为0.3弧度，机器人向后倾斜(抬头)
        }
        // 阶段6：反向Pitch倾斜 (16000ms-20000ms)
        // 功能：向前倾斜，完整展示俯仰控制
        else if(motiontime >= 8000 && motiontime < 10000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[1] = -0.3; // Pitch角设为-0.3弧度，机器人向前倾斜(低头)
        }
        // 阶段7：Yaw方向旋转测试 (20000ms-24000ms)
        // 功能：测试机器人绕Z轴的旋转能力(偏航)
        else if(motiontime >= 10000 && motiontime < 12000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[2] = 0.3;  // Yaw角设为0.3弧度，机器人向左转
        }
        // 阶段8：反向Yaw旋转 (24000ms-28000ms)
        // 功能：向右旋转，完整展示偏航控制
        else if(motiontime >= 12000 && motiontime < 14000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[2] = -0.3; // Yaw角设为-0.3弧度，机器人向右转
        }
        // 阶段9：姿态复位 (28000ms-30000ms)
        // 功能：将所有姿态角复位到0，准备进行移动测试
        else if(motiontime >= 14000 && motiontime < 15000)
        {
            high_cmd_ros.mode = 1;  // 所有euler角都为0，机器人回到中性姿态
        }
        // 阶段10：前进+转向运动 (30000ms-36000ms)
        // 功能：展示复合运动能力，同时前进和转向
        else if(motiontime >= 15000 && motiontime < 18000)
        {
            high_cmd_ros.mode = 2;          // 模式2：行走模式
            high_cmd_ros.velocity[0] = 0.3; // X方向速度0.3m/s(前进)
            high_cmd_ros.yawSpeed = 0.2;    // 偏航角速度0.2rad/s(边走边左转)
        }
        // 阶段11：侧移+转向运动 (36000ms-42000ms)
        // 功能：展示侧向移动能力
        else if(motiontime >= 18000 && motiontime < 21000)
        {
            high_cmd_ros.mode = 2;           // 行走模式
            high_cmd_ros.velocity[1] = -0.3; // Y方向速度-0.3m/s(向右侧移)
            high_cmd_ros.yawSpeed = -0.2;    // 偏航角速度-0.2rad/s(边移动边右转)
        }
        // 阶段12：运动停止 (42000ms-44000ms)
        // 功能：停止所有运动，回到站立状态
        else if(motiontime >= 21000 && motiontime < 22000)
        {
            high_cmd_ros.mode = 1;  // 回到站立模式，停止移动
        }
        // 阶段13：步态切换展示 (44000ms-50000ms)
        // 功能：展示不同的步态类型
        else if(motiontime >= 22000 && motiontime < 25000)
        {
            high_cmd_ros.mode = 2;        // 行走模式
            high_cmd_ros.gaitType = 3;    // 步态类型3(可能是小跑步态)
        }
        // 阶段14：最终停止 (50000ms-52000ms)
        // 功能：完成演示后回到站立状态
        else if(motiontime >= 25000 && motiontime < 26000)
        {
            high_cmd_ros.mode = 1;  // 最终站立状态
        }
        // 阶段15：程序结束后的空闲状态
        // 功能：演示完成，机器人进入空闲模式
        else 
        {
            high_cmd_ros.mode = 0;  // 模式0：空闲模式，机器人放松
        }

        // =============================================================================
        // 发布控制命令和处理ROS回调
        // =============================================================================
        
        // 发布高层控制命令到机器人
        pub.publish(high_cmd_ros);

        // 处理ROS回调函数(接收状态反馈)
        ros::spinOnce();
        
        // 按设定频率休眠，维持500Hz控制循环
        loop_rate.sleep();
    }

    return 0;
}