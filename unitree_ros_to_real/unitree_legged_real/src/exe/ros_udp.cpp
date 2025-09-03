/**
* @file ros_udp.cpp
* @brief ROS与Unitree机器人UDP通信桥梁程序
* 
* 这是unitree_ros_to_real包的核心程序，负责在ROS消息系统与Unitree机器人
* 的UDP通信协议之间建立双向数据转换桥梁。程序支持高层和底层两种控制模式，
* 实现了仿真环境与真实机器人的无缝对接。
* 
* 主要功能：
* 1. 接收ROS控制命令，转换为Unitree SDK格式，通过UDP发送给机器人
* 2. 接收机器人UDP状态数据，转换为ROS消息格式，发布到ROS话题
* 3. 支持高层控制(运动控制)和底层控制(关节控制)双模式
* 4. 提供实时的双向通信，保证控制的实时性
* 
* 使用方法：
* - 高层控制：rosrun unitree_legged_real ros_udp HIGHLEVEL
* - 底层控制：rosrun unitree_legged_real ros_udp LOWLEVEL
* 
* @author Unitree Robotics
* @date 2018-2019
*/

#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>     // 高层控制命令消息
#include <unitree_legged_msgs/HighState.h>   // 高层状态反馈消息
#include <unitree_legged_msgs/LowCmd.h>      // 底层控制命令消息
#include <unitree_legged_msgs/LowState.h>    // 底层状态反馈消息
#include "../unitree_legged_sdk/include/unitree_legged_sdk.h"  // Unitree SDK
#include "convert.h"                         // ROS消息与SDK数据转换函数
#include <chrono>                            // 时间处理
#include <pthread.h>                         // 多线程支持
#include <geometry_msgs/Twist.h>             // 标准ROS速度消息(未使用)

using namespace UNITREE_LEGGED_SDK;

/**
* @class Custom
* @brief 自定义通信类，封装UDP通信和数据转换功能
* 
* 此类管理与Unitree机器人的UDP通信连接，包括高层和底层两个通信通道。
* 每个通道都有独立的发送和接收功能，支持并发操作。
*/
class Custom
{
public:
    // UDP通信对象
    UDP low_udp;   // 底层控制UDP连接
    UDP high_udp;  // 高层控制UDP连接

    // 高层控制数据结构
    HighCmd high_cmd = {0};     // 高层控制命令(发送到机器人)
    HighState high_state = {0}; // 高层状态反馈(从机器人接收)

    // 底层控制数据结构
    LowCmd low_cmd = {0};       // 底层控制命令(发送到机器人)
    LowState low_state = {0};   // 底层状态反馈(从机器人接收)

public:
    /**
    * @brief 构造函数 - 初始化UDP连接
    * 
    * 创建两个UDP连接通道：
    * - 底层控制：用于关节级精确控制
    * - 高层控制：用于整体运动控制
    * 
    * 网络配置：
    * - 机器人IP：192.168.123.10 (底层), 192.168.123.220 (高层)
    * - 本地端口：8091 (底层), 8090 (高层)
    * - 机器人端口：8007 (底层), 8082 (高层)
    */
    Custom()
        : 
        // 初始化底层UDP连接
        // 参数：控制级别，本地端口，目标IP，目标端口
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        
        // 初始化高层UDP连接
        // 参数：本地端口，目标IP，目标端口，发送数据大小，接收数据大小
        high_udp(8090, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        // 初始化命令数据结构为默认值
        high_udp.InitCmdData(high_cmd);  // 初始化高层命令数据
        low_udp.InitCmdData(low_cmd);    // 初始化底层命令数据
    }

    /**
    * @brief 发送高层UDP数据
    * 功能：将高层控制命令通过UDP发送给机器人
    * 调用频率：500Hz (每2ms调用一次)
    */
    void highUdpSend()
    {
        // printf("high udp send is running\n");  // 调试信息(已注释)

        high_udp.SetSend(high_cmd);  // 设置要发送的高层命令数据
        high_udp.Send();             // 执行UDP数据发送
    }

    /**
    * @brief 发送底层UDP数据  
    * 功能：将底层控制命令通过UDP发送给机器人
    * 调用频率：500Hz (每2ms调用一次)
    */
    void lowUdpSend()
    {
        low_udp.SetSend(low_cmd);   // 设置要发送的底层命令数据
        low_udp.Send();             // 执行UDP数据发送
    }

    /**
    * @brief 接收底层UDP数据
    * 功能：从机器人接收底层状态数据并更新到状态结构体
    * 调用频率：500Hz (每2ms调用一次)
    */
    void lowUdpRecv()
    {
        low_udp.Recv();                    // 接收UDP数据包
        low_udp.GetRecv(low_state);        // 解析数据到底层状态结构体
    }

    /**
    * @brief 接收高层UDP数据
    * 功能：从机器人接收高层状态数据并更新到状态结构体  
    * 调用频率：500Hz (每2ms调用一次)
    */
    void highUdpRecv()
    {
        // printf("high udp recv is running\n");  // 调试信息(已注释)

        high_udp.Recv();                   // 接收UDP数据包
        high_udp.GetRecv(high_state);      // 解析数据到高层状态结构体
    }
};

// =============================================================================
// 全局变量定义
// =============================================================================

Custom custom;  // 创建通信对象实例

// ROS订阅者 - 接收来自ROS应用的控制命令
ros::Subscriber sub_high;  // 高层命令订阅者
ros::Subscriber sub_low;   // 底层命令订阅者

// ROS发布者 - 向ROS应用发布机器人状态
ros::Publisher pub_high;   // 高层状态发布者
ros::Publisher pub_low;    // 底层状态发布者

// 计数器变量 - 用于调试和监控
long high_count = 0;       // 高层回调计数器
long low_count = 0;        // 底层回调计数器

// =============================================================================
// ROS回调函数定义
// =============================================================================

/**
* @brief 高层控制命令回调函数
* @param msg 接收到的高层控制命令消息
* 
* 此函数在每次接收到ROS高层控制命令时被调用，负责：
* 1. 将ROS消息转换为Unitree SDK格式
* 2. 获取最新的机器人状态
* 3. 将状态转换为ROS消息格式并发布
* 
* 数据流向：ROS应用 → 本回调函数 → UDP发送 → 机器人硬件
* 状态反馈：机器人硬件 → UDP接收 → 本回调函数 → ROS应用
*/
void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count++);

    // 将接收到的ROS高层命令消息转换为Unitree SDK命令格式
    custom.high_cmd = rosMsg2Cmd(*msg);

    // 创建高层状态消息用于发布
    unitree_legged_msgs::HighState high_state_ros;

    // 将当前机器人高层状态转换为ROS消息格式
    high_state_ros = state2rosMsg(custom.high_state);

    // 发布高层状态消息到ROS话题，供其他ROS节点使用
    pub_high.publish(high_state_ros);

    // printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);  // 调试信息(已注释)
}

/**
* @brief 底层控制命令回调函数
* @param msg 接收到的底层控制命令消息
* 
* 此函数在每次接收到ROS底层控制命令时被调用，负责：
* 1. 将ROS消息转换为Unitree SDK格式
* 2. 获取最新的机器人状态  
* 3. 将状态转换为ROS消息格式并发布
* 
* 数据流向：ROS应用 → 本回调函数 → UDP发送 → 机器人硬件
* 状态反馈：机器人硬件 → UDP接收 → 本回调函数 → ROS应用
*/
void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    printf("lowCmdCallback is running !\t%ld\n", ::low_count++);

    // 将接收到的ROS底层命令消息转换为Unitree SDK命令格式
    custom.low_cmd = rosMsg2Cmd(*msg);

    // 创建底层状态消息用于发布
    unitree_legged_msgs::LowState low_state_ros;

    // 将当前机器人底层状态转换为ROS消息格式
    low_state_ros = state2rosMsg(custom.low_state);

    // 发布底层状态消息到ROS话题，供其他ROS节点使用
    pub_low.publish(low_state_ros);

    // printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);  // 调试信息(已注释)
}

// =============================================================================
// 主函数
// =============================================================================

/**
* @brief 主函数 - ROS-UDP通信桥梁程序入口
* @param argc 命令行参数个数
* @param argv 命令行参数数组
* @return 程序退出状态码
* 
* 程序根据命令行参数决定运行模式：
* - HIGHLEVEL：高层控制模式，用于整体运动控制
* - LOWLEVEL：底层控制模式，用于精确关节控制
*/
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "ros_udp");
    ros::NodeHandle nh;

    // =============================================================================
    // 根据命令行参数选择控制模式
    // =============================================================================
    
    // 底层控制模式
    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        // 创建底层控制相关的ROS订阅者和发布者
        sub_low = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        // 创建UDP通信循环线程
        // 发送线程：每2ms执行一次UDP数据发送，优先级3
        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
        
        // 接收线程：每2ms执行一次UDP数据接收，优先级3  
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

        // 启动UDP通信线程
        loop_udpSend.start();  // 启动发送线程
        loop_udpRecv.start();  // 启动接收线程

        printf("LOWLEVEL is initialized\n");

        // 进入ROS消息循环，处理回调函数
        ros::spin();

        // printf("low level runing!\n");  // 调试信息(已注释)
    }
    // 高层控制模式
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        // 创建高层控制相关的ROS订阅者和发布者
        sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
        pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

        // 创建UDP通信循环线程
        // 发送线程：每2ms执行一次UDP数据发送，优先级3
        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        
        // 接收线程：每2ms执行一次UDP数据接收，优先级3
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        // 启动UDP通信线程
        loop_udpSend.start();  // 启动发送线程
        loop_udpRecv.start();  // 启动接收线程

        printf("HIGHLEVEL is initialized\n");

        // 进入ROS消息循环，处理回调函数
        ros::spin();

        // printf("high level runing!\n");  // 调试信息(已注释)
    }
    // 参数错误处理
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);  // 退出程序并返回错误代码
    }

    return 0;
}