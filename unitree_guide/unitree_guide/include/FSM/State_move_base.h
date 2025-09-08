/**********************************************************************
 * @file State_move_base.h
 * @brief ROS导航集成状态类头文件
 * 
 * 这个文件定义了State_move_base类，它是Unitree四足机器人控制系统中
 * 专门用于与ROS导航栈集成的状态。该状态继承自State_Trotting，在
 * 保持原有小跑运动能力的基础上，增加了接收ROS标准导航命令的功能。
 * 
 * 主要功能：
 * 1. 接收ROS标准的geometry_msgs::Twist消息作为运动命令
 * 2. 与move_base节点配合实现自主导航功能
 * 3. 支持路径规划和动态避障
 * 4. 提供与ROS生态系统的无缝集成
 * 
 * 适用场景：
 * - SLAM建图和自主导航
 * - 与ROS Navigation Stack集成
 * - 远程遥控和自动巡逻
 * - 多机器人协作
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// 编译条件宏：只有在定义了COMPILE_WITH_MOVE_BASE时才编译此状态
// 这种设计允许在不需要ROS导航功能时排除相关代码，减小编译体积
#ifdef COMPILE_WITH_MOVE_BASE

#ifndef STATE_MOVE_BASE_H
#define STATE_MOVE_BASE_H

// ==================== 头文件包含 ====================

#include "FSM/State_Trotting.h"     // 继承自小跑状态，复用运动控制逻辑
#include "ros/ros.h"                // ROS核心功能，提供节点、话题等基础设施
#include <geometry_msgs/Twist.h>    // ROS标准运动消息类型，包含线速度和角速度

/**
 * @class State_move_base
 * @brief ROS导航集成状态类
 * 
 * 这个类继承自State_Trotting，是专门为与ROS导航系统集成而设计的状态。
 * 它在保持原有小跑运动控制能力的基础上，增加了ROS消息接收和处理功能。
 * 
 * 设计思路：
 * - 继承复用：继承State_Trotting的完整运动控制能力
 * - 命令重载：重写getUserCmd()方法，从ROS话题获取运动命令
 * - 无缝集成：保持与现有状态机框架的完全兼容
 * 
 * 与State_Trotting的区别：
 * - State_Trotting：从手柄或键盘获取用户命令
 * - State_move_base：从ROS /cmd_vel话题获取导航命令
 * 
 * 工作流程：
 * 1. 订阅ROS /cmd_vel话题
 * 2. 在回调函数中解析Twist消息
 * 3. 将线速度和角速度存储到成员变量
 * 4. 在控制循环中应用这些速度命令
 * 5. 利用继承的运动控制逻辑执行实际运动
 */
class State_move_base : public State_Trotting{
public:
    /**
     * @brief 构造函数
     * @param ctrlComp 控制组件指针，包含所有控制相关的数据和接口
     * 
     * 构造函数的主要任务：
     * 1. 调用父类State_Trotting的构造函数
     * 2. 设置状态名称为MOVE_BASE
     * 3. 初始化ROS相关组件（节点句柄、订阅器）
     * 4. 配置消息接收机制
     */
    State_move_base(CtrlComponents *ctrlComp);
    
    /**
     * @brief 析构函数
     * 
     * 采用默认析构函数，因为：
     * 1. ROS相关对象会自动清理
     * 2. 父类析构函数会处理基础清理工作
     * 3. 无需额外的资源释放操作
     */
    ~State_move_base(){}
    
    /**
     * @brief 检查状态切换条件
     * @return FSMStateName 下一个状态的名称
     * 
     * 这个函数定义了从MOVE_BASE状态可以切换到的其他状态。
     * 保持与State_Trotting相同的切换逻辑：
     * 
     * 支持的状态转换：
     * - L2_B按钮 → PASSIVE状态（紧急停止，安全第一）
     * - L2_A按钮 → FIXEDSTAND状态（停止导航，站立等待）
     * - 其他情况 → 保持MOVE_BASE状态（继续导航运动）
     * 
     * 注意：即使在自动导航模式下，安全停止按钮仍然有效
     */
    FSMStateName checkChange();

private:
    // ==================== ROS消息处理方法 ====================
    
    /**
     * @brief 获取用户运动命令（重写父类方法）
     * 
     * 这是对父类State_Trotting::getUserCmd()的重写，改变了命令来源：
     * - 父类版本：从手柄或键盘获取命令
     * - 本类版本：从ROS话题获取导航命令
     * 
     * 主要功能：
     * 1. 将当前存储的速度命令(_vx, _vy, _wz)传递给运动控制系统
     * 2. 调用ros::spinOnce()处理ROS消息队列
     * 3. 确保ROS回调函数能够及时执行
     * 
     * 调用时机：每个控制周期(500Hz)都会调用
     */
    void getUserCmd();
    
    /**
     * @brief 初始化ROS消息接收机制
     * 
     * 设置ROS订阅器，建立与导航系统的通信链路：
     * 1. 创建/cmd_vel话题的订阅器
     * 2. 设置消息队列长度为1（保持最新命令）
     * 3. 绑定回调函数twistCallback
     * 4. 配置消息处理参数
     * 
     * /cmd_vel话题说明：
     * - 这是ROS导航系统的标准话题名称
     * - 由move_base、navigation stack等节点发布
     * - 消息类型为geometry_msgs::Twist
     * - 包含期望的线速度和角速度信息
     */
    void initRecv();
    
    /**
     * @brief ROS Twist消息回调函数
     * @param msg 接收到的Twist消息，包含导航系统发送的运动命令
     * 
     * 这是ROS消息系统的回调函数，当/cmd_vel话题收到新消息时自动调用。
     * 
     * 消息解析：
     * - msg.linear.x：前进/后退速度 (m/s)，正值向前，负值向后
     * - msg.linear.y：左右平移速度 (m/s)，正值向左，负值向右
     * - msg.angular.z：偏航角速度 (rad/s)，正值逆时针，负值顺时针
     * 
     * 注意事项：
     * 1. 忽略linear.z（垂直速度），因为地面机器人不需要
     * 2. 忽略angular.x和angular.y（俯仰和横滚角速度）
     * 3. 速度值需要在合理范围内，避免机器人运动过快
     * 4. 回调函数应保持简洁，避免阻塞ROS消息处理
     */
    void twistCallback(const geometry_msgs::Twist& msg);

    // ==================== ROS相关成员变量 ====================
    
    /**
     * @brief ROS节点句柄
     * 
     * NodeHandle是ROS中与ROS Master通信的接口，提供以下功能：
     * 1. 创建订阅器和发布器
     * 2. 管理话题和服务
     * 3. 处理参数服务器访问
     * 4. 控制节点生命周期
     * 
     * 在本类中主要用于创建/cmd_vel话题的订阅器
     */
    ros::NodeHandle _nm;
    
    /**
     * @brief ROS消息订阅器
     * 
     * 订阅/cmd_vel话题的订阅器对象，负责：
     * 1. 监听指定话题的消息
     * 2. 在收到消息时调用回调函数
     * 3. 管理消息队列和缓冲
     * 4. 处理网络通信和序列化
     * 
     * 配置参数：
     * - 话题名："/cmd_vel"
     * - 队列长度：1（只保留最新消息）
     * - 回调函数：twistCallback
     */
    ros::Subscriber _cmdSub;

    // ==================== 运动命令存储变量 ====================
    
    /**
     * @brief X方向线速度命令 (m/s)
     * 
     * 存储从ROS消息中解析出的前进/后退速度：
     * - 正值：机器人向前运动
     * - 负值：机器人向后运动
     * - 零值：在X方向保持静止
     * 
     * 取值范围：建议限制在[-2.0, 2.0] m/s以内，确保运动安全
     */
    double _vx;
    
    /**
     * @brief Y方向线速度命令 (m/s)
     * 
     * 存储从ROS消息中解析出的左右平移速度：
     * - 正值：机器人向左侧移动
     * - 负值：机器人向右侧移动
     * - 零值：在Y方向保持静止
     * 
     * 注意：并非所有四足机器人都支持侧向移动，
     * 具体能力取决于步态生成器的实现
     */
    double _vy;
    
    /**
     * @brief Z轴角速度命令 (rad/s)
     * 
     * 存储从ROS消息中解析出的偏航角速度（转向速度）：
     * - 正值：机器人逆时针旋转（左转）
     * - 负值：机器人顺时针旋转（右转）
     * - 零值：保持当前朝向
     * 
     * 取值范围：建议限制在[-2.0, 2.0] rad/s以内
     */
    double _wz;
};

#endif  // STATE_MOVE_BASE_H

#endif  // COMPILE_WITH_MOVE_BASE