/**
* @file example_position.cpp
* @brief Unitree四足机器人底层位置控制示例程序
* 
* 本程序演示如何使用底层控制模式对机器人的关节进行精确的位置控制。
* 程序控制前右腿(FR)的三个关节，展示了从初始位置到目标位置的平滑过渡，
* 以及基于正弦函数的周期性运动控制。
* 
* ⚠️ 警告：底层控制模式需要特殊的安全措施
* 1. 按L2+A让机器人坐下
* 2. 按L1+L2+start进入底层控制模式  
* 3. 确保机器人悬挂固定，避免跌倒
* 
* @author Unitree Robotics
* @date 2018-2019
*/

#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>      // 底层控制命令消息
#include <unitree_legged_msgs/LowState.h>    // 底层状态反馈消息
#include "unitree_legged_sdk/unitree_legged_sdk.h"  // Unitree SDK
#include "convert.h"                         // ROS消息与SDK数据转换函数

using namespace UNITREE_LEGGED_SDK;

// 全局变量：存储底层状态反馈数据
unitree_legged_msgs::LowState low_state_ros;

/**
* @brief 底层状态回调函数
* @param state 接收到的底层状态消息
* 
* 此函数在每次接收到机器人状态反馈时被调用，用于更新全局状态变量。
* 状态信息包含20个电机的位置、速度、力矩等详细数据。
*/
void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &state)
{
    static long count = 0;
    ROS_INFO("lowStateCallback %ld", count++);

    // 保存最新的状态信息到全局变量
    low_state_ros = *state;
}

/**
* @brief 主函数 - 底层位置控制示例
*/
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "example_postition_without_lcm");

    // 显示安全警告信息
    std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();  // 等待用户确认

    // 创建ROS节点句柄和设置循环频率
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);  // 500Hz控制频率，确保实时性

    // =============================================================================
    // 控制参数初始化
    // =============================================================================
    
    long motiontime = 0;           // 运动时间计数器(单位：2ms)
    int rate_count = 0;            // 频率计数器(未使用)
    int sin_count = 0;             // 正弦计数器(未使用)
    
    // 关节位置数组 - 前右腿三个关节(髋关节、大腿、小腿)
    float qInit[3] = {0};          // 初始关节位置 (rad)
    float qDes[3] = {0};           // 目标关节位置 (rad)
    float sin_mid_q[3] = {0.0, 1.2, -2.0}; // 正弦运动的中心位置 (rad)
    
    // PID控制参数
    float Kp[3] = {0};             // 位置比例增益
    float Kd[3] = {0};             // 速度微分增益

    // 创建底层控制命令消息
    unitree_legged_msgs::LowCmd low_cmd_ros;

    // =============================================================================
    // ROS发布者和订阅者设置
    // =============================================================================
    
    // 创建发布者：发送底层控制命令到机器人
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
    
    // 创建订阅者：接收机器人底层状态反馈
    ros::Subscriber sub = nh.subscribe("low_state", 1, lowStateCallback);

    // =============================================================================
    // 底层控制命令初始化
    // =============================================================================
    
    // 设置数据包头标识(固定值，用于通信协议识别)
    low_cmd_ros.head[0] = 0xFE;
    low_cmd_ros.head[1] = 0xEF;
    
    // 设置控制级别标志为底层控制
    low_cmd_ros.levelFlag = LOWLEVEL;

    // 初始化所有20个电机的控制参数
    for (std::size_t i = 0; i < 20; i++)
    {
        low_cmd_ros.motorCmd[i].mode = 0x0A;   // 设置电机为伺服(PMSM)模式
        low_cmd_ros.motorCmd[i].q = PosStopF;  // 禁止位置环(初始状态)
        low_cmd_ros.motorCmd[i].Kp = 0;        // 位置增益设为0
        low_cmd_ros.motorCmd[i].dq = VelStopF; // 禁止速度环(初始状态)
        low_cmd_ros.motorCmd[i].Kd = 0;        // 速度增益设为0
        low_cmd_ros.motorCmd[i].tau = 0;       // 前馈力矩设为0
    }

    // =============================================================================
    // 主控制循环
    // =============================================================================
    
    while (ros::ok())
    {
        // 打印前右腿三个关节的当前位置信息(用于调试)
        printf("FR_joint_pos: %f %f %f\n", 
            low_state_ros.motorState[FR_0].q,    // 髋关节位置
            low_state_ros.motorState[FR_1].q,    // 大腿关节位置
            low_state_ros.motorState[FR_2].q);   // 小腿关节位置

        // 时间计数器递增(每个循环周期2ms)
        motiontime += 2;

        // =============================================================================
        // 阶段1：初始化阶段 (0-20ms)
        // 功能：记录机器人当前关节位置作为运动起始点
        // =============================================================================
        if(motiontime >= 0 && motiontime <= 10)
        {
            // 获取并保存当前关节位置作为初始位置
            qInit[0] = low_state_ros.motorState[FR_0].q;  // 髋关节初始位置
            qInit[1] = low_state_ros.motorState[FR_1].q;  // 大腿关节初始位置
            qInit[2] = low_state_ros.motorState[FR_2].q;  // 小腿关节初始位置
        }
        // =============================================================================
        // 阶段2：平滑过渡阶段 (20ms-5000ms)
        // 功能：从当前位置平滑过渡到目标位置，避免突变造成的冲击
        // =============================================================================
        else if(motiontime > 10 && motiontime < 2500)
        {
            // 设置PID控制增益
            Kp[0] = 20.0; Kp[1] = 20.0; Kp[2] = 20.0;  // 位置比例增益
            Kd[0] = 2.0;  Kd[1] = 2.0;  Kd[2] = 2.0;   // 速度微分增益

            // 计算过渡进度(0到1之间的插值系数)
            const float interval = 2000.0;  // 过渡时间间隔
            float phase = std::min(std::max((motiontime - 10) / interval, 0.0f), 1.0f);

            // 使用线性插值计算目标位置，实现平滑过渡
            // qDes = (1-phase)*qInit + phase*sin_mid_q
            qDes[0] = (1 - phase) * qInit[0] + phase * sin_mid_q[0];  // 髋关节目标位置
            qDes[1] = (1 - phase) * qInit[1] + phase * sin_mid_q[1];  // 大腿关节目标位置
            qDes[2] = (1 - phase) * qInit[2] + phase * sin_mid_q[2];  // 小腿关节目标位置
        }
        // =============================================================================
        // 阶段3：正弦周期运动阶段 (5000ms以后)
        // 功能：执行基于正弦函数的周期性运动，模拟腿部摆动
        // =============================================================================
        else if(motiontime >= 2500)
        {
            float period = 5.0;  // 运动周期：5秒

            // 计算正弦运动的目标位置
            qDes[0] = sin_mid_q[0];  // 髋关节保持在中心位置
            
            // 大腿关节：在中心位置基础上叠加正弦运动(幅度0.6rad)
            qDes[1] = sin_mid_q[1] + 0.6 * std::sin(2 * M_PI / period * (motiontime - 2500) / 1000.0);
            
            // 小腿关节：在中心位置基础上叠加反向正弦运动(幅度0.9rad)
            qDes[2] = sin_mid_q[2] + (-0.9) * std::sin(2 * M_PI / period * (motiontime - 2500) / 1000.0);
        }

        // =============================================================================
        // 设置前右腿髋关节(FR_0)控制参数
        // =============================================================================
        low_cmd_ros.motorCmd[FR_0].tau = -4.0;      // 前馈力矩(补偿重力等)
        low_cmd_ros.motorCmd[FR_0].Kp = Kp[0];      // 位置比例增益
        low_cmd_ros.motorCmd[FR_0].Kd = Kd[0];      // 速度微分增益
        low_cmd_ros.motorCmd[FR_0].q = qDes[0];     // 目标位置
        low_cmd_ros.motorCmd[FR_0].dq = 0.0;        // 目标速度(设为0，纯位置控制)

        // =============================================================================
        // 设置前右腿大腿关节(FR_1)控制参数
        // =============================================================================
        low_cmd_ros.motorCmd[FR_1].tau = 0.0;       // 前馈力矩
        low_cmd_ros.motorCmd[FR_1].Kp = Kp[1];      // 位置比例增益
        low_cmd_ros.motorCmd[FR_1].Kd = Kd[1];      // 速度微分增益
        low_cmd_ros.motorCmd[FR_1].q = qDes[1];     // 目标位置
        low_cmd_ros.motorCmd[FR_1].dq = 0.0;        // 目标速度

        // =============================================================================
        // 设置前右腿小腿关节(FR_2)控制参数
        // =============================================================================
        low_cmd_ros.motorCmd[FR_2].tau = 0.0;       // 前馈力矩
        low_cmd_ros.motorCmd[FR_2].Kp = Kp[2];      // 位置比例增益
        low_cmd_ros.motorCmd[FR_2].Kd = Kd[2];      // 速度微分增益
        low_cmd_ros.motorCmd[FR_2].q = qDes[2];     // 目标位置
        low_cmd_ros.motorCmd[FR_2].dq = 0.0;        // 目标速度

        // =============================================================================
        // 发布控制命令和处理ROS回调
        // =============================================================================
        
        // 发布底层控制命令到机器人
        pub.publish(low_cmd_ros);

        // 处理ROS回调函数(接收状态反馈)
        ros::spinOnce();
        
        // 按设定频率休眠，维持500Hz控制循环
        loop_rate.sleep();
    }

    return 0;
}