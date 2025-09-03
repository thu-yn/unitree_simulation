#ifndef UNITREE_NAVIGATION_ROBOT_CONTROLLER_H
#define UNITREE_NAVIGATION_ROBOT_CONTROLLER_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <memory>
#include <string>
#include <ros/ros.h>

// 引入Unitree SDK头文件
#include "/home/yangnan/unitree_go2_catkin_ws/src/unitree_simulation/unitree_ros_to_real/unitree_legged_real/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

/**
 * @brief 机器人控制器类，用于接收ROS速度命令并控制Unitree四足机器人
 */
class RobotController {
public:
    /**
     * @brief 构造函数
     */
    RobotController(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~RobotController();
    
    /**
     * @brief 初始化控制器
     * @return 是否成功初始化
     */
    bool Initialize();
    
    /**
     * @brief 启动控制器
     * @return 是否成功启动
     */
    bool Start();
    
    /**
     * @brief 停止控制器
     */
    void Stop();
    
    /**
     * @brief 设置机器人速度
     * @param cmd_vel 速度命令
     */
    void SetVelocity(const geometry_msgs::Twist& cmd_vel);

    void run();

    /**
     * @brief 更新机器人状态
     */
    void UpdateRobotState();

private:
    // 机器人状态枚举
    enum class RobotState {
        UNKNOWN,
        STANDING,
        SITTING,
        WALKING
    };
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher joint_states_pub_;
    ros::ServiceServer stand_service_;
    ros::ServiceServer sit_service_;
    ros::Time last_cmd_time_;

    // 参数
    std::string robot_ip_;
    double control_frequency_;
    double max_linear_speed_;
    double max_angular_speed_;
    
    // 消息
    nav_msgs::Odometry odom_msg_;
    sensor_msgs::JointState joint_state_msg_;
    
    // TF广播
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Unitree SDK相关
    std::unique_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
    UNITREE_LEGGED_SDK::HighCmd high_cmd_;
    UNITREE_LEGGED_SDK::HighState high_state_;
    std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> udp_send_thread_;
    std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> udp_recv_thread_;
    std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> control_thread_;
    
    // 机器人状态
    RobotState robot_state_;
    
    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool standService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool sitService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void InitRobot();
    void RobotControl();
    // UDP通信
    void InitUDPConnection();
    void CloseUDPConnection();
    void UDPSend();
    void UDPRecv();
    
    // 状态更新和发布
    void updateRobotState();
    void publishTransforms();


};

#endif // UNITREE_NAVIGATION_ROBOT_CONTROLLER_H