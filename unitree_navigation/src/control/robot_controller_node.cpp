#include <csignal>
#include "unitree_navigation/robot_controller.h"


// 全局控制器指针，用于信号处理
RobotController *g_controller_ptr = nullptr;

// 信号处理函数
void signalHandler(int sig)
{
    ROS_INFO("关闭信号已接收，正在安全停止机器人...");
    if (g_controller_ptr)
    {
        g_controller_ptr->Stop();
    }
    ros::shutdown();
}

/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "robot_controller_node",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // 注册信号处理
    signal(SIGINT, signalHandler);

    // 创建控制器
    RobotController controller(nh);
    g_controller_ptr = &controller;

    // 初始化控制器
    if (!controller.Initialize())
    {
        ROS_ERROR("机器人控制器初始化失败");
        return -1;
    }

    // 启动控制器
    if (!controller.Start())
    {
        ROS_ERROR("机器人控制器启动失败");
        return -1;
    }

    ROS_INFO("机器人控制器节点已初始化");

    // 运行控制器主循环
    controller.run();

    // 清理
    g_controller_ptr = nullptr;

    return 0;
}