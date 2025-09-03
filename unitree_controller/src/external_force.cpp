/************************************************************************
* 文件名: external_force.cpp
* 功能描述: Unitree四足机器人外力扰动测试工具
* 
* 主要功能:
* 1. 🟡 测试工具 - 通过键盘控制施加外力扰动
* 2. 抗扰动测试 - 测试机器人的平衡控制能力
* 3. 两种控制模式 - 脉冲模式和连续模式
* 4. 实时力反馈 - 显示当前施加的力大小和方向
* 
* 控制模式说明:
* - 脉冲模式(默认): 按键触发瞬时力，持续100ms后自动归零
* - 连续模式: 按键施加持续力，需要手动调整或归零
* 
* 在整个项目中的作用:
* - 🟡 开发调试工具 - 用于测试机器人控制算法的鲁棒性
* - 验证平衡控制效果，模拟真实环境中的外部扰动
* - 帮助调试PD控制器参数和步态规划算法
* 
* 使用方法:
* 1. 先启动Gazebo仿真和机器人控制器
* 2. 运行此程序: rosrun unitree_controller unitree_external_force
* 3. 使用键盘控制施加不同方向的外力
* 
* 键盘操作:
* - 上/下方向键: 施加前后方向的力(±Fx)
* - 左/右方向键: 施加左右方向的力(±Fy)  
* - 空格键: 切换脉冲/连续模式
* 
* Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
* Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>         // 力和力矩消息类型
#include <signal.h>                       // 信号处理
#include <termios.h>                      // 终端控制
#include <stdio.h>

// ==================== 键盘按键码定义 ====================

/**
* 键盘按键的ASCII码定义
* 用于识别用户的键盘输入
*/
#define KEYCODE_UP    0x41    // 上方向键 - 施加向前的力(+Fx)
#define KEYCODE_DOWN  0x42    // 下方向键 - 施加向后的力(-Fx)
#define KEYCODE_LEFT  0x44    // 左方向键 - 施加向左的力(+Fy)
#define KEYCODE_RIGHT 0x43    // 右方向键 - 施加向右的力(-Fy)
#define KEYCODE_SPACE 0x20    // 空格键 - 切换控制模式

// ==================== 全局控制模式变量 ====================

/**
* @brief 控制模式标志
* 
* 值的含义:
* - mode > 0 (默认1): 脉冲模式 - 按键触发瞬时力，100ms后自动归零
* - mode < 0 (设为-1): 连续模式 - 按键施加持续力，需要手动控制
* 
* 模式特点:
* - 脉冲模式: 适合模拟踢、推等瞬时扰动，更安全
* - 连续模式: 适合模拟持续的外力，如风力、斜坡等
*/
int mode = 1; // pulsed mode or continuous mode

/**
* @class teleForceCmd
* @brief 远程力控制命令类
* 
* 功能描述:
* - 管理键盘输入和力的施加
* - 处理两种不同的力控制模式
* - 实时显示力的状态信息
* - 通过ROS话题发布力命令到Gazebo
*/
class teleForceCmd
{
public:
    /**
    * @brief 构造函数 - 初始化力控制系统
    */
    teleForceCmd();
    
    /**
    * @brief 键盘监听主循环
    * 
    * 功能描述:
    * - 持续监听键盘输入
    * - 解析按键并转换为力控制命令
    * - 根据当前模式执行相应的力控制逻辑
    */
    void keyLoop();
    
    /**
    * @brief 发布力命令函数
    * 
    * @param x X轴方向的力(N) - 前后方向
    * @param y Y轴方向的力(N) - 左右方向  
    * @param z Z轴方向的力(N) - 上下方向(目前未使用)
    * 
    * 功能描述:
    * - 将力参数封装为ROS消息
    * - 通过ROS话题发布到Gazebo仿真环境
    * - 触发机器人本体受到相应的外力
    */
    void pubForce(double x, double y, double z);

private:
    // ==================== 私有成员变量 ====================
    
    double Fx, Fy, Fz;                    // 当前施加的三轴力(N)
    ros::NodeHandle n;                    // ROS节点句柄
    ros::Publisher force_pub;             // 力发布器
    geometry_msgs::Wrench Force;         // 力和力矩消息
};

/**
* @brief teleForceCmd类构造函数实现
* 
* 功能描述:
* - 初始化所有力分量为0
* - 创建力发布器，连接到Gazebo的力施加话题
* - 等待1秒确保发布器连接成功
* - 发布初始的零力状态
*/
teleForceCmd::teleForceCmd()
{
    // 初始化所有力分量为0
    Fx = 0;
    Fy = 0;
    Fz = 0;
    
    /**
    * 创建力发布器
    * 话题: /apply_force/trunk
    * 消息类型: geometry_msgs::Wrench
    * 队列大小: 20
    * 
    * 功能: 向Gazebo中的机器人躯干施加外力
    */
    force_pub = n.advertise<geometry_msgs::Wrench>("/apply_force/trunk", 20);
    
    // 等待1秒确保发布器初始化完成
    sleep(1);
    
    // 发布初始零力状态，确保系统干净启动
    pubForce(Fx, Fy, Fz);
}

// ==================== 全局变量和信号处理 ====================

int kfd = 0;                      // 键盘文件描述符
struct termios cooked, raw;       // 终端设置结构体

/**
* @brief 程序退出信号处理函数
* 
* @param sig 信号类型
* 
* 功能描述:
* - 恢复终端原始设置
* - 优雅地关闭ROS节点
* - 安全退出程序
*/
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);  // 恢复终端设置
    ros::shutdown();                    // 关闭ROS节点
    exit(0);                           // 退出程序
}

// ==================== 主函数 ====================

/**
* @brief 主函数 - 外力控制程序入口
* 
* 执行流程:
* 1. 初始化ROS节点
* 2. 创建远程力控制对象
* 3. 注册信号处理函数
* 4. 进入键盘监听循环
*/
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "external_force");
    
    // 创建远程力控制对象
    teleForceCmd remote;
    
    // 注册Ctrl+C信号处理函数，确保优雅退出
    signal(SIGINT, quit);
    
    // 进入键盘监听主循环
    remote.keyLoop();
    
    return(0);
}

/**
* @brief 发布力命令函数实现
* 
* @param x X轴方向的力
* @param y Y轴方向的力
* @param z Z轴方向的力
* 
* 功能描述:
* - 将力参数设置到Wrench消息中
* - 通过ROS发布器发送到Gazebo
* - 处理ROS消息队列
*/
void teleForceCmd::pubForce(double x, double y, double z)
{
    // 设置力分量到消息结构体
    Force.force.x = Fx;
    Force.force.y = Fy;
    Force.force.z = Fz;
    
    // 发布力消息到Gazebo
    force_pub.publish(Force);
    
    // 处理ROS消息队列
    ros::spinOnce();
}

/**
* @brief 键盘监听主循环函数实现
* 
* 功能描述:
* - 设置终端为原始模式，能够捕获单个按键
* - 持续监听键盘输入
* - 根据按键和当前模式执行相应的力控制
* - 实时显示力状态信息
*/
void teleForceCmd::keyLoop()
{
    char c;
    bool dirty = false;  // 标记是否需要更新力状态
    
    // ========== 终端设置 - 进入原始模式 ==========
    
    // 获取当前终端设置
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    
    // 设置为原始模式：关闭行缓冲和回显
    raw.c_lflag &=~ (ICANON | ECHO);
    
    // 设置特殊字符
    raw.c_cc[VEOL] = 1;  // 行结束字符
    raw.c_cc[VEOF] = 2;  // 文件结束字符
    
    // 应用新的终端设置
    tcsetattr(kfd, TCSANOW, &raw);
    
    // ========== 用户界面提示信息 ==========
    
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'Space' to change mode, default is Pulsed mode:");
    puts("Use 'Up/Down/Left/Right' to change direction");
    
    // ========== 主键盘监听循环 ==========
    
    for(;;){
        // 读取键盘输入
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }
        
        // 调试信息：显示按键的十六进制值
        ROS_DEBUG("value: 0x%02X\n", c);
        
        // ========== 按键处理逻辑 ==========
        
        switch(c){
        case KEYCODE_UP:    // 上方向键 - 向前推力
            if(mode > 0) {
                // 脉冲模式：固定60N的前向力
                Fx = 60;
            } else {
                // 连续模式：递增16N，最大220N
                Fx += 16;
                if(Fx > 220) Fx = 220;   // 正向限幅
                if(Fx < -220) Fx = -220; // 负向限幅
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
            
        case KEYCODE_DOWN:  // 下方向键 - 向后推力
            if(mode > 0) {
                // 脉冲模式：固定60N的后向力
                Fx = -60;
            } else {
                // 连续模式：递减16N
                Fx -= 16;
                if(Fx > 220) Fx = 220;
                if(Fx < -220) Fx = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
            
        case KEYCODE_LEFT:  // 左方向键 - 向左推力
            if(mode > 0) {
                // 脉冲模式：固定30N的左向力
                Fy = 30;
            } else {
                // 连续模式：递增8N
                Fy += 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
            
        case KEYCODE_RIGHT: // 右方向键 - 向右推力
            if(mode > 0) {
                // 脉冲模式：固定30N的右向力
                Fy = -30;
            } else {
                // 连续模式：递减8N
                Fy -= 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
            
        case KEYCODE_SPACE: // 空格键 - 切换控制模式
            mode = mode * (-1);  // 在1和-1之间切换
            
            // 显示当前模式
            if(mode > 0){
                ROS_INFO("Change to Pulsed mode.");
            } else {
                ROS_INFO("Change to Continuous mode.");
            }
            
            // 切换模式时清零所有力
            Fx = 0;
            Fy = 0;
            Fz = 0;            
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        }
        
        // ========== 力状态更新和发布 ==========
        
        if(dirty == true){
            // 发布当前力状态
            pubForce(Fx, Fy, Fz);
            
            // 如果是脉冲模式，100ms后自动归零
            if(mode > 0){
                usleep(100000); // 等待100ms
                
                // 清零所有力
                Fx = 0;
                Fy = 0;
                Fz = 0;
                
                // 发布零力状态
                pubForce(Fx, Fy, Fz);
            }
            
            dirty = false;  // 重置更新标志
        }
    }
    return;
}