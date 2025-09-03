/*
* Unitree电机SDK - C语言完整控制示例
* 文件名: check.c
* 功能: 演示电机的多种控制模式（力矩/位置/速度），包含过热保护机制
* 说明: 这是包含所有功能和重要教程的完整控制示例
*/

#include <stdio.h>
#include <errno.h>          // 错误码定义
#include <string.h>         // 字符串操作函数
#include <unistd.h>         // Unix标准函数定义，包含usleep()等
#include <sys/time.h>       // 时间相关函数
#include "LSerial.h"        // 串口通信函数声明
#include "motor_ctrl.h"     // 电机控制结构体和函数声明

/*
* 电机控制模式枚举定义
* 用于选择不同的控制策略
*/
enum motor_command_type
{
    TORQUE,                 // 力矩控制模式
    POSITION,               // 位置控制模式
    VELOCITY                // 速度控制模式
};

/**
* @brief 电机多模式控制演示程序主函数
* @return int 程序执行结果，0表示成功，1表示参数错误
* @note 演示三种控制模式：力矩控制、位置控制、速度控制
*/
int main()
{
    /* 控制模式选择和参数定义 */
    int com_type = VELOCITY;            // 当前选择的控制模式（默认速度控制）
    float goal_T;                       // 目标力矩 (Nm)
    float goal_W;                       // 目标速度 (rad/s)
    float goal_Pos;                     // 目标位置 (rad)
    float goal_K_W;                     // 速度刚度系数
    float goal_K_P;                     // 位置刚度系数

    /* 根据控制模式设置相应的控制参数 */
    if(com_type == TORQUE)              // 力矩控制模式设置
    {
        goal_T = 1;                     // 目标力矩: 1 Nm
        goal_W = 0;                     // 速度期望: 0（不使用速度控制）
        goal_Pos = 0;                   // 位置期望: 0（不使用位置控制）
        goal_K_W = 0;                   // 速度刚度: 0（不使用速度反馈）
        goal_K_P = 0;                   // 位置刚度: 0（不使用位置反馈）
    }
    else if(com_type == POSITION)       // 位置控制模式设置
    {
        goal_T = 0;                     // 前馈力矩: 0（不使用力矩前馈）
        goal_W = 0;                     // 速度期望: 0（必须为0）
        goal_Pos = 0;                   // 目标位置: 0 rad
        goal_K_W = 3;                   // 速度刚度: 3（作为阻尼系数）
        goal_K_P = 0.1;                 // 位置刚度: 0.1（建议值）
    }
    else if(com_type == VELOCITY)       // 速度控制模式设置
    {
        goal_T = 0;                     // 前馈力矩: 0（不使用力矩前馈）
        goal_W = 50.0;                  // 目标速度: 50.0 rad/s
        goal_Pos = 0;                   // 位置期望: 0（不使用位置控制）
        goal_K_W = 3;                   // 速度刚度: 3（速度控制增益）
        goal_K_P = 0;                   // 位置刚度: 0（不使用位置反馈）
    }
    else                                // 错误的控制模式
    {
        printf("motor command type error!\n");
        return 1;
    }

    /* 电机控制结构体初始化 */
    MOTOR_send motor_s, motor_s1;       // 发送命令结构体（运行命令和停止命令）
    MOTOR_recv motor_r;                 // 接收状态结构体
    
    /* 运行控制参数配置 (motor_s) */
    motor_s.id = 0;                     // 电机ID: 0表示单个电机
    motor_s.mode = 10;                  // 控制模式: 10=闭环伺服控制模式
    motor_s.T = goal_T;                 // 期望力矩 (范围: T < 255.9 Nm)
    motor_s.W = goal_W;                 // 期望速度 (范围: W < 511.9 rad/s)
    motor_s.Pos = goal_Pos;             // 期望位置 (范围: Pos < 131071.9 rad)
    motor_s.K_P = goal_K_P;             // 位置刚度系数 (范围: K_P < 31.9，建议值约0.1)
    motor_s.K_W = goal_K_W;             // 速度刚度系数 (范围: K_W < 63.9，建议值约3)

    /* 停止控制参数配置 (motor_s1) */
    motor_s1.id = 0;                    // 电机ID: 0表示单个电机
    motor_s1.mode = 0;                  // 控制模式: 0=空闲模式（停止）
    
    /* 过热保护参数定义 */
    const float safe_torque = 0.7;                 // 安全力矩阈值: 0.7 Nm
    const float cooldown_torque = 0.5;             // 冷却期力矩限制: 0.5 Nm
    const long long overload_duration = 70000000;  // 过载持续时间: 70秒（微秒）
    const long long cooldown_duration = 600000000; // 冷却持续时间: 600秒（微秒）
    float safe_ratio = 1;               // 安全系数比例
    long long current = 0;              // 当前时间戳
    long long overload_start = 0;       // 过载开始时间
    long long cooldown_start = 0;       // 冷却开始时间
    
    /* 电机状态枚举定义 */
    enum motor_status
    {
        NORMAL,                         // 正常运行状态
        OVERHEAT,                       // 过热状态
        COOLDOWN                        // 冷却状态
    };

printf("87\n");        // 调试信息：程序执行到第87行

    /* 跨平台串口初始化 */
#if defined(__linux__)
    int fd;                             // Linux平台文件描述符
    /**
    * @brief 打开Linux串口设备
    * @note 使用/dev/ttyUSB0，根据实际情况可能需要调整设备名
    */
    fd = open_set("/dev/ttyUSB0");
#elif defined(__WIN32__)
    HANDLE fd;                          // Windows平台句柄
    /**
    * @brief 打开Windows串口设备
    * @note 使用COM4端口，根据实际情况可能需要调整端口号
    */
    fd = open_set("\\\\.\\COM4");
#endif

printf("95\n");        // 调试信息：程序执行到第95行
    
    int status = NORMAL;                // 初始化保护状态为正常

    /**
    * @brief 数据格式转换和预处理
    * @note modify_data()将用户友好的浮点数参数转换为底层通信格式
    */
    modify_data(&motor_s);              // 转换运行命令数据格式
    modify_data(&motor_s1);             // 转换停止命令数据格式

    /**
    * @brief 电机启动握手流程
    * @note 先发送运行命令建立通信，再发送停止命令确保安全状态
    */
    send_recv(fd, &motor_s, &motor_r);  // 发送运行命令（建立通信）
    send_recv(fd, &motor_s1, &motor_r); // 发送停止命令（确保安全）

printf("102\n");       // 调试信息：程序执行到第102行
    
    /**
    * @brief 解析电机状态反馈
    * @note extract_data()将底层数据格式转换为用户友好的浮点数
    */
    extract_data(&motor_r);
    printf("START\n");

    /* 主控制循环 - 包含过热保护机制 */
    for(int i=0; i<500; i++)            // 执行500次控制循环
    {
        /* 重置控制参数到目标值 */
        motor_s.T = goal_T;
        motor_s.K_P = goal_K_P;
        motor_s.K_W = goal_K_W;
        modify_data(&motor_s);

        /* 过热保护状态机逻辑 */
        if(status == NORMAL)             // 正常状态处理
        {
            /**
            * @brief 监控力矩是否超过安全阈值
            * @note 实时检测电机输出力矩，预防过热损坏
            */
            if(motor_r.T > safe_torque)
            {
                overload_start = getSystemTime();  // 记录过热开始时间
                status = OVERHEAT;                  // 切换到过热状态
            }
        }
        else if(status == OVERHEAT)      // 过热状态处理
        {
            current = getSystemTime();
            
            /**
            * @brief 检查力矩是否恢复到安全范围
            */
            if(motor_r.T < safe_torque)
            {
                status = NORMAL;        // 力矩恢复正常，切换到正常状态
            }
            /**
            * @brief 检查过热持续时间是否过长
            * @note 如果过热时间超过阈值，强制进入冷却模式
            */
            else if((current - overload_start) > overload_duration)
            {
                cooldown_start = getSystemTime();  // 记录冷却开始时间
                status = COOLDOWN;                  // 强制进入冷却状态
            }
        }
        else if(status == COOLDOWN)      // 冷却状态处理
        {
            current = getSystemTime();
            
            /**
            * @brief 检查冷却时间是否足够
            */
            if((current - cooldown_start) > cooldown_duration)
            {
                status = NORMAL;        // 冷却完成，恢复正常状态
            }
            else
            {
                /**
                * @brief 在冷却期间降低控制参数
                * @note 按比例降低所有控制参数，减少电机负载
                */
                safe_ratio = cooldown_torque / motor_r.T;
                motor_s.T = safe_ratio * goal_T;
                motor_s.K_P = safe_ratio * goal_K_P;
                motor_s.K_W = safe_ratio * goal_K_W;
                modify_data(&motor_s);
            }
        }
        
        /* 状态输出和控制命令发送 */
        printf("******************\n");
        printf("Torque command: %f\n", motor_s.T);
        
        /**
        * @brief 发送控制命令并获取电机状态反馈
        * @note 这是实时控制的核心，保持与电机的双向通信
        */
        send_recv(fd, &motor_s, &motor_r);
        extract_data(&motor_r);

        /* 状态监控和数据输出 */
        printf("status: %d\n", status);
        // printf("pos: %f\n", motor_r.Pos);    // 位置反馈（已注释）
        printf("w: %f\n", motor_r.LW);         // 实际转速（低速高精度）
        printf("T: %f\n", motor_r.T);          // 实际力矩
        
        usleep(500000);                        // 暂停0.5秒（控制频率2Hz）
    }

    /**
    * @brief 安全停机流程
    * @note 发送停止命令，确保电机安全停止
    */
    send_recv(fd, &motor_s1, &motor_r);
    extract_data(&motor_r);
    // show_resv_data_hex(&motor_r);           // 显示十六进制数据（已注释）
    printf("END\n");
    printf("ID: %d\n", motor_r.motor_id);
    // show_resv_data(&motor_r);               // 显示接收数据（已注释）

    /**
    * @brief 关闭串口连接，释放系统资源
    * @note 良好的编程习惯：及时释放系统资源
    */
    close_serial(fd);
    
#if defined(__WIN32__)
    system("pause");                           // Windows下暂停以查看结果
#endif
    
    return 0;
}

/*
* 程序功能总结：
* 
* 1. 多控制模式支持：
*    - TORQUE: 纯力矩控制（1 Nm），适用于力控应用
*    - POSITION: 位置控制（目标0 rad），适用于精确定位
*    - VELOCITY: 速度控制（50 rad/s），适用于恒速运行
* 
* 2. 智能保护机制：
*    - 实时力矩监控（安全阈值0.7 Nm）
*    - 三状态保护：NORMAL → OVERHEAT → COOLDOWN
*    - 过载持续时间检测（70秒）
*    - 强制冷却机制（600秒）
*    - 安全参数自适应调整
* 
* 3. 控制特性：
*    - 闭环伺服控制（mode=10）
*    - 500次控制循环
*    - 2Hz控制频率
*    - 实时状态反馈
* 
* 4. 工程实践：
*    - 跨平台兼容性（Linux/Windows）
*    - 完善的初始化和停止流程
*    - 调试信息输出
*    - 资源管理和清理
*/