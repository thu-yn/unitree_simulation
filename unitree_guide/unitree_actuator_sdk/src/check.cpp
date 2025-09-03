/*
* Unitree四足机器人电机控制SDK测试程序
* 
* 功能描述：
* 本程序是Unitree执行器SDK中的C++示例程序，用于演示如何通过串口与电机控制板进行通信。
* 程序会启动电机，让其以指定参数运行一段时间，然后停止电机。
* 
* 主要流程：
* 1. 初始化串口通信
* 2. 配置电机运行参数（模式5：速度控制模式）
* 3. 发送停止命令确保电机初始状态安全
* 4. 循环发送运行命令让电机运转
* 5. 发送停止命令结束运行
* 6. 关闭串口连接
*/

#include <stdio.h>
#include <errno.h>      //错误定义
#include <string.h>
#include <unistd.h>     //Unix标准函数定义, usleep()
#include <sys/time.h> 
#include "LSerial.h"    //串口通信函数
#include "motor_ctrl.h" //声明发送数据、接收数据的结构体，以及函数声明

int main()
{
    // ========== 电机控制参数定义 ==========
    
    //发送参数 - 定义两个电机命令结构体
    MOTOR_send motor_s, motor_s1;   // motor_s用于运行命令，motor_s1用于停止命令
    // long long start_time = 0;   // 预留的时间戳变量（未使用）
    
    // 配置电机运行参数（motor_s - 用于让电机运转）
    motor_s.id = 0;           // 电机ID，0表示目标电机
    motor_s.mode = 5;         // 控制模式：5 = 速度控制模式
    motor_s.T = 0;            // 目标力矩，单位：Nm, T<255.9（此处设为0，主要使用速度控制）
    motor_s.W = 21.0;         // 目标角速度，单位：rad/s, W<511.9（设置电机转速为21 rad/s）
    motor_s.Pos = 0.0;        // 目标位置，单位：rad, Pos<131071.9（速度模式下此参数无效）
    motor_s.K_P = 0.0;        // 位置环比例增益，K_P<31.9（速度模式下设为0）
    motor_s.K_W = 10;         // 速度环比例增益，K_W<63.9（用于速度控制的PID参数）

    // 配置电机停止参数（motor_s1 - 用于停止电机）
    motor_s1.id = 0;          // 电机ID，与运行命令相同
    motor_s1.mode = 0;        // 控制模式：0 = 停止模式/空闲模式
    // 其他参数使用默认值（结构体初始化时自动设为0）
    
    //接收参数 - 用于存储电机反馈的状态信息
    MOTOR_recv motor_r;       // 包含电机当前位置、速度、力矩等状态信息
    
    // ========== 串口连接初始化 ==========
    
    //文件描述符/句柄定义（根据操作系统选择不同类型）
#if defined(__linux__)
    int fd;                   // Linux系统使用整型文件描述符
    
#elif defined(__WIN32__)
    HANDLE fd;                // Windows系统使用HANDLE句柄
#endif

    // 打开并配置串口连接
    fd = open_set((char*)"/dev/ttyUSB0");  // Linux下的USB转串口设备路径
    // fd = open_set((char*)"\\\\.\\COM4"); // Windows下的串口路径（注释掉）
    
    // ========== 数据预处理 ==========
    
    // 对发送数据进行编码处理（将浮点数等转换为电机通信协议格式）
    modify_data(&motor_s);    // 处理运行命令数据
    modify_data(&motor_s1);   // 处理停止命令数据

    // ========== 电机控制序列 ==========
    
    int sta;  // 用于存储通信状态的变量
    
    // 第一步：发送停止命令，确保电机初始状态安全
    sta = send_recv(fd, &motor_s1, &motor_r);  // 发送停止命令并接收反馈
    // printf("status2: %d\n", sta);           // 打印通信状态（注释掉）
    extract_data(&motor_r);                    // 解码接收到的电机状态数据
    // show_resv_data_hex(&motor_r);           // 显示十六进制格式的反馈数据（注释掉）
    printf("START\n");                         // 打印程序开始标志
    // show_resv_data(&motor_r);               // 显示解码后的电机状态（注释掉）

    // 第二步：循环发送运行命令，让电机持续运转
    for(int i=0; i<100; i++)  // 执行100次控制循环
    {
        // 发送运行命令到电机，并接收当前状态反馈
        send_recv(fd, &motor_s, &motor_r);     // 发送motor_s（速度控制命令）
        // extract_data(&motor_r);             // 解码状态数据（注释掉，提高执行效率）
        // show_resv_data(&motor_r);           // 显示状态信息（注释掉）
        usleep(100000);                       // 延时100ms（100,000微秒），控制命令发送频率
    }

    // 第三步：发送停止命令，安全停止电机运行
    sta = send_recv(fd, &motor_s1, &motor_r);  // 再次发送停止命令
    // printf("status2: %d\n", sta);           // 打印最终通信状态（注释掉）
    extract_data(&motor_r);                    // 解码最终的电机状态
    // show_resv_data_hex(&motor_r);           // 显示十六进制数据（注释掉）
    printf("END\n");                           // 打印程序结束标志
    // show_resv_data(&motor_r);               // 显示最终电机状态（注释掉）

    // ========== 资源清理 ==========
    
    // 关闭串口连接，释放系统资源
    close_serial(fd);
    
    // Windows系统下暂停程序，等待用户按键（便于查看结果）
#if defined(__WIN32__)
    system("pause");
#endif
    
    return 0;  // 程序正常退出
}

/*
* 程序说明：
* 
* 1. 电机控制模式：
*    - 模式0：停止/空闲模式，电机不输出力矩
*    - 模式5：速度控制模式，电机按指定角速度运行
* 
* 2. 通信协议：
*    - 发送数据：34字节的命令包
*    - 接收数据：78字节的状态反馈包
*    - 同步通信：每发送一个命令必须接收一个反馈
* 
* 3. 安全措施：
*    - 程序开始和结束都发送停止命令
*    - 确保电机在安全状态下启动和停止
* 
* 4. 控制频率：
*    - 100ms间隔发送命令（10Hz控制频率）
*    - 总运行时间约10秒（100次×100ms）
* 
* 5. 使用场景：
*    - 用于测试电机连接和基本功能
*    - 验证串口通信是否正常
*    - 作为更复杂控制程序的基础模板
*/