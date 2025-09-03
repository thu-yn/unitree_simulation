/*
* Unitree电机SDK - 串口通信接口定义
* 文件名: LSerial.h
* 功能: 提供跨平台的串口操作函数声明，支持Linux和Windows系统
* 说明: 封装了底层串口操作，为电机通信提供统一的接口
*/

#ifndef LSERIAL
#define LSERIAL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_ctrl.h"

/* Linux平台串口操作函数 */
#if defined(__linux__)

    /**
    * @brief 打开并配置串口
    * @param serial_name 串口设备名称指针（如"/dev/ttyUSB0"）
    * @return int 串口文件描述符句柄，失败返回负值
    * @note 波特率固定为4800000 bps，8位数据位，1位停止位，无校验位
    * @warning 需要root权限才能访问串口设备
    */
    extern int open_set(char* serial_name);

    /**
    * @brief 关闭串口连接
    * @param fd 串口文件描述符句柄
    * @return int 操作结果，0表示成功，非0表示失败
    * @note 释放串口资源，关闭文件描述符
    */
    extern int close_serial(int fd);

    /**
    * @brief 向所有电机广播命令（无响应）
    * @param fd 串口文件描述符句柄
    * @param motor_send 指向发送数据结构的指针
    * @return int 发送状态，0表示失败，1表示成功
    * @note 广播模式下电机不会返回响应数据，适用于同时控制多个电机
    * @warning 广播时motorID应设置为0xBB
    */
    extern int broadcast(int fd, MOTOR_send* motor_send);

    /**
    * @brief 同步发送接收电机命令
    * @param fd 串口文件描述符句柄
    * @param motor_send 指向发送数据结构的指针
    * @param motor_recv 指向接收数据结构的指针
    * @return int 通信状态码
    *   - 0: 发送失败且接收失败
    *   - 1: 发送成功但接收失败
    *   - 11: 发送成功且接收成功（正常状态）
    * @note 每次发送34字节命令，必须接收78字节响应
    * @warning 这是阻塞函数，会等待电机响应或超时
    */
    extern int send_recv(int fd, MOTOR_send* motor_send, MOTOR_recv* motor_recv);

/* Windows平台串口操作函数 */
#elif defined(__WIN32__)
    #include <windows.h>

    /**
    * @brief 打开并配置串口（Windows版本）
    * @param serial_name 串口设备名称指针（如"\\\\.\\COM4"）
    * @return HANDLE 串口句柄，失败返回INVALID_HANDLE_VALUE
    * @note 波特率固定为4800000 bps，8位数据位，1位停止位，无校验位
    * @warning Windows下串口名称格式为"\\\\.\\COMx"
    */
    extern HANDLE open_set(char* serial_name);

    /**
    * @brief 关闭串口连接（Windows版本）
    * @param hSerial 串口句柄
    * @return int 操作结果，0表示成功，非0表示失败
    * @note 释放串口资源，关闭句柄
    */
    extern int close_serial(HANDLE hSerial);

    /**
    * @brief 向所有电机广播命令（Windows版本）
    * @param hSerial 串口句柄
    * @param motor_send 指向发送数据结构的指针
    * @return int 发送状态，0表示失败，1表示成功
    * @note 广播模式下电机不会返回响应数据，适用于同时控制多个电机
    * @warning 广播时motorID应设置为0xBB
    */
    extern int broadcast(HANDLE hSerial, MOTOR_send* motor_send);

    /**
    * @brief 同步发送接收电机命令（Windows版本）
    * @param hSerial 串口句柄
    * @param motor_send 指向发送数据结构的指针
    * @param motor_recv 指向接收数据结构的指针
    * @return int 通信状态码
    *   - 0: 发送失败且接收失败
    *   - 1: 发送成功但接收失败
    *   - 11: 发送成功且接收成功（正常状态）
    * @note 每次发送34字节命令，必须接收78字节响应
    * @warning 这是阻塞函数，会等待电机响应或超时
    */
    extern int send_recv(HANDLE hSerial, MOTOR_send* motor_send, MOTOR_recv* motor_recv);

#endif

/*
* 使用说明:
* 
* 1. 串口初始化流程:
*    Linux: fd = open_set("/dev/ttyUSB0");
*    Windows: hSerial = open_set("\\\\.\\COM4");
* 
* 2. 通信流程:
*    - 调用modify_data()准备发送数据
*    - 调用send_recv()进行同步通信
*    - 调用extract_data()解析接收数据
* 
* 3. 资源清理:
*    close_serial(fd/hSerial);
* 
* 4. 错误处理:
*    - 检查open_set()返回值确认串口打开成功
*    - 检查send_recv()返回值确认通信状态
*    - 建议添加重试机制处理通信异常
*/

#ifdef __cplusplus
}
#endif

#endif /* LSERIAL */