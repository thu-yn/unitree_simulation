/*
* Unitree电机SDK - 电机控制核心接口定义
* 文件名: motor_ctrl.h
* 功能: 定义电机控制的高层数据结构和核心函数接口
* 说明: 提供用户友好的电机控制接口，封装底层通信协议的复杂性
*/

#ifndef MOTOR_CTRL
#define MOTOR_CTRL

#ifdef __cplusplus
extern "C"{
#endif

#include "motor_msg.h"  // 电机通信协议底层消息结构
#include <stdint.h>

/*
* 电机发送命令结构 - 用户接口层
* 功能: 为用户提供简化的电机控制接口，隐藏底层协议细节
* 说明: 将用户友好的浮点数参数转换为底层的定点数格式
*/
typedef struct {
    // 底层通信数据包
    MasterComdDataV3  motor_send_data;  // 电机控制数据结构体（详见motor_msg.h）
    
    // 数据包属性
    int hex_len;                        // 发送命令字节数，本电机固定为34字节
    long long send_time;                // 发送该命令的时间戳（微秒us）
    
    // 用户控制参数 - 浮点数接口（用户友好）
    unsigned short id;                  // 电机ID编号
                                        // 0: 单个电机 | 0xBB: 广播到所有电机
    
    unsigned short mode;                // 电机控制模式
                                        // 0: 空闲模式（电机自由转动）
                                        // 5: 开环缓慢转动模式
                                        // 10: 闭环FOC精确控制模式
    
    /*
    * 力矩控制参数说明:
    * 实际传递给FOC控制器的指令力矩计算公式：
    * 最终力矩 = K_P × δ位置 + K_W × δ速度 + T
    * 
    * 注意: 以下参数均为电机本身参数，不包含减速器影响
    */
    float T;                            // 期望电机本身的输出力矩（单位: Nm）
                                        // 范围: T < 255.9 Nm
    
    float W;                            // 期望电机本身的转速（单位: rad/s）
                                        // 范围: W < 511.9 rad/s
    
    float Pos;                          // 期望电机本身的位置（单位: rad）
                                        // 范围: Pos < 131071.9 rad
    
    float K_P;                          // 电机本身的位置刚度系数
                                        // 范围: K_P < 31.9
                                        // 说明: 越大则位置控制越强
    
    float K_W;                          // 电机本身的速度刚度系数（阻尼系数）
                                        // 范围: K_W < 63.9
                                        // 说明: 越大则速度控制越强，运动越平滑
}MOTOR_send;

/*
* 电机接收数据结构 - 用户接口层
* 功能: 为用户提供解析后的电机状态反馈信息
* 说明: 将底层定点数格式转换为用户友好的浮点数格式
*/
typedef struct
{
    // 底层通信数据包
    ServoComdDataV3 motor_recv_data;    // 电机接收数据结构体（详见motor_msg.h）
    
    // 数据包属性
    int hex_len;                        // 接收命令字节数，本电机固定为78字节
    long long recv_time;                // 接收该命令的时间戳（微秒us）
    int correct;                        // 接收数据完整性标志
                                        // 1: 数据完整 | 0: 数据不完整
    
    // 电机基本状态信息
    unsigned char motor_id;             // 电机ID编号
    unsigned char mode;                 // 当前电机控制模式
                                        // 0: 空闲 | 5: 开环转动 | 10: 闭环FOC控制
    int Temp;                           // 电机当前温度（摄氏度）
    unsigned char MError;               // 电机错误状态码
                                        // 0: 正常 | 其他值: 对应错误类型

    // 电机运动状态反馈（浮点数格式，用户友好）
    float T;                            // 当前实际电机输出力矩（Nm）
    float W;                            // 当前实际电机转速-高速模式（rad/s）
    float LW;                           // 当前实际电机转速-低速高精度模式（rad/s）
    int Acc;                            // 电机转子加速度（rad/s²）
    float Pos;                          // 当前电机位置（rad）
                                        // 注: 主控0点修正，电机关节仍以编码器0点为准

    // 电机驱动板集成的6轴传感器数据
    float gyro[3];                      // 3轴陀螺仪数据 [X, Y, Z]（rad/s）
    float acc[3];                       // 3轴加速度计数据 [X, Y, Z]（m/s²）

}MOTOR_recv;

/*
 * 核心功能函数声明
 * 功能: 提供电机控制的核心API接口
 */

/**
* @brief 获取当前系统时间
* @return long long 自系统启动以来的微秒数
* @note 用于时间戳记录和性能分析
* @warning 在不同系统上实现可能不同，建议仅用于相对时间计算
*/
extern long long getSystemTime();

/**
* @brief 将用户数据处理为STM32控制器需求的格式
* @param motor_send 指向MOTOR_send结构的指针
* @return int 处理结果
*   - 0: 数据处理失败
*   - 1: 数据处理成功
* @note 执行以下操作：
*   - 浮点数参数转换为定点数格式
*   - 数据打包为底层通信协议格式
*   - 计算并添加CRC32校验码
*   - 设置数据包头和长度信息
* @warning 调用前必须正确设置所有控制参数，超出范围的参数会被截断
*/
extern int modify_data(MOTOR_send* motor_send);

/**
* @brief 将接收到的原始数据解读为用户友好格式
* @param motor_recv 指向MOTOR_recv结构的指针
* @return int 解析结果
*   - 0: 数据解析失败（可能是校验错误或数据损坏）
*   - 1: 数据解析成功
* @note 执行以下操作：
*   - 验证数据包完整性和CRC校验
*   - 定点数格式转换为浮点数
*   - 解析电机状态和传感器数据
*   - 设置correct标志位
* @warning 仅处理经过CRC验证的有效数据包
*/
extern int extract_data(MOTOR_recv* motor_recv);

/**
* @brief 计算CRC32校验码
* @param data_array 待校验数据数组指针
* @param array_length 数组长度（以32位字为单位，余数舍去）
* @return uint32_t 计算得到的CRC32校验码
* @note 用于确保数据传输的完整性和正确性
* @warning 输入数组长度必须是4字节的整数倍
* @see IEEE 802.3 CRC32算法标准
*/
uint32_t crc32_core(uint32_t* data_array, uint32_t array_length);

/*
* 使用说明:
* 
* 1. 数据发送流程:
*    - 填充MOTOR_send结构体的用户参数
*    - 调用modify_data()进行数据格式转换
*    - 使用串口发送functions发送数据包
* 
* 2. 数据接收流程:
*    - 使用串口接收functions接收数据包
*    - 调用extract_data()进行数据解析
*    - 读取MOTOR_recv结构体获取电机状态
* 
* 3. 控制模式选择:
*    - mode=0: 适用于电机断电或自由转动
*    - mode=5: 适用于缓慢调试和测试
*    - mode=10: 适用于精确的力矩/位置/速度控制
* 
* 4. 参数范围注意:
*    - 所有控制参数都有最大值限制，超出会被截断
*    - 建议在实际应用中留有安全余量
*/

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CTRL */