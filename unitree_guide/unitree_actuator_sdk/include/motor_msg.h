/*
* Unitree电机SDK - 电机通信消息结构定义
* 文件名: motor_msg.h
* 功能: 定义PC与电机控制板之间通信的数据包格式和消息结构
* 说明: 包含命令发送和状态接收的完整数据结构定义
*/

#ifndef MOTOR_MSG
#define MOTOR_MSG

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
typedef int16_t q15_t;   // 定义Q15定点数类型，用于高精度数值表示

#pragma pack(1)          // 强制1字节对齐，确保数据包结构的准确性

/*
* 通用数据联合体 - 4字节通用数据结构
* 功能: 提供多种数据类型的统一存储格式，方便数据转换和传输
*/
typedef union{
    int32_t    L;        // 32位有符号整数
    uint8_t    u8[4];    // 4个8位无符号整数数组
    uint16_t   u16[2];   // 2个16位无符号整数数组  
    uint32_t   u32;      // 32位无符号整数
    float      F;        // 32位浮点数
}COMData32;

/*
* 通信数据包头结构
* 功能: 定义每个数据包的头部信息，用于数据包识别和路由
*/
typedef struct {
    unsigned char  start[2];    // 数据包起始标识 [0xFE, 0xEE]
    unsigned char  motorID;     // 目标电机ID (0,1,2,3... | 0xBB=广播)
    unsigned char  reserved;    // 保留字节，用于未来扩展
}COMHead;

#pragma pack()           // 恢复默认对齐

#pragma pack(1)          // 重新设置1字节对齐

/*
* 低频率电机控制命令结构
* 功能: 定义低频率更新的电机附加功能控制（如LED、音响、风扇等）
*/
typedef struct { 
    uint8_t  fan_d;           // 关节散热风扇转速控制
    uint8_t  Fmusic;          // 电机发声频率 (/64*1000, 15.625Hz分辨率)
    uint8_t  Hmusic;          // 电机发声强度 (推荐值4, 0.1分辨率)
    uint8_t  reserved4;       // 保留字节
    uint8_t  FRGB[4];         // 足端LED颜色控制 (RGBW)
}LowHzMotorCmd;

/*
* 主控制命令结构 (发送给电机)
* 功能: 定义发送给电机的完整控制命令，包含所有控制参数
* 大小: 30字节数据 + 4字节包头 = 34字节总包
*/
typedef struct {
    // 控制模式和修改标志位
    uint8_t   mode;                 // 关节控制模式选择
                                    // 0: 空闲模式 | 5: 开环缓慢转动 | 10: 闭环FOC控制
    uint8_t   ModifyBit;            // 电机控制参数修改位
    uint8_t   ReadBit;              // 电机控制参数发送位  
    uint8_t   reserved;             // 保留字节

    // 电机参数修改数据
    COMData32 Modify;               // 电机参数修改的数据容器

    // 核心控制参数 (实际FOC控制器的指令力矩 = K_P*δPos + K_W*δW + T)
    q15_t     T;                    // 期望关节输出力矩 (电机本身力矩×256, 7+8位描述)
    q15_t     W;                    // 期望关节速度 (电机本身速度×128, 8+7位描述)  
    int32_t   Pos;                  // 期望关节位置 (×16384/6.2832, 14位编码器分辨率)
                                    // 注: 主控0点修正，但电机关节仍以编码器0点为准

    // PID控制参数
    q15_t     K_P;                  // 关节位置刚度系数 (×2048, 4+11位描述)
    q15_t     K_W;                  // 关节速度阻尼系数 (×1024, 5+10位描述)

    // 低频控制命令
    uint8_t   LowHzMotorCmdIndex;   // 低频控制命令索引 (0-7，对应LowHzMotorCmd的8个字节)
    uint8_t   LowHzMotorCmdByte;    // 低频控制命令字节数据
    
    COMData32 Res[1];               // 通信保留字节，用于实现其他通信内容
}MasterComdV3;

/*
* 主控制命令数据包 (完整发送包)
* 功能: 封装完整的发送数据包，包含包头、控制数据和CRC校验
* 总大小: 4 + 30 + 4 = 34字节
*/
typedef struct {
    COMHead       head;       // 数据包头 (4字节)
    MasterComdV3  Mdata;      // 主控制数据 (30字节)
    COMData32     CRCdata;    // CRC校验数据 (4字节)
}MasterComdDataV3;

#pragma pack()            // 恢复默认对齐

#pragma pack(1)           // 重新设置1字节对齐

/*
* 电机状态反馈结构 (从电机接收)
* 功能: 定义从电机接收的完整状态反馈信息
* 大小: 70字节数据 + 4字节包头 + 4字节CRC = 78字节总包
*/
typedef struct {
    // 基本状态信息
    uint8_t   mode;           // 当前关节控制模式
    uint8_t   ReadBit;        // 电机控制参数修改成功标志位
    int8_t    Temp;           // 电机当前平均温度
    uint8_t   MError;         // 电机错误状态标识码

    // 当前控制数据读取
    COMData32 Read;           // 读取的当前电机控制数据
    
    // 力矩和速度反馈
    int16_t   T;              // 当前实际电机输出力矩 (7+8位描述)
    int16_t   W;              // 当前实际电机速度-高速 (8+7位描述)
    float     LW;             // 当前实际电机速度-低速 (高精度)
    int16_t   W2;             // 当前实际关节速度-高速 (8+7位描述)
    float     LW2;            // 当前实际关节速度-低速 (高精度)

    // 加速度反馈
    int16_t   Acc;            // 电机转子加速度 (15+0位描述，惯量较小)
    int16_t   OutAcc;         // 输出轴加速度 (12+3位描述，惯量较大)
        
    // 位置反馈
    int32_t   Pos;            // 当前电机位置 (主控0点修正，电机关节以编码器0点为准)
    int32_t   Pos2;           // 关节编码器位置 (输出编码器)

    // 6轴惯性传感器数据 (电机驱动板集成)
    int16_t   gyro[3];        // 3轴陀螺仪数据 [X, Y, Z]
    int16_t   acc[3];         // 3轴加速度计数据 [X, Y, Z]

    // 力传感器数据 (足端力传感器)
    int16_t   Fgyro[3];       // 力传感器陀螺仪数据 [X, Y, Z]
    int16_t   Facc[3];        // 力传感器加速度计数据 [X, Y, Z]
    int16_t   Fmag[3];        // 力传感器磁力计数据 [X, Y, Z]
    uint8_t   Ftemp;          // 力传感器温度 (7位:-28~100°C, 1位:0.5°C分辨率)
    
    // 力值数据
    int16_t   Force16;        // 力传感器高16位数据
    int8_t    Force8;         // 力传感器低8位数据
        
    // 错误和保留
    uint8_t   FError;         // 足端传感器错误标识
    int8_t    Res[1];         // 通信保留字节
}ServoComdV3;

/*
* 电机状态反馈数据包 (完整接收包)
* 功能: 封装完整的接收数据包，包含包头、状态数据和CRC校验
* 总大小: 4 + 70 + 4 = 78字节
*/
typedef struct {
    COMHead       head;       // 数据包头 (4字节)
    ServoComdV3   Mdata;      // 电机状态数据 (70字节)
    COMData32     CRCdata;    // CRC校验数据 (4字节)
}ServoComdDataV3;

#pragma pack()            // 恢复默认对齐

/*
* 数据包初始化参考:
* 
* 发送数据包默认初始化示例:
* head.start[0] = 0xFE; head.start[1] = 0xEE;  // 包头标识
* Mdata.ModifyBit = 0xFF; Mdata.mode = 0;      // 默认不修改参数，空闲模式
* head.motorID = 0;                            // 目标电机ID
* Mdata.T = 0.0f;                              // 默认零力矩
* Mdata.Pos = 0x7FE95C80;                      // 默认位置(不启用位置环)
* Mdata.W = 16000.0f;                          // 默认速度(不启用速度环)
* Mdata.K_P = (q15_t)(0.6f*(1<<11));          // 默认位置刚度系数
* Mdata.K_W = (q15_t)(1.0f*(1<<10));          // 默认速度阻尼系数
*/

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MSG */