/**********************************************************************
 * 文件名: WirelessHandle.h
 * 版权所有: Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 * 
 * 文件作用: 
 * 本文件定义了WirelessHandle类，是unitree_guide项目中专门处理无线遥控手柄输入的核心组件。
 * 该类继承自CmdPanel抽象基类，负责接收和解析来自Unitree机器人无线手柄的控制信号，
 * 并将这些信号转换为标准化的用户命令和控制数值。这是真实机器人环境下用户与机器人
 * 交互的主要接口，替代了仿真环境中的KeyBoard键盘控制。
 * 
 * 在unitree_guide控制架构中的位置:
 * - 作为IOInterface的输入处理组件，专门服务于IOSDK真实机器人接口
 * - 与IOROS中的KeyBoard形成对应关系：仿真用键盘，实机用手柄
 * - 为FSM状态机提供标准化的用户指令输入
 * - 支持A1和Go1等不同型号的Unitree四足机器人
 ***********************************************************************/

#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

// 包含手柄消息数据结构定义
#include "message/unitree_joystick.h"
// 包含命令面板基类定义
#include "interface/CmdPanel.h"
// 包含Unitree SDK底层通信接口
#include "unitree_legged_sdk/comm.h"

/**
 * @class WirelessHandle
 * @brief 无线手柄控制器类
 * 
 * 这个类是unitree_guide框架中专门负责处理无线游戏手柄输入的核心组件。
 * 它继承自CmdPanel基类，实现了从物理手柄设备到标准化控制指令的转换。
 * 
 * 主要职责:
 * 1. 接收并解析无线手柄的原始数据
 * 2. 将手柄按键组合转换为预定义的用户命令枚举
 * 3. 将摇杆模拟量转换为归一化的控制数值
 * 4. 提供死区处理，消除摇杆零点漂移
 * 5. 为上层控制系统提供统一的用户输入接口
 * 
 * 设计特点:
 * - 实时性: 配合500Hz控制循环实时处理手柄输入
 * - 跨平台: 支持A1和Go1不同机器人平台的手柄协议差异
 * - 标准化: 输出标准的UserCommand和UserValue格式
 * - 容错性: 包含死区处理和数据有效性检查
 * 
 * 使用场景:
 * - 真实机器人的遥控操作
 * - 步态切换和运动模式选择  
 * - 机器人速度和方向的实时控制
 * - 特殊功能模式的激活（如平衡测试、摆动测试等）
 */
class WirelessHandle : public CmdPanel{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化WirelessHandle对象，设置默认参数。
     * 继承自CmdPanel的userCmd和userValue成员会被初始化为默认状态。
     * _keyData结构会被初始化为零状态，等待第一次有效数据接收。
     */
    WirelessHandle();
    
    /**
     * @brief 析构函数
     * 
     * 由于WirelessHandle不需要特殊的清理操作（没有动态分配内存或系统资源），
     * 因此使用默认的空析构函数。所有成员变量会自动销毁。
     */
    ~WirelessHandle(){}
    
    /**
     * @brief 接收并处理手柄数据的核心函数
     * @param lowState 指向底层状态数据的指针，包含从机器人SDK接收的完整状态信息
     * 
     * 这是WirelessHandle类的核心功能函数，负责:
     * 
     * 1. 数据提取: 从LowState的wirelessRemote字段中提取40字节的手柄原始数据
     * 2. 协议适配: 根据机器人型号(A1/Go1)使用不同的数据提取方式
     * 3. 按键解析: 将16位按键状态位解析为具体的按键组合
     * 4. 命令映射: 将按键组合映射为UserCommand枚举值:
     *    - L2+B → L2_B (被动模式/紧急停止)
     *    - L2+A → L2_A (固定站立模式)  
     *    - L2+X → L2_X (自由站立模式)
     *    - L2+Y → L2_Y (导航模式，需要MOVE_BASE编译选项)
     *    - L1+X → L1_X (平衡测试模式)
     *    - L1+A → L1_A (摆动测试模式)
     *    - L1+Y → L1_Y (步态测试模式)
     *    - START → START (开始行走/Trotting模式)
     * 
     * 5. 摇杆处理: 将5个模拟摇杆通道的数据进行处理:
     *    - lx/ly: 左摇杆X/Y轴，通常控制机器人前进后退和左右平移
     *    - rx/ry: 右摇杆X/Y轴，通常控制机器人转向和侧移
     *    - L2: L2扳机深度，可用于速度调节
     * 
     * 6. 死区消除: 使用killZeroOffset函数消除摇杆的零点漂移（死区0.08）
     * 
     * 数据流向:
     * 机器人硬件 → SDK → LowState.wirelessRemote → _keyData → userCmd/userValue → 上层控制器
     * 
     * 注意事项:
     * - 此函数仅在COMPILE_WITH_REAL_ROBOT宏开启时编译
     * - 需要Unitree SDK版本3.2及以上才支持wirelessRemote数据
     * - 函数应在500Hz控制循环中被调用，保证实时性
     * - 不同机器人型号的数据格式略有差异，通过条件编译处理
     */
    void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState);

private:
    /**
     * @brief 手柄数据缓存结构
     * 
     * 用于存储从机器人SDK接收到的40字节手柄原始数据。
     * 这个结构体包含了完整的手柄状态信息:
     * 
     * 结构组成:
     * - head[2]: 数据包头部，用于协议识别
     * - btn: 16位按键状态联合体，每一位对应一个按键
     *   * R1/L1: 右肩键/左肩键
     *   * R2/L2: 右扳机/左扳机  
     *   * start/select: 开始键/选择键
     *   * A/B/X/Y: 四个主按键
     *   * up/down/left/right: 方向键
     *   * F1/F2: 功能键
     * - lx/ly: 左摇杆X/Y轴模拟量 [-1.0, 1.0]
     * - rx/ry: 右摇杆X/Y轴模拟量 [-1.0, 1.0]  
     * - L2: L2扳机深度模拟量 [0.0, 1.0]
     * - idle[16]: 保留字节，用于协议扩展
     * 
     * 数据更新:
     * 每次调用receiveHandle时，会从LowState中复制最新的40字节数据到此结构。
     * 这确保了手柄状态的实时同步，延迟通常小于2ms(500Hz控制频率)。
     * 
     * 内存布局:
     * 该结构体使用标准的C结构体内存布局，总大小为40字节，
     * 与Unitree SDK中定义的无线手柄数据包格式完全兼容。
     */
    xRockerBtnDataStruct _keyData;
};

#endif  // WIRELESSHANDLE_H