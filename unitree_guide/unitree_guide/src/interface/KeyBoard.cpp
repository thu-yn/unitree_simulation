/**********************************************************************
 * 文件名: KeyBoard.cpp
 * 文件作用: 键盘输入接口实现类，用于在仿真环境中通过键盘输入控制四足机器人
 *          提供键盘按键到机器人命令的映射功能，支持运动控制和状态切换
 * 
 * 主要功能:
 * - 非阻塞键盘输入检测
 * - 按键到机器人控制命令的映射转换
 * - 模拟手柄摇杆的连续值控制
 * - 多线程键盘监听
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

/**
 * @brief 构造函数 - 初始化键盘输入接口
 * 
 * 功能说明:
 * 1. 初始化用户命令和控制值
 * 2. 配置终端为非阻塞、非回显模式
 * 3. 创建独立线程进行键盘监听
 * 
 * 终端设置详解:
 * - ICANON: 禁用规范模式(按行输入)，启用字符级输入
 * - ECHO: 禁用回显，输入字符不会显示在终端
 * - 这样设置可以实现实时键盘检测，无需按回车确认
 */
KeyBoard::KeyBoard(){
    // 初始化用户命令为无命令状态
    userCmd = UserCommand::NONE;
    // 清零所有控制值(左摇杆lx,ly, 右摇杆rx,ry, L2扳机值)
    userValue.setZero();

    // 保存当前终端设置，用于析构时恢复
    tcgetattr( fileno( stdin ), &_oldSettings );
    // 复制当前设置作为新设置的基础
    _newSettings = _oldSettings;
    // 修改终端属性：禁用规范模式和回显
    // ~ICANON: 禁用行缓冲模式，可以逐字符读取
    // ~ECHO: 禁用输入回显显示
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    // 立即应用新的终端设置
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    // 创建独立线程执行键盘监听任务
    // runKeyBoard是静态成员函数，this指针作为参数传递
    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

/**
 * @brief 析构函数 - 清理资源并恢复终端设置
 * 
 * 功能说明:
 * 1. 取消键盘监听线程
 * 2. 等待线程结束
 * 3. 恢复终端的原始设置
 */
KeyBoard::~KeyBoard(){
    // 发送取消信号给键盘监听线程
    pthread_cancel(_tid);
    // 等待线程完全结束，避免资源泄露
    pthread_join(_tid, NULL);
    // 恢复终端的原始设置(规范模式和回显)
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

/**
 * @brief 检查按键命令 - 将按键字符转换为机器人控制命令
 * 
 * @return UserCommand 对应的用户命令枚举值
 * 
 * 按键映射说明:
 * 数字键 1-4: 对应L2+按钮的组合命令(状态切换)
 * 数字键 8-0: 对应L1+按钮的组合命令(特殊功能)
 * 空格键: 停止并清零所有控制值
 * 
 * 命令对应关系:
 * '1' -> L2_B    : 通常用于切换到被动模式
 * '2' -> L2_A    : 通常用于切换到固定站立模式  
 * '3' -> L2_X    : 通常用于切换到小跑模式
 * '4' -> START   : 开始命令
 * '5' -> L2_Y    : 移动基座相关(仅在MOVE_BASE编译选项下)
 * '0' -> L1_X    : L1+X组合命令
 * '9' -> L1_A    : L1+A组合命令
 * '8' -> L1_Y    : L1+Y组合命令
 */
UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;    // L2+B组合 - 被动模式
    case '2':
        return UserCommand::L2_A;    // L2+A组合 - 固定站立
    case '3':
        return UserCommand::L2_X;    // L2+X组合 - 小跑模式
    case '4':
        return UserCommand::START;   // 开始命令
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;    // L2+Y组合 - 移动基座(条件编译)
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;    // L1+X组合
    case '9':
        return UserCommand::L1_A;    // L1+A组合
    case '8':
        return UserCommand::L1_Y;    // L1+Y组合
    case ' ':
        // 空格键：重置所有控制值为零(紧急停止)
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;    // 未识别按键，无命令
    }
}

/**
 * @brief 改变控制值 - 根据按键调整模拟摇杆数值
 * 
 * 功能说明:
 * 模拟游戏手柄的双摇杆控制，通过键盘按键产生连续的控制值
 * 
 * 按键布局设计:
 * WASD键组 - 模拟左摇杆(通常控制机器人平移)
 * IJKL键组 - 模拟右摇杆(通常控制机器人旋转)
 * 
 * 控制值范围: [-1.0, 1.0]
 * 灵敏度设置: 左摇杆和右摇杆分别可设置不同灵敏度
 * 
 * 坐标系定义:
 * lx/rx: 左右方向 (正值向右)
 * ly/ry: 前后方向 (正值向前)
 */
void KeyBoard::changeValue(){
    switch (_c){
    // 左摇杆控制组 (WASD键) - 控制机器人身体平移
    case 'w':case 'W':
        // W键: 向前移动，增加ly值(纵向正方向)
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        // S键: 向后移动，减少ly值(纵向负方向)  
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        // D键: 向右移动，增加lx值(横向正方向)
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        // A键: 向左移动，减少lx值(横向负方向)
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    // 右摇杆控制组 (IJKL键) - 控制机器人身体旋转
    case 'i':case 'I':
        // I键: 向前倾斜或俯仰，增加ry值
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        // K键: 向后倾斜或俯仰，减少ry值
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        // L键: 向右旋转，增加rx值(偏航正方向)
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        // J键: 向左旋转，减少rx值(偏航负方向)
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        // 其他按键不改变控制值
        break;
    }
}

/**
 * @brief 线程启动函数 - pthread线程的静态入口点
 * 
 * @param arg 传入的KeyBoard对象指针
 * @return void* 线程返回值(始终为NULL)
 * 
 * 说明:
 * 由于pthread_create要求线程函数为静态函数，
 * 此函数作为跳板，将调用转发到实例的run方法
 */
void* KeyBoard::runKeyBoard(void *arg){
    // 将void*参数转换为KeyBoard对象指针，并调用其run方法
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

/**
 * @brief 键盘监听主循环 - 实际的键盘检测和处理逻辑
 * 
 * @param arg 未使用的参数
 * @return void* 线程返回值(始终为NULL)
 * 
 * 工作流程:
 * 1. 使用select系统调用监听标准输入
 * 2. 有输入时读取单个字符
 * 3. 优先检查是否为命令按键
 * 4. 如非命令按键，则处理为控制值调整
 * 5. 短暂休眠后继续循环
 * 
 * 技术要点:
 * - 使用select实现非阻塞I/O
 * - 每次只读取一个字符，确保实时响应
 * - 读取后立即清零字符变量，避免重复处理
 */
void* KeyBoard::run(void *arg){
    while(1){
        // 清零文件描述符集合
        FD_ZERO(&set);
        // 将标准输入文件描述符添加到监听集合
        FD_SET( fileno( stdin ), &set );

        // 使用select监听标准输入，等待有数据可读
        // fileno( stdin )+1: 监听的文件描述符范围
        // &set: 读文件描述符集合
        // NULL, NULL, NULL: 写集合、异常集合、超时时间(无限等待)
        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        // 如果有数据可读(res > 0)
        if(res > 0){
            // 从标准输入读取一个字符
            ret = read( fileno( stdin ), &_c, 1 );
            
            // 首先检查是否为命令按键
            userCmd = checkCmd();
            
            // 如果不是命令按键，则作为控制值调整按键处理
            if(userCmd == UserCommand::NONE)
                changeValue();
                
            // 清零字符变量，避免重复处理同一按键
            _c = '\0';
        }
        
        // 短暂休眠1毫秒，降低CPU占用率
        // 既保证实时性，又避免过度占用系统资源
        usleep(1000);
    }
    return NULL;
}