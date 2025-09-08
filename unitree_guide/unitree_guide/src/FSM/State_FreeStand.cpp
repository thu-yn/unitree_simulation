/**********************************************************************
 * 文件名称: State_FreeStand.cpp
 * 文件作用: 自由站立状态实现
 * 
 * 功能描述:
 * 本文件实现了机器人的自由站立状态(FREESTAND)，这是一个动态平衡站立状态。
 * 与固定站立状态不同，自由站立状态允许用户通过手柄输入来实时调整机器人的:
 * - 身体姿态（横滚roll、俯仰pitch、偏航yaw角度）
 * - 身体高度（上下移动）
 * 
 * 该状态是静态平衡控制的典型应用，展示了如何通过逆运动学计算
 * 将笛卡尔空间的身体姿态控制转换为关节空间的位置控制指令。
 * 
 * 主要特点:
 * 1. 实时响应用户输入，动态调整机器人姿态
 * 2. 保持四腿支撑，确保静态稳定
 * 3. 使用逆运动学将期望的身体姿态转换为各关节角度
 * 4. 具有安全的角度和高度限制
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "FSM/State_FreeStand.h"

/**
 * @brief 自由站立状态构造函数
 * @param ctrlComp 控制组件指针，包含所有必要的控制接口和数据
 * 
 * 在构造函数中设置该状态的基本参数和安全限制：
 * - 初始化状态名称和字符串标识
 * - 设置各轴旋转角度的安全限制范围
 * - 设置身体高度调整的安全限制范围
 */
State_FreeStand::State_FreeStand(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::FREESTAND, "free stand"){
    
    // ==================== 姿态角度限制设定 ====================
    // 设置横滚角(roll)限制：±20度
    // 横滚角控制机器人左右倾斜，防止侧翻
    _rowMax = 20 * M_PI / 180;    // 最大向右倾斜20度
    _rowMin = -_rowMax;           // 最大向左倾斜20度
    
    // 设置俯仰角(pitch)限制：±15度  
    // 俯仰角控制机器人前后倾斜，防止前倾或后倾过度
    _pitchMax = 15 * M_PI / 180;  // 最大向前倾斜15度
    _pitchMin = -_pitchMax;       // 最大向后倾斜15度
    
    // 设置偏航角(yaw)限制：±20度
    // 偏航角控制机器人水平旋转，防止扭转过大
    _yawMax = 20 * M_PI / 180;    // 最大顺时针旋转20度
    _yawMin = -_yawMax;           // 最大逆时针旋转20度
    
    // ==================== 高度调整限制设定 ====================
    // 设置身体高度调整限制：±0.04米（±4cm）
    // 在标准站立高度基础上允许小幅度上下调节
    _heightMax = 0.04;            // 最高可升高4cm
    _heightMin = -_heightMax;     // 最低可降低4cm
}

/**
 * @brief 进入自由站立状态时的初始化函数
 * 
 * 当状态机切换到自由站立状态时会调用此函数，进行必要的初始化操作：
 * 1. 设置所有关节的控制增益（根据平台类型选择仿真或真实机器人增益）
 * 2. 初始化关节速度和力矩命令为零
 * 3. 记录当前机器人的身体位置和足端位置作为参考
 * 4. 设置所有腿为支撑相
 * 5. 清零用户命令面板
 */
void State_FreeStand::enter(){
    
    // ==================== 关节控制参数初始化 ====================
    // 为每条腿的3个关节设置控制增益和初始命令
    for(int i=0; i<4; i++){
        
        // 根据控制平台类型设置不同的站立增益
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            // 仿真环境：使用仿真专用的站立增益参数
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            // 真实机器人：使用真实机器人的站立增益参数
            _lowCmd->setRealStanceGain(i);
        }
        
        // 将所有关节的期望速度设为0（静态站立，无运动）
        _lowCmd->setZeroDq(i);
        
        // 将所有关节的前馈力矩设为0（纯位置控制）
        _lowCmd->setZeroTau(i);
    }

    // ==================== 关节位置初始化 ====================
    // 将当前实际关节角度作为初始期望角度
    // 这样可以避免状态切换时的突然跳跃，实现平滑过渡
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    
    // ==================== 参考位置记录 ====================
    // 记录进入状态时的身体位置，作为后续姿态调整的参考原点
    _initVecOX = _ctrlComp->robotModel->getX(*_lowState);
    
    // 记录进入状态时各足端相对于身体的位置，作为足端位置控制的参考
    _initVecXP = _ctrlComp->robotModel->getVecXP(*_lowState);

    // ==================== 状态设置 ====================
    // 设置所有腿都处于支撑相（站立状态下四腿着地）
    _ctrlComp->setAllStance();
    
    // 清零用户命令面板，确保没有残留的用户输入
    _ctrlComp->ioInter->zeroCmdPanel();
}

/**
 * @brief 自由站立状态的主要执行函数
 * 
 * 这是状态的核心函数，在500Hz控制循环中被持续调用。
 * 主要功能：
 * 1. 读取用户手柄输入
 * 2. 将输入映射为身体姿态和高度调整
 * 3. 计算新的足端位置
 * 4. 发送关节控制命令
 */
void State_FreeStand::run(){
    Vec34 vecOP;  // 足端位置矩阵，4列3行，每列代表一只脚的3D位置
    
    // ==================== 用户输入获取 ====================
    // 获取当前的用户输入值（手柄摇杆数据）
    _userValue = _lowState->userValue;

    // ==================== 输入映射和足端位置计算 ====================
    // 调用_calcOP函数，将标准化的用户输入映射为期望的足端位置
    // 输入参数说明：
    // - invNormalize(_userValue.lx, _rowMin, _rowMax): 左摇杆X轴 → 横滚角
    // - invNormalize(_userValue.ly, _pitchMin, _pitchMax): 左摇杆Y轴 → 俯仰角  
    // - -invNormalize(_userValue.rx, _yawMin, _yawMax): 右摇杆X轴 → 偏航角（取负号是为了符合操作习惯）
    // - invNormalize(_userValue.ry, _heightMin, _heightMax): 右摇杆Y轴 → 高度调整
    vecOP = _calcOP( invNormalize(_userValue.lx, _rowMin, _rowMax),     // 横滚角调整
                     invNormalize(_userValue.ly, _pitchMin, _pitchMax), // 俯仰角调整
                    -invNormalize(_userValue.rx, _yawMin, _yawMax),     // 偏航角调整（取负符合习惯）
                     invNormalize(_userValue.ry, _heightMin, _heightMax) ); // 高度调整

    // ==================== 关节命令计算和发送 ====================
    // 将计算得到的足端位置转换为关节角度命令并发送
    _calcCmd(vecOP);
}

/**
 * @brief 退出自由站立状态时的清理函数
 * 
 * 当状态机准备切换到其他状态时会调用此函数，进行必要的清理操作。
 * 主要功能是清零用户命令面板，避免对下一个状态产生干扰。
 */
void State_FreeStand::exit(){
    // 清零用户命令面板，确保状态切换时的干净环境
    _ctrlComp->ioInter->zeroCmdPanel();
}

/**
 * @brief 检查状态转换条件
 * @return FSMStateName 下一个应该切换到的状态名称
 * 
 * 根据用户的按键输入决定是否需要切换状态：
 * - L2+A: 切换到固定站立状态
 * - L2+B: 切换到被动状态（机器人躺下）
 * - START: 切换到小跑运动状态
 * - 其他：保持当前自由站立状态
 */
FSMStateName State_FreeStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        // L2+A组合键：切换到固定站立状态
        // 固定站立状态会将机器人锁定在预设姿态，不响应姿态调整输入
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        // L2+B组合键：切换到被动状态
        // 被动状态下机器人会躺下，所有关节放松
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::START){
        // START键：切换到小跑运动状态
        // 小跑状态是机器人的主要运动状态，可以行走、转向等
        return FSMStateName::TROTTING;
    }
    else{
        // 没有有效的切换命令，保持当前自由站立状态
        return FSMStateName::FREESTAND;
    }
}

/**
 * @brief 根据期望的身体姿态和高度计算各足端的目标位置
 * @param row 期望的横滚角（绕X轴旋转）[弧度]
 * @param pitch 期望的俯仰角（绕Y轴旋转）[弧度]  
 * @param yaw 期望的偏航角（绕Z轴旋转）[弧度]
 * @param height 期望的高度调整量[米]
 * @return Vec34 4×3矩阵，每列代表一只脚的期望位置(x,y,z)
 * 
 * 计算流程：
 * 1. 构建新的身体位置向量（在初始位置基础上调整高度）
 * 2. 根据姿态角构建旋转矩阵
 * 3. 构建从世界坐标系到身体坐标系的变换矩阵
 * 4. 将初始足端位置通过变换矩阵转换到新的身体坐标系下
 * 
 * 这个函数实现了逆运动学控制的关键步骤：将笛卡尔空间的身体姿态控制
 * 转换为各足端在身体坐标系下的目标位置。
 */
Vec34 State_FreeStand::_calcOP(float row, float pitch, float yaw, float height){
    
    // ==================== 构建新的身体位置向量 ====================
    // 以初始身体位置为基准，应用高度调整
    Vec3 vecXO = -_initVecOX;     // 获取初始身体位置的负值（用于坐标变换）
    vecXO(2) += height;           // 在Z轴方向（垂直方向）上应用高度调整

    // ==================== 构建旋转矩阵 ====================
    // 根据期望的横滚、俯仰、偏航角构建3D旋转矩阵
    // 旋转矩阵描述了身体坐标系相对于世界坐标系的姿态变化
    RotMat rotM = rpyToRotMat(row, pitch, yaw);

    // ==================== 构建齐次变换矩阵 ====================
    // 齐次变换矩阵包含了旋转和平移信息，用于坐标系变换
    // Tsb: 从身体坐标系(body)到支撑坐标系(support)的变换
    HomoMat Tsb = homoMatrix(vecXO, rotM);
    
    // Tbs: 从支撑坐标系到身体坐标系的反向变换
    HomoMat Tbs = homoMatrixInverse(Tsb);

    // ==================== 足端位置变换计算 ====================
    Vec4 tempVec4;    // 临时4D向量，用于齐次坐标变换
    Vec34 vecOP;      // 输出矩阵：各足端在新身体坐标系下的位置

    // 对每只脚进行坐标变换
    for(int i(0); i<4; ++i){
        // 1. 将初始足端位置转换为齐次坐标（添加第4维为1）
        // 2. 通过变换矩阵Tbs将其从支撑坐标系转换到新的身体坐标系
        tempVec4 = Tbs * homoVec(_initVecXP.col(i));
        
        // 3. 将齐次坐标转换回3D坐标（移除第4维）
        vecOP.col(i) = noHomoVec(tempVec4);
    }

    return vecOP;  // 返回各足端的目标位置矩阵
}

/**
 * @brief 根据足端目标位置计算并发送关节角度命令
 * @param vecOP 4×3矩阵，每列代表一只脚的期望位置
 * 
 * 这个函数实现了逆运动学计算的最后一步：
 * 1. 使用机器人动力学模型将足端位置转换为关节角度
 * 2. 将计算出的关节角度作为命令发送给底层控制器
 * 
 * 关节角度的计算考虑了：
 * - 机器人的运动学链结构（髋关节-大腿-小腿）
 * - 各关节的物理限制
 * - 身体坐标系下的运动学约束
 */
void State_FreeStand::_calcCmd(Vec34 vecOP){
    
    // ==================== 逆运动学计算 ====================
    // 使用机器人模型的逆运动学接口，将足端位置转换为12个关节角度
    // 参数说明：
    // - vecOP: 各足端的期望位置矩阵
    // - FrameType::BODY: 指定足端位置是在身体坐标系下定义的
    Vec12 q = _ctrlComp->robotModel->getQ(vecOP, FrameType::BODY);
    
    // ==================== 关节命令发送 ====================
    // 将计算出的关节角度设置为期望位置命令
    // 底层控制器会执行位置控制，驱动各关节到达目标角度
    _lowCmd->setQ(q);
}