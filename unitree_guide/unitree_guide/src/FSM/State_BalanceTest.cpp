/**********************************************************************
 * 文件名: State_BalanceTest.cpp
 * 文件作用: 平衡测试状态的实现文件
 * 
 * 这个文件实现了Unitree四足机器人的平衡测试状态（BALANCETEST）。
 * 该状态主要用于测试机器人在静态站立时的平衡控制性能，包括：
 * 1. 响应用户手柄输入，调整机器人的位置和姿态
 * 2. 实现重心位置控制和姿态稳定控制
 * 3. 通过力控制算法保持机器人平衡
 * 4. 测试和验证平衡控制器的效果
 * 
 * 主要功能：
 * - 允许用户通过手柄控制机器人的x、y、z位置和yaw角度
 * - 实现PD控制器进行位置和姿态调节
 * - 通过全身动力学计算关节力矩
 * - 提供平衡控制算法的测试平台
 * 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "FSM/State_BalanceTest.h"

/**
 * @brief State_BalanceTest类的构造函数
 * 
 * 初始化平衡测试状态，设置控制参数和运动限制范围。
 * 这些参数决定了机器人在平衡测试中的运动幅度和控制特性。
 * 
 * @param ctrlComp 控制组件指针，包含所有必要的控制模块
 */
State_BalanceTest::State_BalanceTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::BALANCETEST, "balanceTest"),  // 调用基类构造函数，设置状态名称
                  _est(ctrlComp->estimator),      // 状态估计器引用
                  _robModel(ctrlComp->robotModel), // 机器人动力学模型引用
                  _balCtrl(ctrlComp->balCtrl),    // 平衡控制器引用
                  _contact(ctrlComp->contact){    // 接触状态引用

    // ==================== 运动范围限制设定 ====================
    // 设置X方向（前后）的运动范围限制，单位：米
    _xMax = 0.05;      // X方向最大前进距离：5厘米
    _xMin = -_xMax;    // X方向最大后退距离：-5厘米

    // 设置Y方向（左右）的运动范围限制，单位：米
    _yMax = 0.05;      // Y方向最大右移距离：5厘米
    _yMin = -_yMax;    // Y方向最大左移距离：-5厘米

    // 设置Z方向（上下）的运动范围限制，单位：米
    _zMax = 0.04;      // Z方向最大上升距离：4厘米
    _zMin = -_zMax;    // Z方向最大下降距离：-4厘米

    // 设置偏航角（yaw）的旋转范围限制，单位：弧度
    _yawMax = 20 * M_PI / 180;  // 最大偏航角：20度转换为弧度
    _yawMin = -_yawMax;         // 最小偏航角：-20度

    // ==================== 位置控制器PD参数设定 ====================
    // 位置控制的比例增益矩阵（Kp for position）
    // 对角矩阵，分别对应x、y、z三个方向的比例增益
    _Kpp = Vec3(150, 150, 150).asDiagonal();

    // 位置控制的微分增益矩阵（Kd for position）
    // 对角矩阵，分别对应x、y、z三个方向的微分增益
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    // ==================== 姿态控制器PD参数设定 ====================
    // 姿态控制的比例增益（Kp for orientation/angular）
    _kpw = 200;

    // 姿态控制的微分增益矩阵（Kd for orientation/angular）
    // 对角矩阵，分别对应roll、pitch、yaw三个角度的微分增益
    _Kdw = Vec3(30, 30, 30).asDiagonal();
}

/**
 * @brief 进入平衡测试状态时的初始化函数
 * 
 * 当状态机从其他状态切换到平衡测试状态时调用此函数。
 * 主要功能：
 * 1. 记录初始位置和姿态作为参考
 * 2. 设置所有腿为支撑状态
 * 3. 清零命令面板
 */
void State_BalanceTest::enter(){
    // 记录进入状态时的初始位置，作为控制的参考位置
    _pcdInit = _est->getPosition();    // 获取当前估计的机器人重心位置
    _pcd = _pcdInit;                   // 设置期望位置初始值等于当前位置

    // 记录进入状态时的初始旋转矩阵，作为姿态控制的参考
    _RdInit = _lowState->getRotMat();  // 获取当前的旋转矩阵

    // 设置所有腿都处于支撑状态（stance phase）
    // 这确保所有四条腿都接触地面，为平衡控制提供稳定的支撑
    _ctrlComp->setAllStance();

    // 清零I/O接口的命令面板，确保没有残留的控制指令
    _ctrlComp->ioInter->zeroCmdPanel();
}

/**
 * @brief 平衡测试状态的主运行函数
 * 
 * 这是平衡测试状态的核心函数，每个控制周期（500Hz）都会调用。
 * 主要功能：
 * 1. 读取用户手柄输入
 * 2. 计算期望的位置和姿态
 * 3. 更新机器人状态估计
 * 4. 计算所需的关节力矩
 * 5. 发送控制命令
 */
void State_BalanceTest::run(){
    // ==================== 用户输入处理 ====================
    // 获取用户手柄的输入值，这些值通常在-1到1之间
    _userValue = _lowState->userValue;

    // 根据手柄输入计算期望的重心位置
    // invNormalize函数将-1~1的输入映射到指定的运动范围
    // ly控制x方向（前后移动）
    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.ly, _xMin, _xMax);
    // lx控制y方向（左右移动），注意使用负号是为了符合直觉的控制方向
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue.lx, _yMin, _yMax);
    // ry控制z方向（上下移动）
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);

    // 根据手柄输入计算期望的偏航角
    // rx控制yaw角度（左右旋转）
    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    // 计算期望的旋转矩阵：在初始姿态基础上叠加偏航旋转
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;  // Roll=0, Pitch=0, 只改变Yaw

    // ==================== 状态估计更新 ====================
    // 获取当前机器人重心的实际位置和速度（通过状态估计器）
    _posBody = _est->getPosition();    // 当前重心位置
    _velBody = _est->getVelocity();    // 当前重心速度

    // 获取当前的旋转矩阵并计算其转置
    _B2G_RotMat = _lowState->getRotMat();      // 机体坐标系到全局坐标系的旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();    // 全局坐标系到机体坐标系的旋转矩阵

    // ==================== 控制力矩计算 ====================
    // 调用力矩计算函数，计算各关节所需的驱动力矩
    calcTau();

    // ==================== 命令发送 ====================
    // 设置稳定的控制增益（通常是预设的安全增益）
    _lowCmd->setStableGain();
    // 发送计算得到的关节力矩命令
    _lowCmd->setTau(_tau);
    // 发送期望的关节位置（用于混合控制）
    _lowCmd->setQ(_q);
}

/**
 * @brief 退出平衡测试状态时的清理函数
 * 
 * 当状态机从平衡测试状态切换到其他状态时调用此函数。
 * 主要功能是清理和重置控制命令，确保安全退出。
 */
void State_BalanceTest::exit(){
    // 清零I/O接口的命令面板，确保退出状态时没有残留的控制指令
    // 这是一个重要的安全措施，防止状态切换时出现意外的控制输出
    _ctrlComp->ioInter->zeroCmdPanel();
}

/**
 * @brief 检查状态转换条件
 * 
 * 根据用户输入判断是否需要切换到其他状态。
 * 这个函数在每个控制周期都会被调用，用于实现状态机的状态转换逻辑。
 * 
 * @return FSMStateName 下一个状态的名称
 */
FSMStateName State_BalanceTest::checkChange(){
    // 检查用户按下L2+B组合键，切换到被动状态
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    // 检查用户按下L2+A组合键，切换到固定站立状态
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    // 如果没有状态切换命令，保持在当前的平衡测试状态
    else{
        return FSMStateName::BALANCETEST;
    }
}

/**
 * @brief 计算关节驱动力矩的核心函数
 * 
 * 这是平衡控制的核心算法，通过以下步骤计算各关节所需的驱动力矩：
 * 1. 使用PD控制器计算期望的重心加速度和角加速度
 * 2. 通过平衡控制器计算足端所需的反力
 * 3. 使用逆动力学计算关节力矩
 * 
 * 算法基于全身动力学控制理论，是现代四足机器人的标准控制方法。
 */
void State_BalanceTest::calcTau(){

    // ==================== 计算期望的重心加速度 ====================
    // 使用PD控制器计算重心的期望加速度
    // _ddPcd = Kp * (期望位置 - 当前位置) + Kd * (期望速度 - 当前速度)
    // 这里期望速度设为0，因为平衡测试主要是位置保持
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);

    // ==================== 计算期望的角加速度 ====================
    // 使用PD控制器计算机体的期望角加速度
    // rotMatToExp函数将旋转矩阵差值转换为轴角表示（角度误差）
    // _dWbd = Kp * (期望姿态误差) + Kd * (期望角速度 - 当前角速度)
    // 期望角速度设为0，因为主要是姿态保持
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

    // ==================== 获取足端位置信息 ====================
    // 获取所有足端相对于机体重心的位置（在全局坐标系下表示）
    // 这个信息用于平衡控制器计算各足端应该施加的反力
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    // ==================== 平衡控制器计算足端反力 ====================
    // 调用平衡控制器，根据期望的重心加速度和角加速度计算各足端需要施加的反力
    // calF函数实现了全身动力学平衡控制算法，通常使用二次规划或加权最小二乘法
    // 输入：期望重心加速度、期望角加速度、旋转矩阵、足端位置、接触状态
    // 输出：各足端在全局坐标系下的反力
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    // ==================== 坐标变换：全局坐标系到机体坐标系 ====================
    // 将足端反力从全局坐标系转换到机体坐标系
    // 这是因为后续的逆动力学计算通常在机体坐标系下进行
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    // ==================== 逆动力学计算关节力矩 ====================
    // 获取当前的关节角度（从3x4矩阵转换为12x1向量格式）
    _q = vec34ToVec12(_lowState->getQ());
    
    // 使用机器人动力学模型的逆动力学功能，根据足端反力计算各关节所需的驱动力矩
    // getTau函数实现了雅可比转置或递归牛顿-欧拉算法
    // 输入：关节角度、足端反力（机体坐标系）
    // 输出：各关节的驱动力矩
    _tau = _robModel->getTau(_q, _forceFeetBody);
}