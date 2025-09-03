/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

/**
* @brief State_Trotting构造函数
* @param ctrlComp 控制组件指针
* 
* 这是整个控制系统中最复杂的构造函数，需要初始化：
* 1. 步态生成器
* 2. 各种控制参数
* 3. 根据机器人类型设置PD增益
* 4. 设置运动限制
*/
State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    
    /**
    * 创建步态生成器实例
    * GaitGenerator是实现小跑步态的核心算法模块
    */
    _gait = new GaitGenerator(ctrlComp);

    /**
    * 设置默认步态抬腿高度
    * 0.08m是经过实验验证的合适高度：
    * - 足够高以跨越小障碍
    * - 不会太高影响步态效率
    */
    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    /**
    * Go1机器人的控制参数
    * 这些参数经过大量实验调优，适合Go1的硬件特性
    */
    _Kpp = Vec3(70, 70, 70).asDiagonal();           // 位置控制增益(XYZ方向)
    _Kdp = Vec3(10, 10, 10).asDiagonal();           // 位置阻尼增益
    _kpw = 780;                                     // 角位置控制增益 
    _Kdw = Vec3(70, 70, 70).asDiagonal();           // 角速度阻尼增益
    _KpSwing = Vec3(400, 400, 400).asDiagonal();    // 摆动腿位置增益
    _KdSwing = Vec3(10, 10, 10).asDiagonal();       // 摆动腿阻尼增益
#endif

#ifdef ROBOT_TYPE_A1
    /**
    * A1机器人的控制参数
    * A1较Go1更轻，因此使用较低的增益避免振荡
    */
    _Kpp = Vec3(20, 20, 100).asDiagonal();     // Z方向增益更高(抗重力)
    _Kdp = Vec3(20, 20, 20).asDiagonal();      
    _kpw = 400;                                // 较低的角位置增益
    _Kdw = Vec3(50, 50, 50).asDiagonal();      
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    /**
    * 从机器人模型获取速度限制
    * 不同机器人有不同的安全速度范围
    */
    _vxLim = _robModel->getRobVelLimitX();      // 前向速度限制
    _vyLim = _robModel->getRobVelLimitY();      // 侧向速度限制  
    _wyawLim = _robModel->getRobVelLimitYaw();  // 转向速度限制
}

/**
* @brief 析构函数
* 释放动态分配的步态生成器内存
*/
State_Trotting::~State_Trotting(){
    delete _gait;
}

/**
* @brief 进入小跑状态
* 
* 这是状态切换的关键函数，确保平滑过渡到运动状态
*/
void State_Trotting::enter(){
    /**
    * 初始化期望位置为当前位置
    * 这样做避免位置指令突跳，确保平滑启动
    */
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);  // 设置期望高度
    
    /**
    * 清零速度命令
    * 从静止状态开始，等待用户输入
    */
    _vCmdBody.setZero();
    
    /**
    * 初始化偏航角命令
    * 设置为当前朝向，避免突然转向
    */
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);                            // 计算期望旋转矩阵
    _wCmdGlobal.setZero();                          // 清零角速度命令

    /**
    * 重置控制面板和步态生成器
    * 确保控制系统从干净状态开始
    */
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

/**
* @brief 退出小跑状态
* 
* 清理和重置，为切换到其他状态做准备
*/
void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();    // 清零控制面板
    _ctrlComp->setAllSwing();              // 设置所有腿为摆动状态
}

/**
* @brief 检查状态切换条件
* @return FSMStateName 下一个状态
* 
* 小跑状态支持的退出路径较少，主要为安全考虑
*/
FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        /**
        * L2_B：紧急停止，回到被动状态
        * 这是最重要的安全退出机制
        */
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        /**
        * L2_A：停止运动，切换到固定站立
        * 用于从运动状态平滑过渡到静止状态
        */
        return FSMStateName::FIXEDSTAND;
    }
    else{
        /**
        * 其他情况：保持小跑状态
        * 这确保机器人在运动过程中的状态稳定性
        */
        return FSMStateName::TROTTING;
    }
}

/**
* @brief 主运行函数 - 核心控制循环
* 
* 这是整个控制系统的心脏，每500Hz执行一次
* 实现从传感器输入到电机输出的完整控制流程
*/
void State_Trotting::run(){
    /**
    * 第一步：获取机器人当前状态
    * 从状态估计器获取所有必要的状态信息
    */
    _posBody = _est->getPosition();                    // 机体位置
    _velBody = _est->getVelocity();                    // 机体速度
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();     // 足端相对位置
    _posFeetGlobal = _est->getFeetPos();               // 足端绝对位置
    _velFeetGlobal = _est->getFeetVel();               // 足端速度
    _B2G_RotMat = _lowState->getRotMat();              // 旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();             // 逆旋转矩阵
    _yaw = _lowState->getYaw();                        // 偏航角
    _dYaw = _lowState->getDYaw();                      // 偏航角速度

    /**
    * 第二步：获取用户输入
    * 从手柄或键盘获取运动命令
    */
    _userValue = _lowState->userValue;
    getUserCmd();

    /**
    * 第三步：计算运动指令
    * 将用户命令转换为机器人的运动目标
    */
    calcCmd();

    /**
    * 第四步：运行步态生成器
    * 根据速度命令和步态高度生成足端轨迹
    * 这是小跑步态的核心算法
    */
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    /**
    * 第五步：计算平衡控制力矩
    * 根据位置和速度误差计算所需的控制力矩
    */
    calcTau();

    /**
    * 第六步：计算关节角度和速度
    * 通过逆运动学将足端轨迹转换为关节指令
    */
    calcQQd();

    /**
    * 第七步：决定步态模式
    * 根据运动命令判断是否需要启动步态
    */
    if(checkStepOrNot()){
        _ctrlComp->setStartWave();    // 启动步态波形
    }else{
        _ctrlComp->setAllStance();    // 保持站立
    }

    /**
    * 第八步：输出控制命令
    * 将计算结果发送给底层控制器
    */
    _lowCmd->setTau(_tau);                          // 设置关节力矩
    _lowCmd->setQ(vec34ToVec12(_qGoal));           // 设置关节位置
    _lowCmd->setQd(vec34ToVec12(_qdGoal));         // 设置关节速度

    /**
    * 第九步：设置控制增益
    * 根据足端接触状态为每条腿设置不同的控制参数
    */
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);     // 摆动腿：位置控制
        }else{
            _lowCmd->setStableGain(i);    // 支撑腿：力控制
        }
    }
}

/**
* @brief 检查是否需要启动步态
* @return bool true-需要步态运动，false-可以站立不动
* 
* 这个函数实现智能步态切换，避免不必要的腿部运动
*/
bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||              // 前向速度命令
        (fabs(_vCmdBody(1)) > 0.03) ||              // 侧向速度命令
        (fabs(_posError(0)) > 0.08) ||              // 前向位置误差
        (fabs(_posError(1)) > 0.08) ||              // 侧向位置误差
        (fabs(_velError(0)) > 0.05) ||              // 前向速度误差
        (fabs(_velError(1)) > 0.05) ||              // 侧向速度误差
        (fabs(_dYawCmd) > 0.20) ){                  // 转向速度命令
        return true;    // 需要步态运动
    }else{
        return false;   // 可以静止站立
    }
}

/**
* @brief 设置高层运动命令
* @param vx 前向速度 (m/s)
* @param vy 侧向速度 (m/s)
* @param wz 偏航角速度 (rad/s)
* 
* 这个接口供外部模块（如ROS导航）调用
*/
void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;      // 设置前向速度
    _vCmdBody(1) = vy;      // 设置侧向速度
    _vCmdBody(2) = 0;       // Z方向速度保持为0
    _dYawCmd = wz;          // 设置转向速度
}

/**
* @brief 获取用户运动命令
* 
* 从手柄摇杆值转换为实际的速度命令
* 包括归一化、限制和滤波处理
*/
void State_Trotting::getUserCmd(){
    /**
    * 运动命令处理
    * 将手柄摇杆值(-1~1)转换为实际速度值
    */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));  // 前向
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));  // 侧向(取负号)
    _vCmdBody(2) = 0;                                                   // Z方向

    /**
    * 转向命令处理
    * 包括归一化和低通滤波
    */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    
    /**
    * 低通滤波：平滑转向命令
    * 滤波系数0.9提供良好的平滑效果，避免突然转向
    */
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

/**
* @brief 计算运动指令
* 
* 将机体坐标系下的速度命令转换为全局坐标系，并进行积分得到位置命令
* 同时处理转向命令和旋转矩阵更新
*/
void State_Trotting::calcCmd(){
    /**
    * 运动命令处理
    * 将机体坐标系的速度命令转换为全局坐标系
    */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    /**
    * 速度限制和平滑处理
    * 限制速度变化率，避免过大的加速度
    */
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    /**
    * 位置积分计算
    * 通过速度积分得到期望位置，同时限制位置偏差
    */
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, 
                        Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, 
                        Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;  // Z方向速度保持为0

    /**
    * 转向命令处理
    * 通过角速度积分得到期望偏航角
    */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    /**
    * 更新期望旋转矩阵和角速度命令
    */
    _Rd = rotz(_yawCmd);           // 根据期望偏航角计算旋转矩阵
    _wCmdGlobal(2) = _dYawCmd;     // 设置Z轴角速度
}

/**
* @brief 计算平衡控制力矩
* 
* 这是平衡控制的核心函数，实现机器人重心位置和姿态的稳定控制
* 基于经典的PD控制理论，结合四足机器人的动力学特性
*/
void State_Trotting::calcTau(){
    /**
    * 计算位置和速度误差
    * 这些误差是控制器的输入信号
    */
    _posError = _pcd - _posBody;                    // 位置误差
    _velError = _vCmdGlobal - _velBody;             // 速度误差

    /**
    * 计算期望线性加速度
    * 使用PD控制器：ddP = Kp*e_pos + Kd*e_vel
    */
    _ddPcd = _Kpp * _posError + _Kdp * _velError;

    /**
    * 计算期望角加速度
    * 基于旋转矩阵误差和角速度误差
    */
    _dWbd = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw*(_wCmdGlobal - _lowState->getGyroGlobal());

    /**
    * 调用平衡控制器计算足端力
    * 平衡控制器将期望加速度转换为各足端所需的支撑力
    */
    _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, _forceFeetGlobal);

    /**
    * 坐标变换：全局力转换为机体力
    * 便于后续的关节力矩计算
    */
    for(int i(0); i<4; ++i){
        _forceFeetBody.col(i) = _G2B_RotMat * _forceFeetGlobal.col(i);
    }

    /**
    * 计算关节力矩
    * 通过雅可比矩阵将足端力转换为关节力矩
    */
    _robModel->getJointTorque(_tau, _forceFeetBody);
}

/**
* @brief 计算关节角度和角速度指令
* 
* 通过逆运动学将足端目标位置转换为关节角度
* 这是运动学控制的核心部分
*/
void State_Trotting::calcQQd(){
    /**
    * 坐标变换：将全局足端目标转换为机体坐标系
    * 逆运动学需要在机体坐标系下进行计算
    */
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
    }

    /**
    * 逆运动学计算
    * 根据足端位置计算关节角度
    */
    _robModel->getQ(_posFeet2BGoal, _qGoal);
    
    /**
    * 雅可比矩阵计算关节速度
    * 根据足端速度计算关节角速度
    */
    _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal, _qdGoal);
}

// 注意：这里源码可能不完整，部分函数实现可能在其他地方
// calcBalanceKp() 函数可能是空实现或在其他文件中