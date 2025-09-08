/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
* @file GaitGenerator.cpp
* @brief 四足机器人步态生成器实现文件
* 
* 文件作用：
* 本文件实现了四足机器人的步态生成算法，是机器人运动控制系统的核心模块之一。
* 主要功能包括：
* 1. 根据速度指令和步态相位生成四条腿的足端轨迹
* 2. 使用摆线轨迹算法生成平滑的足端运动
* 3. 协调支撑腿和摆动腿的运动状态
* 4. 为上层控制器提供足端位置和速度信息
* 
* 该模块与WaveGenerator(波形生成器)配合工作：
* - WaveGenerator负责生成各腿的相位和接触状态
* - GaitGenerator根据相位信息生成具体的足端轨迹
* 
* 在整个控制系统中的位置：
* FSM状态机 -> State_Trotting -> GaitGenerator -> 足端轨迹 -> BalanceCtrl
*/

#include "Gait/GaitGenerator.h"

/**
* @brief 步态生成器构造函数
* @param ctrlComp 控制组件集合指针，包含系统所需的各种组件
* 
* 初始化步态生成器，建立与其他模块的连接：
* - 波形生成器：提供步态相位和接触状态
* - 状态估计器：提供当前足端位置信息  
* - 机器人模型：提供运动学计算
* - 足端计算器：计算目标足端位置
*/
GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
            : _waveG(ctrlComp->waveGen),        // 波形生成器，控制步态节拍
                _est(ctrlComp->estimator),        // 状态估计器，获取当前状态
                _phase(ctrlComp->phase),          // 步态相位指针（4条腿）
                _contact(ctrlComp->contact),      // 足端接触状态指针
                _robModel(ctrlComp->robotModel),  // 机器人模型
                _state(ctrlComp->lowState){       // 底层状态
    
    // 创建足端位置计算器实例
    // FeetEndCal负责计算理想的足端落点位置
    _feetCal = new FeetEndCal(ctrlComp);
    
    // 标记首次运行，用于初始化起始位置
    _firstRun = true;
}

/**
* @brief 析构函数
* 清理动态分配的内存
*/
GaitGenerator::~GaitGenerator(){
    // 注意：这里应该释放_feetCal的内存
    // delete _feetCal; (原代码中可能遗漏了这一行)
}

/**
* @brief 设置步态参数
* @param vxyGoalGlobal 期望的全局坐标系XY方向速度 [m/s]
* @param dYawGoal 期望的偏航角速度 [rad/s]
* @param gaitHeight 步态抬腿高度 [m]
* 
* 这是外部接口函数，通常由State_Trotting等上层状态调用
* 将用户的运动命令转换为步态生成器的内部参数
*/
void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;      // 存储期望XY速度（全局坐标系）
    _dYawGoal = dYawGoal;          // 存储期望偏航角速度
    _gaitHeight = gaitHeight;      // 存储抬腿高度（通常为0.08m）
}

/**
* @brief 重启步态生成器
* 
* 用于状态切换时重置步态生成器的内部状态
* 例如从静止状态切换到运动状态时调用
*/
void GaitGenerator::restart(){
    _firstRun = true;              // 重置首次运行标志
    _vxyGoal.setZero();           // 清零速度目标
}

/**
* @brief 步态生成器主运行函数
* @param feetPos 输出参数：四条腿的足端位置 [3x4矩阵，每列代表一条腿的XYZ位置]
* @param feetVel 输出参数：四条腿的足端速度 [3x4矩阵，每列代表一条腿的XYZ速度]
* 
* 这是步态生成的核心函数，每个控制周期（500Hz）调用一次
* 根据当前步态相位为每条腿生成相应的足端轨迹
*/
void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    // 首次运行时初始化起始位置
    if(_firstRun){
        // 从状态估计器获取当前足端位置作为轨迹起点
        // 这样可以避免轨迹突跳，确保平滑启动
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    // 遍历四条腿（i=0:前左腿, i=1:前右腿, i=2:后左腿, i=3:后右腿）
    for(int i(0); i<4; ++i){
        // 检查当前腿是否处于支撑状态（contact=1表示接触地面）
        if((*_contact)(i) == 1){
            // 支撑腿逻辑：保持与地面接触，不产生轨迹运动
            
            // 如果相位小于0.5（支撑前半段），更新起始位置
            // 这样做是为了在支撑后期准备下一步的摆动轨迹
            if((*_phase)(i) < 0.5){
                _startP.col(i) = _est->getFootPos(i);
            }
            
            // 支撑腿位置保持不变（与地面接触点固定）
            feetPos.col(i) = _startP.col(i);
            
            // 支撑腿速度为零（相对于地面静止）
            feetVel.col(i).setZero();
        }
        else{
            // 摆动腿逻辑：根据运动命令生成足端轨迹
            
            // 计算摆动腿的目标落点位置
            // FeetEndCal会根据速度命令、偏航角速度和当前相位计算理想落点
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            // 根据摆线轨迹算法计算当前足端位置
            feetPos.col(i) = getFootPos(i);
            
            // 根据摆线轨迹算法计算当前足端速度
            feetVel.col(i) = getFootVel(i);
        }
    }
    
    // 保存当前状态供下一周期使用
    _pastP = feetPos;           // 保存足端位置
    _phasePast = *_phase;       // 保存相位信息
}

/**
* @brief 计算指定腿的足端位置
* @param i 腿的索引 (0-3)
* @return Vec3 足端位置 [x, y, z]
* 
* 使用摆线轨迹算法计算摆动腿的足端位置
* 摆线轨迹的优点：
* 1. 起点和终点速度为零，运动平滑
* 2. 轨迹连续可导，适合控制
* 3. 计算简单，实时性好
*/
Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    // X方向位置：从起点到终点的摆线轨迹
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    
    // Y方向位置：从起点到终点的摆线轨迹  
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    
    // Z方向位置：抬腿轨迹，从起点高度经过最高点再回到地面
    footPos(2) = cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

/**
* @brief 计算指定腿的足端速度
* @param i 腿的索引 (0-3)
* @return Vec3 足端速度 [vx, vy, vz]
* 
* 通过对摆线轨迹求导得到足端速度
* 速度信息对于力控制和冲击控制很重要
*/
Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    // X方向速度：摆线轨迹的导数
    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    
    // Y方向速度：摆线轨迹的导数
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    
    // Z方向速度：抬腿轨迹的导数
    footVel(2) = cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

/**
* @brief 计算XY方向的摆线位置
* @param start 起始位置
* @param end 结束位置  
* @param phase 当前相位 [0,1]
* @return float 当前位置
* 
* 摆线轨迹方程：s(t) = (end-start) * (t - sin(t))/(2π) + start
* 其中 t = 2π * phase
* 
* 摆线的特点：
* - 在 phase=0 时，位置=start，速度=0
* - 在 phase=1 时，位置=end，速度=0  
* - 轨迹平滑，适合机器人运动
*/
float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;  // 将相位映射到 [0, 2π]
    
    // 摆线位置公式
    return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
}

/**
* @brief 计算XY方向的摆线速度
* @param start 起始位置
* @param end 结束位置
* @param phase 当前相位 [0,1] 
* @return float 当前速度
* 
* 摆线速度方程：v(t) = (end-start) * (1-cos(t)) / Tswing
* 其中 Tswing 是摆动时间
* 
* 注意：速度需要除以摆动时间来得到正确的量纲
*/
float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    
    // 摆线速度公式，除以摆动时间得到真实速度
    return (end - start) * (1 - cos(phasePI)) / _waveG->getTswing();
}

/**
* @brief 计算Z方向的摆线位置（抬腿轨迹）
* @param start 起始Z位置（通常接近地面）
* @param h 最大抬腿高度
* @param phase 当前相位 [0,1]
* @return float 当前Z位置
* 
* Z方向使用余弦抬腿轨迹：z(t) = h*(1-cos(t))/2 + start
* 
* 轨迹特点：
* - phase=0: z=start (贴地)
* - phase=0.5: z=start+h (最高点)  
* - phase=1: z=start (回到地面)
* - 运动平滑，避免冲击
*/
float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    
    // 余弦抬腿轨迹公式
    return h * (1 - cos(phasePI)) / 2 + start;
}

/**
* @brief 计算Z方向的摆线速度（抬腿速度）
* @param h 最大抬腿高度
* @param phase 当前相位 [0,1]
* @return float 当前Z方向速度
* 
* Z方向速度：vz(t) = h*π*sin(t) / Tswing
* 
* 速度特点：
* - phase=0: vz=0 (静止离地)
* - phase=0.5: vz=0 (最高点暂停)
* - phase=1: vz=0 (平稳着地)
* - 中间过程速度连续变化
*/
float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    
    // Z方向摆线速度公式
    return h * M_PI * sin(phasePI) / _waveG->getTswing();
}