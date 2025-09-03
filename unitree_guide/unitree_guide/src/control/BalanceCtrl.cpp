/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
* @file BalanceCtrl.cpp
* @brief 四足机器人平衡控制器实现
* 
* 文件作用：
* 该文件实现了四足机器人的平衡控制器(BalanceCtrl类)，是整个控制系统的核心组件之一。
* 主要功能包括：
* 1. 🎯 力分配优化 - 通过二次规划(QP)优化分配四条腿的接触力
* 2. 🏋️ 平衡维持 - 实现机器人重心控制和姿态稳定
* 3. 🦶 足端力约束 - 处理摩擦锥约束和单边接触约束
* 4. ⚖️ 动力学平衡 - 满足机器人的力和力矩平衡方程
* 
* 在整个控制系统中的作用：
* - 接收来自上层控制器的期望加速度命令(线性和角度)
* - 根据当前足端接触状态，计算最优的足端反力分配
* - 为下层的关节力矩控制提供力参考值
* - 确保机器人在运动过程中保持稳定和平衡
* 
* 算法原理：
* 基于虚功原理和动力学约束，将足端力分配问题转化为带约束的二次规划问题，
* 通过最小化控制代价和跟踪误差来求解最优的足端接触力。
*/

#include "control/BalanceCtrl.h"
#include "common/mathTools.h"
#include "common/timeMarker.h"

/**
* @brief 平衡控制器构造函数（参数版本）
* @param mass 机器人质量 [kg]
* @param Ib 机器人惯性矩阵 [kg⋅m²]（相对于质心）
* @param S 权重矩阵 [6×6] 用于平衡约束的权重
* @param alpha 正则化参数，控制力的平滑性
* @param beta 时间连续性参数，避免力的突变
*/
BalanceCtrl::BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta)
            : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
    
    // 初始化上一时刻的足端力为零向量（12维：4条腿×3个力分量）
    _Fprev.setZero();
    
    // 设置重力向量 [0, 0, -9.81] m/s²（Z轴向下为负）
    _g << 0, 0, -9.81;
    
    // 设置摩擦系数（静摩擦系数，用于摩擦锥约束）
    _fricRatio = 0.3;
    
    // 构建摩擦锥约束矩阵 [5×3]
    // 每一行代表一个不等式约束：确保足端力在摩擦锥内
    _fricMat <<  1,  0, _fricRatio,   // Fx ≤ μ*Fz （前向摩擦约束）
                -1,  0, _fricRatio,   // -Fx ≤ μ*Fz（后向摩擦约束）
                0,  1, _fricRatio,   // Fy ≤ μ*Fz （右向摩擦约束）
                0, -1, _fricRatio,   // -Fy ≤ μ*Fz（左向摩擦约束）
                0,  0, 1;            // Fz ≥ 0    （单边接触约束）
}

/**
* @brief 平衡控制器构造函数（机器人模型版本）
* @param robModel 机器人模型指针，包含物理参数和几何信息
* 
* 该构造函数从机器人模型中自动提取参数，设置合适的权重和约束
*/
BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel){
    Vec6 s;      // 平衡约束权重向量
    Vec12 w, u;  // 力权重和正则化权重向量

    // 从机器人模型获取物理参数
    _mass = robModel->getRobMass();           // 机器人总质量
    _pcb = robModel->getPcb();                // 质心相对于机体坐标系的位置
    _Ib = robModel->getRobInertial();         // 机器人惯性矩阵
    
    // 重力向量
    _g << 0, 0, -9.81;

    // 设置力权重向量 w [12×1]：每条腿的xyz三个方向的力权重
    // 较小的权重意味着该方向的力更容易被激活
    w << 10, 10, 4,    // 前左腿：x,y方向权重较大，z方向权重较小
        10, 10, 4,    // 前右腿：优先使用垂直方向的力
        10, 10, 4,    // 后左腿：水平方向力的代价较高
        10, 10, 4;    // 后右腿：鼓励使用支撑力

    // 设置正则化权重向量 u [12×1]：所有方向使用相同权重
    // 用于平滑力的变化，避免力的剧烈跳变
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;

    // QP优化参数设置
    _alpha = 0.001;  // 力大小的正则化参数（较小：允许较大的力）
    _beta  = 0.1;    // 时间连续性参数（较大：强制力平滑变化）

    // 摩擦系数（适中的值，适合大多数地面）
    _fricRatio = 0.4;

    // 平衡约束权重向量 s [6×1]
    // 前3个：线性运动约束权重，后3个：角度运动约束权重
    s << 20, 20, 50,        // x,y,z线性运动：z方向(垂直)权重最高
        450, 450, 450;     // roll,pitch,yaw角度运动：姿态控制权重很高

    // 构建对角权重矩阵
    _S = s.asDiagonal();    // 6×6 平衡约束权重矩阵
    _W = w.asDiagonal();    // 12×12 力权重矩阵  
    _U = u.asDiagonal();    // 12×12 正则化权重矩阵
    
    // 初始化历史力向量
    _Fprev.setZero();
    
    // 摩擦锥约束矩阵（与上面构造函数相同）
    _fricMat <<  1,  0, _fricRatio,   
                -1,  0, _fricRatio,   
                0,  1, _fricRatio,   
                0, -1, _fricRatio,   
                0,  0, 1;            
}

/**
* @brief 计算最优足端接触力的主函数
* @param ddPcd 期望的质心线性加速度 [3×1] [m/s²]
* @param dWbd 期望的机体角加速度 [3×1] [rad/s²]  
* @param rotM 当前机体旋转矩阵 [3×3]（从机体坐标系到世界坐标系）
* @param feetPos2B 四条腿足端相对于机体的位置 [3×4] [m]
* @param contact 足端接触状态 [4×1]（1表示接触，0表示离地）
* @return Vec34 计算得到的足端接触力 [3×4] [N]（世界坐标系）
* 
* 该函数是平衡控制的核心，通过以下步骤计算最优力分配：
* 1. 构建动力学约束矩阵A（力到加速度的映射）
* 2. 计算期望的合外力和合外力矩向量bd  
* 3. 根据接触状态设置约束条件
* 4. 求解二次规划优化问题
* 5. 返回优化后的足端接触力
*/
Vec34 BalanceCtrl::calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact){
    
    // 步骤1：构建动力学约束矩阵A [6×12]
    // A矩阵将12维的足端力向量映射为6维的合外力和合外力矩
    calMatrixA(feetPos2B, rotM, contact);
    
    // 步骤2：计算期望的合外力和合外力矩向量bd [6×1]  
    // bd = [Fd; τd] 其中Fd是合外力，τd是合外力矩
    calVectorBd(ddPcd, dWbd, rotM);
    
    // 步骤3：根据接触状态设置等式和不等式约束
    // 离地的腿：等式约束（力为0）
    // 接触的腿：不等式约束（摩擦锥约束）
    calConstraints(contact);

    // 步骤4：构建QP优化问题的目标函数
    // 最小化：(1/2)*F^T*G*F + g0^T*F
    // 其中G是Hessian矩阵，g0是梯度向量
    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;  // Hessian矩阵[12×12]
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;  // 梯度向量[1×12]

    // 步骤5：求解二次规划问题
    solveQP();

    // 步骤6：保存当前力作为下一时刻的历史力（用于平滑化）
    _Fprev = _F;
    
    // 步骤7：将12维力向量转换为3×4矩阵格式并返回
    return vec12ToVec34(_F);
}

/**
* @brief 构建动力学约束矩阵A
* @param feetPos2B 足端相对机体的位置 [3×4]
* @param rotM 机体旋转矩阵 [3×3]  
* @param contact 接触状态 [4×1]（此参数在当前实现中未使用）
* 
* 动力学约束矩阵A [6×12] 描述了足端力如何产生合外力和合外力矩：
* A * F = [ΣFi; Σri×Fi]
* 
* 矩阵结构：
* A = [I3, I3, I3, I3;           <- 前3行：合外力约束（每条腿贡献相同）
*      [r1×], [r2×], [r3×], [r4×]] <- 后3行：合外力矩约束（ri是足端到质心的向量）
*/
void BalanceCtrl::calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact){
    for(int i(0); i < 4; ++i){
        // 前3行：合外力约束 - 每条腿的力直接累加
        // A[0:3, 3*i:3*i+3] = I3 (3×3单位矩阵)
        _A.block(0, 3*i, 3, 3) = I3;
        
        // 后3行：合外力矩约束 - 每条腿的力对质心产生的力矩
        // A[3:6, 3*i:3*i+3] = skew(ri - pcm)
        // 其中：ri是第i条腿的足端位置，pcm是质心位置
        // skew(v)是向量v的反对称矩阵，用于计算叉积 v×F
        _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i) - rotM*_pcb);
    }
}

/**
* @brief 计算期望的合外力和合外力矩向量
* @param ddPcd 期望质心线性加速度 [3×1]
* @param dWbd 期望机体角加速度 [3×1] 
* @param rotM 当前机体旋转矩阵 [3×3]
* 
* 根据牛顿-欧拉方程计算期望的合外力和合外力矩：
* Fd = m*(ddPcd - g)                    <- 合外力
* τd = (R*Ib*R^T)*dWbd                  <- 合外力矩（考虑旋转后的惯性矩阵）
*/
void BalanceCtrl::calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM){
    // 计算期望合外力：质量×(期望加速度 - 重力加速度)
    // 重力加速度已经包含在_g向量中，所以减去它来消除重力影响
    _bd.head(3) = _mass * (ddPcd - _g);
    
    // 计算期望合外力矩：惯性矩阵×期望角加速度
    // 需要将机体坐标系的惯性矩阵转换到世界坐标系：R*Ib*R^T
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
}

/**
* @brief 根据接触状态设置QP优化的约束条件
* @param contact 足端接触状态 [4×1]（1=接触地面，0=离地）
* 
* 该函数设置两类约束：
* 1. 等式约束（CE, ce0）：离地腿的力必须为0
* 2. 不等式约束（CI, ci0）：接触腿必须满足摩擦锥约束
* 
* 约束数量：
* - 等式约束：3×(离地腿数) 个
* - 不等式约束：5×(接触腿数) 个
*/
void BalanceCtrl::calConstraints(VecInt4 contact){
    int contactLegNum = 0;  // 统计接触地面的腿数
    
    // 统计接触的腿数
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }

    // 调整约束矩阵大小
    _CI.resize(5*contactLegNum, 12);        // 不等式约束矩阵 [5*接触腿数 × 12]
    _ci0.resize(5*contactLegNum);           // 不等式约束向量
    _CE.resize(3*(4 - contactLegNum), 12);  // 等式约束矩阵 [3*离地腿数 × 12]
    _ce0.resize(3*(4 - contactLegNum));     // 等式约束向量

    // 初始化约束矩阵和向量为零
    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;  // 等式约束计数器
    int ciID = 0;  // 不等式约束计数器
    
    // 遍历四条腿，根据接触状态设置对应的约束
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            // 接触的腿：设置摩擦锥不等式约束
            // _fricMat * Fi ≤ 0 (摩擦锥约束)
            // CI * F ≤ ci0，这里ci0 = 0
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            ++ciID;
        }else{
            // 离地的腿：设置等式约束Fi = 0
            // CE * F = ce0，这里ce0 = 0
            _CE.block(3*ceID, 3*i, 3, 3) = I3;  // 3×3单位矩阵
            ++ceID;
        }
    }
}

/**
* @brief 求解二次规划优化问题
* 
* 优化问题形式：
* minimize:   (1/2) * F^T * G * F + g0^T * F
* subject to: CE^T * F = ce0  (等式约束)
*             CI^T * F ≤ ci0  (不等式约束)
* 
* 该函数使用quadprog++库求解QP问题，步骤如下：
* 1. 将Eigen矩阵转换为quadprog++格式
* 2. 调用solve_quadprog求解器  
* 3. 将解转换回Eigen格式
*/
void BalanceCtrl::solveQP(){
    // 获取问题维度
    int n = _F.size();      // 优化变量维度（12）
    int m = _ce0.size();    // 等式约束数量
    int p = _ci0.size();    // 不等式约束数量

    // 调整quadprog++库的矩阵和向量大小
    G.resize(n, n);     // Hessian矩阵
    CE.resize(n, m);    // 等式约束矩阵（转置）
    CI.resize(n, p);    // 不等式约束矩阵（转置）
    g0.resize(n);       // 梯度向量
    ce0.resize(m);      // 等式约束向量
    ci0.resize(p);      // 不等式约束向量
    x.resize(n);        // 求解结果

    // 步骤1：将Eigen矩阵转换为quadprog++格式
    
    // 复制Hessian矩阵G（对称矩阵）
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    // 复制等式约束矩阵CE（注意：quadprog++需要转置矩阵）
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }
    }

    // 复制不等式约束矩阵CI（注意：quadprog++需要转置矩阵）
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    // 复制梯度向量
    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    // 复制约束向量
    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    // 步骤2：调用QP求解器
    // solve_quadprog返回目标函数的最优值
    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    // 步骤3：将解复制回_F向量
    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}

/**
* 总结：BalanceCtrl类的工作流程
* 
* 1. 初始化阶段：
*    - 设置机器人物理参数（质量、惯性矩阵等）
*    - 配置优化权重和约束参数
*    - 初始化摩擦锥约束矩阵
* 
* 2. 实时控制阶段（每个控制周期调用calF）：
*    - 接收期望的加速度指令（线性+角度）
*    - 根据当前机器人状态构建动力学约束
*    - 设置足端接触约束（摩擦锥+单边接触）  
*    - 通过QP优化求解最优足端力分配
*    - 返回优化后的足端接触力供下层使用
* 
* 3. 关键算法思想：
*    - 平衡约束：确保足端力的合成满足期望的机体运动
*    - 物理约束：足端力必须满足摩擦锥和单边接触限制
*    - 优化目标：最小化控制代价，保证力的平滑连续
*    - 接触处理：根据步态自动切换约束条件
* 
* 该控制器是四足机器人平衡控制的核心，为上层步态规划和下层关节控制
* 之间提供了重要的力学接口，确保机器人能够稳定地执行各种运动任务。
*/