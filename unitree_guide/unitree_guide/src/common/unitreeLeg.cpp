/**********************************************************************
Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
* @file unitreeLeg.cpp
* @brief 四足机器人单腿运动学与动力学计算实现文件
* 
* 该文件实现了四足机器人单条腿部的运动学和动力学计算，主要功能包括：
* 1. 正运动学计算：由关节角度计算足端位置
* 2. 逆运动学计算：由足端期望位置计算关节角度
* 3. 速度运动学计算：关节角速度与足端速度之间的转换
* 4. 雅可比矩阵计算：关节空间与笛卡尔空间的映射关系
* 5. 动力学计算：由足端力计算关节力矩
* 
* 机器人腿部模型：
* - 每条腿有3个关节：髋外展关节(Hip Abad)、髋俯仰关节(Hip)、膝关节(Knee)
* - 使用改进的Denavit-Hartenberg参数描述连杆结构
* - 支持身体坐标系和髋关节坐标系的坐标变换
* 
* 坐标系定义：
* - 身体坐标系(BODY)：以机器人身体中心为原点
* - 髋关节坐标系(HIP)：以各腿髋关节中心为原点
* - 全局坐标系(GLOBAL)：世界坐标系
*/

#include "common/unitreeLeg.h"
#include <iostream>

/************************/
/*******QuadrupedLeg*****/
/************************/

/**
* @brief 四足机器人腿部构造函数
* @param legID 腿部编号 (0:右前, 1:左前, 2:右后, 3:左后)
* @param abadLinkLength 髋外展连杆长度 (m)
* @param hipLinkLength 髋关节连杆长度 (m) 
* @param kneeLinkLength 膝关节连杆长度 (m)
* @param pHip2B 髋关节相对于身体中心的位置向量
* 
* 构造函数主要完成：
* 1. 初始化腿部几何参数
* 2. 根据腿部编号确定左右侧标志(_sideSign)
* 3. 设置坐标系变换参数
* 
* 腿部编号约定：
* - 0: FR (Front Right) - 右前腿
* - 1: FL (Front Left)  - 左前腿  
* - 2: RR (Rear Right)  - 右后腿
* - 3: RL (Rear Left)   - 左后腿
* 
* 左右侧标志用于处理机器人左右对称的腿部结构差异
*/
QuadrupedLeg::QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                        float kneeLinkLength, Vec3 pHip2B)
            :_abadLinkLength(abadLinkLength), 
            _hipLinkLength(hipLinkLength), 
            _kneeLinkLength(kneeLinkLength), 
            _pHip2B(pHip2B){
    
    // 根据腿部编号设置左右侧标志
    // 右侧腿(FR, RR)设为-1，左侧腿(FL, RL)设为1
    // 这个标志用于处理左右腿在几何结构上的镜像对称关系
    if (legID == 0 || legID == 2)        // 右前腿和右后腿
        _sideSign = -1;
    else if (legID == 1 || legID == 3)   // 左前腿和左后腿  
        _sideSign = 1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }
}

/**
* @brief 正运动学计算：计算足端相对于髋关节的位置
* @param q 关节角度向量 [q0, q1, q2] (髋外展角, 髋俯仰角, 膝关节角)
* @return 足端相对于髋关节坐标系的位置向量 (m)
* 
* 该函数实现了三连杆机构的正运动学解析解：
* 1. 基于修改的DH参数建立运动学模型
* 2. 使用齐次变换矩阵方法计算足端位置
* 3. 考虑了左右腿的几何差异
* 
* 关节角度约定：
* - q(0): 髋外展关节角度，正值表示腿向外展开
* - q(1): 髋俯仰关节角度，正值表示大腿向前摆动  
* - q(2): 膝关节角度，正值表示小腿向后弯曲
* 
* 连杆长度约定：
* - l1: 髋外展连杆长度，带符号以区分左右腿
* - l2: 髋关节连杆长度，取负值符合DH约定
* - l3: 膝关节连杆长度，取负值符合DH约定
*/
Vec3 QuadrupedLeg::calcPEe2H(Vec3 q){
    // 定义连杆长度，考虑左右腿的几何差异
    float l1 = _sideSign * _abadLinkLength;  // 髋外展连杆，左右腿符号相反
    float l2 = -_hipLinkLength;              // 髋关节连杆
    float l3 = -_kneeLinkLength;             // 膝关节连杆

    // 计算关节角度的三角函数值
    float s1 = std::sin(q(0));  // sin(髋外展角)
    float s2 = std::sin(q(1));  // sin(髋俯仰角)
    float s3 = std::sin(q(2));  // sin(膝关节角)

    float c1 = std::cos(q(0));  // cos(髋外展角)
    float c2 = std::cos(q(1));  // cos(髋俯仰角)
    float c3 = std::cos(q(2));  // cos(膝关节角)

    // 计算复合角的三角函数值
    // c23 = cos(q1 + q2), s23 = sin(q1 + q2)
    // 这是髋关节和膝关节角度的合成角度
    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    Vec3 pEe2H;

    // 基于齐次变换矩阵推导的足端位置解析解
    // x方向：前后方向的位移
    pEe2H(0) = l3 * s23 + l2 * s2;
    
    // y方向：左右方向的位移
    pEe2H(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    
    // z方向：上下方向的位移
    pEe2H(2) = l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return pEe2H;
}

/**
* @brief 正运动学计算：计算足端相对于身体坐标系的位置
* @param q 关节角度向量
* @return 足端相对于身体坐标系的位置向量
* 
* 该函数通过坐标变换将足端从髋关节坐标系转换到身体坐标系：
* 足端身体系位置 = 髋关节身体系位置 + 足端髋关节系位置
*/
Vec3 QuadrupedLeg::calcPEe2B(Vec3 q){
    return _pHip2B + calcPEe2H(q);
}

/**
* @brief 微分正运动学：计算足端速度
* @param q 关节角度向量
* @param qd 关节角速度向量
* @return 足端在髋关节坐标系中的速度向量
* 
* 该函数使用雅可比矩阵将关节空间速度转换为笛卡尔空间速度：
* 足端速度 = 雅可比矩阵 × 关节角速度
* 
* 这是机器人运动控制中的关键计算，用于：
* 1. 足端轨迹跟踪控制
* 2. 步态生成中的速度规划
* 3. 动力学计算中的速度项
*/
Vec3 QuadrupedLeg::calcVEe(Vec3 q, Vec3 qd){
    return calcJaco(q) * qd;
}

/**
* @brief 逆运动学计算：由足端位置计算关节角度
* @param pEe 足端期望位置
* @param frame 坐标系类型 (HIP或BODY)
* @return 对应的关节角度向量
* 
* 该函数实现了三连杆机构的逆运动学解析解：
* 1. 根据指定坐标系进行坐标变换
* 2. 使用几何法求解关节角度
* 3. 按顺序求解：q1(髋外展) -> q3(膝关节) -> q2(髋俯仰)
* 
* 求解策略：
* - 利用几何约束条件
* - 使用三角恒等式和反三角函数
* - 确保解的唯一性和连续性
*/
Vec3 QuadrupedLeg::calcQ(Vec3 pEe, FrameType frame){
    Vec3 pEe2H;
    
    // 坐标系转换：将足端位置转换到髋关节坐标系
    if(frame == FrameType::HIP)
        pEe2H = pEe;                    // 已经是髋关节坐标系
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;         // 从身体坐标系转换到髋关节坐标系
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    float q1, q2, q3;      // 三个关节角度
    Vec3 qResult;          // 结果向量
    float px, py, pz;      // 足端位置分量
    float b2y, b3z, b4z;   // 连杆参数
    float a, b, c;         // 几何计算中间变量

    // 提取足端位置分量
    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    // 设置连杆参数
    b2y = _abadLinkLength * _sideSign;  // 髋外展连杆长度(带符号)
    b3z = -_hipLinkLength;              // 髋关节连杆长度
    b4z = -_kneeLinkLength;             // 膝关节连杆长度
    a = _abadLinkLength;                // 髋外展连杆长度(绝对值)
    
    // 几何计算
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // 髋关节到足端的直线距离
    b = sqrt(pow(c, 2) - pow(a, 2));                 // 肩关节到足端的距离

    // 分步求解关节角度
    q1 = q1_ik(py, pz, b2y);               // 求解髋外展关节角
    q3 = q3_ik(b3z, b4z, b);               // 求解膝关节角
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z); // 求解髋俯仰关节角

    // 组装结果向量
    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;

    return qResult;
}

/**
* @brief 微分逆运动学：由关节角度和足端速度计算关节角速度
* @param q 当前关节角度
* @param vEe 足端期望速度
* @return 对应的关节角速度向量
* 
* 该函数通过雅可比矩阵的逆将笛卡尔空间速度转换为关节空间速度：
* 关节角速度 = 雅可比逆矩阵 × 足端速度
*/
Vec3 QuadrupedLeg::calcQd(Vec3 q, Vec3 vEe){
    return calcJaco(q).inverse() * vEe;
}

/**
* @brief 微分逆运动学：由足端位置和速度计算关节角速度
* @param pEe 足端位置
* @param vEe 足端速度  
* @param frame 坐标系类型
* @return 对应的关节角速度向量
* 
* 该函数先通过逆运动学求解关节角度，再计算关节角速度
*/
Vec3 QuadrupedLeg::calcQd(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q = calcQ(pEe, frame);
    return calcJaco(q).inverse() * vEe;
}

/**
* @brief 逆动力学计算：由足端力计算关节力矩
* @param q 关节角度向量
* @param force 足端受到的外力
* @return 各关节需要输出的力矩向量
* 
* 该函数使用虚功原理计算关节力矩：
* 关节力矩 = 雅可比转置矩阵 × 足端力
* 
* 这是力控制的核心计算，用于：
* 1. 足端力跟踪控制
* 2. 阻抗控制和柔顺控制
* 3. 接触力控制
*/
Vec3 QuadrupedLeg::calcTau(Vec3 q, Vec3 force){
    return calcJaco(q).transpose() * force;
}

/**
* @brief 雅可比矩阵计算
* @param q 关节角度向量
* @return 3×3雅可比矩阵
* 
* 雅可比矩阵描述了关节角速度与足端速度之间的线性关系：
* [vx]   [J11 J12 J13] [q1_dot]
* [vy] = [J21 J22 J23] [q2_dot]
* [vz]   [J31 J32 J33] [q3_dot]
* 
* 雅可比矩阵的应用：
* 1. 速度运动学：v = J * qd
* 2. 静力学：tau = J^T * F
* 3. 奇异性分析：det(J) = 0时为奇异位形
* 4. 可操作性分析：机器人运动能力评估
*/
Mat3 QuadrupedLeg::calcJaco(Vec3 q){
    Mat3 jaco;

    // 连杆长度参数
    float l1 = _abadLinkLength * _sideSign;
    float l2 = -_hipLinkLength;
    float l3 = -_kneeLinkLength;

    // 关节角度的三角函数值
    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    // 复合角的三角函数值
    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    
    // 雅可比矩阵各元素
    // 第一列：对q1(髋外展角)的偏导数
    jaco(0, 0) = 0;                                    // ∂x/∂q1
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1; // ∂y/∂q1
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1; // ∂z/∂q1
    
    // 第二列：对q2(髋俯仰角)的偏导数
    jaco(0, 1) = l3 * c23 + l2 * c2;                  // ∂x/∂q2
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;       // ∂y/∂q2
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;      // ∂z/∂q2
    
    // 第三列：对q3(膝关节角)的偏导数
    jaco(0, 2) = l3 * c23;                            // ∂x/∂q3
    jaco(1, 2) = l3 * s1 * s23;                       // ∂y/∂q3
    jaco(2, 2) = -l3 * c1 * s23;                      // ∂z/∂q3

    return jaco;
}

/**
* @brief 逆运动学辅助函数：计算髋外展关节角度q1
* @param py 足端y坐标
* @param pz 足端z坐标  
* @param l1 髋外展连杆长度
* @return 髋外展关节角度
* 
* 基于几何约束求解髋外展关节角度：
* 利用足端在yz平面的投影和髋外展连杆的几何关系
* 使用atan2函数确保角度在正确的象限
*/
float QuadrupedLeg::q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

/**
* @brief 逆运动学辅助函数：计算膝关节角度q3
* @param b3z 髋关节连杆长度
* @param b4z 膝关节连杆长度
* @param b 肩关节到足端的距离
* @return 膝关节角度
* 
* 基于余弦定理求解膝关节角度：
* 在由髋关节、膝关节、足端构成的三角形中应用余弦定理
* 确保解在物理可行范围内 [-1, 1]
*/
float QuadrupedLeg::q3_ik(float b3z, float b4z, float b){
    float q3, temp;
    // 余弦定理：cos(q3) = (b3z² + b4z² - b²) / (2*|b3z|*|b4z|)
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    
    // 限制余弦值在有效范围内
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    
    q3 = acos(temp);
    q3 = -(M_PI - q3); // 转换到期望的角度范围 [0, π]
    return q3;
}

/**
* @brief 逆运动学辅助函数：计算髋俯仰关节角度q2
* @param q1 髋外展关节角度
* @param q3 膝关节角度
* @param px 足端x坐标
* @param py 足端y坐标
* @param pz 足端z坐标
* @param b3z 髋关节连杆长度
* @param b4z 膝关节连杆长度
* @return 髋俯仰关节角度
* 
* 在已知q1和q3的情况下，通过几何约束求解q2：
* 利用足端位置在经过髋外展变换后的坐标系中的投影关系
* 使用atan2函数求解角度
*/
float QuadrupedLeg::q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    // 计算经过髋外展变换后的坐标分量
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    
    // 计算膝关节位置的投影分量
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    
    // 使用atan2求解髋俯仰角
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}