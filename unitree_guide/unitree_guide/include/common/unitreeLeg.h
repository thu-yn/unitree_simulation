/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
 * @file unitreeLeg.h
 * @brief 宇树科技四足机器人腿部运动学库
 * 
 * 【文件作用总览】
 * 本文件定义了四足机器人腿部的运动学计算类，是整个四足机器人控制系统中的核心组件之一。
 * 主要功能包括：
 * 
 * 1. **正向运动学计算**：根据关节角度计算末端执行器（脚尖）的位置
 *    - 相对于髋关节的位置计算 (calcPEe2H)
 *    - 相对于机器人本体的位置计算 (calcPEe2B)
 * 
 * 2. **反向运动学计算**：根据期望的脚尖位置计算所需的关节角度
 *    - 支持不同参考坐标系 (calcQ)
 *    - 采用几何解析方法，计算效率高
 * 
 * 3. **速度运动学计算**：处理关节空间和笛卡尔空间的速度转换
 *    - 脚尖速度计算 (calcVEe)
 *    - 关节速度计算 (calcQd)
 *    - 雅可比矩阵计算 (calcJaco)
 * 
 * 4. **静力学计算**：力和力矩的转换
 *    - 根据脚尖受力计算关节扭矩 (calcTau)
 * 
 * 5. **机型适配**：支持不同宇树机器人型号
 *    - A1腿部运动学 (A1Leg)
 *    - Go1腿部运动学 (Go1Leg)
 *    - 可扩展支持其他机型
 * 
 * 【技术特点】
 * - 使用几何解析方法求解运动学，计算速度快，数值稳定性好
 * - 支持多种坐标系转换，适应不同控制需求
 * - 模块化设计，基类提供通用算法，派生类提供机型参数
 * - 广泛应用于步态规划、足力控制、位置控制等模块
 * 
 * 【应用场景】
 * - 步态生成器：计算步行过程中各腿的运动轨迹
 * - 足力控制器：将期望的地面反作用力转换为关节扭矩
 * - 位置控制器：实现脚尖的精确定位
 * - 运动规划器：进行碰撞检测和路径优化
 * - 状态估计器：根据关节状态估计机器人姿态
 */

 #ifndef UNITREELEG_H
 #define UNITREELEG_H
 
 #include "common/mathTypes.h"   // 数学类型定义（Vec3, Mat3等）
 #include "common/enumClass.h"   // 枚举类型定义（FrameType等）
 
 /**
  * @brief 四足机器人腿部运动学基类
  * 
  * 该类实现了四足机器人单腿的完整运动学计算功能，包括正向运动学、
  * 反向运动学、速度运动学和静力学计算。采用标准的3-DOF串联机构模型：
  * 
  * 关节配置：
  * - Joint 1 (q1): 髋关节外展/内收 (Hip Abduction/Adduction)
  * - Joint 2 (q2): 髋关节屈曲/伸展 (Hip Flexion/Extension) 
  * - Joint 3 (q3): 膝关节屈曲/伸展 (Knee Flexion/Extension)
  * 
  * 坐标系定义：
  * - H坐标系：髋关节坐标系，原点在髋关节转动中心
  * - B坐标系：机器人本体坐标系，原点在机器人本体中心
  * - 末端执行器：脚尖，记为Ee (End Effector)
  * 
  * 连杆参数：
  * - abadLinkLength: 髋关节外展连杆长度
  * - hipLinkLength: 大腿连杆长度
  * - kneeLinkLength: 小腿连杆长度
  */
 class QuadrupedLeg{
 public:
     /**
      * @brief 构造函数 - 初始化腿部运动学参数
      * 
      * @param legID 腿部ID标识
      *               - 0: 前右腿 (Front Right)
      *               - 1: 前左腿 (Front Left) 
      *               - 2: 后右腿 (Rear Right)
      *               - 3: 后左腿 (Rear Left)
      * 
      * @param abadLinkLength 髋关节外展连杆长度 (单位：米)
      *                       从机器人本体到髋关节转动中心的距离
      * 
      * @param hipLinkLength 大腿连杆长度 (单位：米)
      *                      从髋关节到膝关节的距离
      * 
      * @param kneeLinkLength 小腿连杆长度 (单位：米)
      *                       从膝关节到脚尖的距离
      * 
      * @param pHip2B 髋关节相对于机器人本体的位置向量 (单位：米)
      *               在机器人本体坐标系B中的坐标表示
      * 
      * 构造函数会根据腿部ID自动设置侧向符号：
      * - 右侧腿部 (ID=0,2): _sideSign = -1
      * - 左侧腿部 (ID=1,3): _sideSign = +1
      */
     QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                  float kneeLinkLength, Vec3 pHip2B);
     
     /**
      * @brief 析构函数
      * 
      * 默认析构函数，无需特殊清理工作
      */
     ~QuadrupedLeg(){}
     
     /**
      * @brief 正向运动学 - 计算脚尖相对于髋关节的位置
      * 
      * @param q 关节角度向量 [q1, q2, q3] (单位：弧度)
      * @return Vec3 脚尖在髋关节坐标系H中的位置向量 (单位：米)
      * 
      * 算法说明：
      * 基于D-H参数和几何关系，通过齐次变换矩阵链式相乘得到。
      * 计算过程包括每个关节的旋转变换和平移变换。
      * 
      * 使用场景：
      * - 验证反向运动学解的正确性
      * - 碰撞检测和工作空间分析
      * - 轨迹规划中的中间结果验证
      */
     Vec3 calcPEe2H(Vec3 q);
     
     /**
      * @brief 正向运动学 - 计算脚尖相对于机器人本体的位置
      * 
      * @param q 关节角度向量 [q1, q2, q3] (单位：弧度)
      * @return Vec3 脚尖在机器人本体坐标系B中的位置向量 (单位：米)
      * 
      * 计算公式：
      * pEe2B = pHip2B + pEe2H(q)
      * 
      * 其中pHip2B是髋关节到本体的固定偏移向量。
      * 
      * 使用场景：
      * - 步态规划中的脚尖轨迹生成
      * - 全身运动控制
      * - 地形适应性分析
      */
     Vec3 calcPEe2B(Vec3 q);
     
     /**
      * @brief 速度运动学 - 计算脚尖速度
      * 
      * @param q 关节角度向量 [q1, q2, q3] (单位：弧度)
      * @param qd 关节角速度向量 [qd1, qd2, qd3] (单位：弧度/秒)
      * @return Vec3 脚尖线速度向量 (单位：米/秒)
      * 
      * 计算公式：
      * vEe = J(q) * qd
      * 
      * 其中J(q)是几何雅可比矩阵，建立了关节空间速度和笛卡尔空间速度的线性关系。
      * 
      * 使用场景：
      * - 速度控制器的前馈补偿
      * - 动力学建模中的速度项计算
      * - 能量效率分析
      */
     Vec3 calcVEe(Vec3 q, Vec3 qd);
     
     /**
      * @brief 反向运动学 - 根据脚尖位置计算关节角度
      * 
      * @param pEe 脚尖期望位置向量 (单位：米)
      * @param frame 参考坐标系类型
      *              - FrameType::HIP: 髋关节坐标系H
      *              - FrameType::BODY: 机器人本体坐标系B
      * @return Vec3 关节角度解 [q1, q2, q3] (单位：弧度)
      * 
      * 算法特点：
      * - 采用几何解析方法，非迭代求解
      * - 计算速度快，适合实时控制
      * - 自动选择合理的解（通常选择膝关节向后弯曲的解）
      * 
      * 注意事项：
      * - 如果目标位置超出工作空间，可能返回边界解
      * - 奇异位置附近的解可能不稳定
      * 
      * 使用场景：
      * - 轨迹跟踪控制
      * - 步态生成器的关节指令计算
      * - 位置伺服控制
      */
     Vec3 calcQ(Vec3 pEe, FrameType frame);
     
     /**
      * @brief 反向速度运动学 - 根据关节位置和脚尖速度计算关节角速度
      * 
      * @param q 当前关节角度向量 [q1, q2, q3] (单位：弧度)
      * @param vEe 脚尖期望速度向量 (单位：米/秒)
      * @return Vec3 关节角速度解 [qd1, qd2, qd3] (单位：弧度/秒)
      * 
      * 计算公式：
      * qd = J^(-1)(q) * vEe
      * 
      * 其中J^(-1)是雅可比矩阵的逆矩阵。
      * 
      * 使用场景：
      * - 速度控制模式下的关节指令生成
      * - 轨迹跟踪的速度前馈
      * - 阻抗控制中的速度计算
      */
     Vec3 calcQd(Vec3 q, Vec3 vEe);
     
     /**
      * @brief 反向速度运动学 - 根据脚尖位置和速度计算关节角速度
      * 
      * @param pEe 脚尖位置向量 (单位：米)
      * @param vEe 脚尖速度向量 (单位：米/秒)
      * @param frame 参考坐标系类型
      * @return Vec3 关节角速度解 [qd1, qd2, qd3] (单位：弧度/秒)
      * 
      * 该函数是上述calcQd的重载版本，首先通过反向运动学求解关节角度，
      * 然后计算关节角速度。
      * 
      * 适用于只知道脚尖期望位置和速度，但不知道当前关节角度的情况。
      */
     Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
     
     /**
      * @brief 静力学计算 - 根据脚尖受力计算关节扭矩
      * 
      * @param q 关节角度向量 [q1, q2, q3] (单位：弧度)
      * @param force 脚尖受力向量 [fx, fy, fz] (单位：牛顿)
      * @return Vec3 关节扭矩向量 [tau1, tau2, tau3] (单位：牛顿·米)
      * 
      * 计算公式：
      * tau = J^T(q) * force
      * 
      * 其中J^T是雅可比矩阵的转置，这是虚功原理的直接应用。
      * 
      * 物理意义：
      * - 将笛卡尔空间的力映射到关节空间的扭矩
      * - 考虑了机构的几何配置对力传递的影响
      * 
      * 使用场景：
      * - 足力控制器的扭矩指令计算
      * - 平衡控制中的支撑力转换
      * - 动力学仿真中的外力处理
      */
     Vec3 calcTau(Vec3 q, Vec3 force);
     
     /**
      * @brief 计算几何雅可比矩阵
      * 
      * @param q 关节角度向量 [q1, q2, q3] (单位：弧度)
      * @return Mat3 3x3雅可比矩阵
      * 
      * 雅可比矩阵的物理意义：
      * - 描述关节空间速度与笛卡尔空间速度的线性映射关系
      * - 第i行表示脚尖第i个方向的速度对各关节角速度的偏导数
      * - 第j列表示第j个关节角速度对脚尖各方向速度的贡献
      * 
      * 奇异性分析：
      * - 当det(J) = 0时，机构处于奇异位置
      * - 奇异位置附近雅可比矩阵病态，速度和力的转换不稳定
      * 
      * 使用场景：
      * - 速度运动学计算的中间结果
      * - 静力学计算的中间结果  
      * - 机构奇异性分析
      * - 可操作性分析
      */
     Mat3 calcJaco(Vec3 q);
     
     /**
      * @brief 获取髋关节相对于机器人本体的位置向量
      * 
      * @return Vec3 髋关节位置向量 (单位：米)
      * 
      * 该函数返回构造函数中设置的固定偏移向量，用于坐标系转换。
      */
     Vec3 getHip2B(){return _pHip2B;}
 
 protected:
     /**
      * @brief 反向运动学求解 - 计算q1关节角度
      * 
      * @param py 脚尖y坐标 (单位：米)
      * @param pz 脚尖z坐标 (单位：米) 
      * @param b2y 中间计算参数
      * @return float q1关节角度 (单位：弧度)
      * 
      * 几何原理：
      * 髋关节外展角度主要由脚尖的y和z坐标确定，采用几何投影方法求解。
      */
     float q1_ik(float py, float pz, float b2y);
     
     /**
      * @brief 反向运动学求解 - 计算q3关节角度  
      * 
      * @param b3z 中间计算参数
      * @param b4z 中间计算参数
      * @param b 中间计算参数
      * @return float q3关节角度 (单位：弧度)
      * 
      * 几何原理：
      * 膝关节角度通过余弦定理求解，基于大腿和小腿长度以及脚尖到髋关节的距离。
      */
     float q3_ik(float b3z, float b4z, float b);
     
     /**
      * @brief 反向运动学求解 - 计算q2关节角度
      * 
      * @param q1 已求解的q1关节角度 (单位：弧度)
      * @param q3 已求解的q3关节角度 (单位：弧度)
      * @param px 脚尖x坐标 (单位：米)
      * @param py 脚尖y坐标 (单位：米)
      * @param pz 脚尖z坐标 (单位：米)
      * @param b3z 中间计算参数
      * @param b4z 中间计算参数
      * @return float q2关节角度 (单位：弧度)
      * 
      * 几何原理：
      * 髋关节屈曲角度需要综合考虑已求解的q1和q3，以及脚尖的三维坐标。
      */
     float q2_ik(float q1, float q3, float px, 
                 float py, float pz, float b3z, float b4z);
     
     /**
      * @brief 侧向符号，用于区分左右腿
      * 
      * - +1: 左侧腿部 (legID = 1, 3)
      * - -1: 右侧腿部 (legID = 0, 2)
      * 
      * 该符号用于处理左右腿的几何对称性，确保运动学计算的正确性。
      */
     float _sideSign;
     
     /**
      * @brief 连杆长度参数 (单位：米)
      * 
      * 这些参数定义了腿部的几何结构，在构造函数中设置为常量，
      * 不同机器人型号具有不同的参数值。
      */
     const float _abadLinkLength,    // 髋关节外展连杆长度
                 _hipLinkLength,     // 大腿连杆长度  
                 _kneeLinkLength;    // 小腿连杆长度
     
     /**
      * @brief 髋关节相对于机器人本体的位置向量 (单位：米)
      * 
      * 该向量描述了髋关节在机器人本体坐标系中的固定位置，
      * 用于H坐标系和B坐标系之间的转换。
      */
     const Vec3 _pHip2B;
 };
 
 /**
  * @brief A1机器人腿部运动学类
  * 
  * A1是宇树科技的经典四足机器人型号，具有以下特点：
  * - 中等体型，适合研究和教学
  * - 载重能力适中
  * - 运动性能均衡
  * 
  * 几何参数：
  * - 髋关节外展连杆长度：83.8mm
  * - 大腿长度：200mm  
  * - 小腿长度：200mm
  * 
  * 这些参数经过优化，兼顾了工作空间大小、载重能力和运动灵活性。
  */
 class A1Leg : public QuadrupedLeg{
 public:
     /**
      * @brief A1腿部构造函数
      * 
      * @param legID 腿部ID (0-3)
      * @param pHip2B 髋关节相对于本体的位置向量
      * 
      * 构造函数直接调用基类构造函数，传入A1机器人的标准几何参数。
      * 参数值基于A1机器人的实际机械设计和制造规格。
      */
     A1Leg(const int legID, const Vec3 pHip2B):
         QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
     
     /**
      * @brief A1腿部析构函数
      * 
      * 使用默认析构函数，基类会自动处理资源清理。
      */
     ~A1Leg(){}
 };
 
 /**
  * @brief Go1机器人腿部运动学类
  * 
  * Go1是宇树科技的新一代四足机器人，相比A1有以下改进：
  * - 更紧凑的设计
  * - 更高的载重比
  * - 优化的运动性能
  * - 更好的量产性
  * 
  * 几何参数：
  * - 髋关节外展连杆长度：80mm
  * - 大腿长度：213mm
  * - 小腿长度：213mm
  * 
  * Go1的腿部设计针对商业应用进行了优化，在保持高性能的同时降低了制造成本。
  */
 class Go1Leg : public QuadrupedLeg{
 public:
     /**
      * @brief Go1腿部构造函数
      * 
      * @param legID 腿部ID (0-3)  
      * @param pHip2B 髋关节相对于本体的位置向量
      * 
      * 传入Go1机器人的标准几何参数。相比A1，Go1的连杆更细长，
      * 这种设计提供了更大的步长和更好的地形适应能力。
      */
     Go1Leg(const int legID, const Vec3 pHip2B):
         QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
     
     /**
      * @brief Go1腿部析构函数
      * 
      * 使用默认析构函数，基类会自动处理资源清理。
      */
     ~Go1Leg(){}
 };
 
 #endif  // UNITREELEG_H