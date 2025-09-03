# unitree_legged_control

<div align="center">

**Unitree四足机器人关节级控制器**

*专业的ROS Control框架插件，为Unitree机器人提供高精度实时关节控制*

[![ROS Version](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-MPL--2.0-green.svg)](LICENSE)
[![Gazebo](https://img.shields.io/badge/Gazebo-8%2B-orange.svg)](http://gazebosim.org/)

</div>

## 📖 模块概述

`unitree_legged_control` 是Unitree四足机器人项目的核心关节控制模块，提供符合ROS Control框架标准的专业关节控制器插件。该模块实现了高精度、实时的关节级控制，支持位置、速度、力矩的复合控制模式，是连接ROS控制命令与Gazebo仿真环境的关键桥梁。

### 🎯 核心特性

- **🔴 标准化控制器**: 完全符合ROS Control框架规范的关节控制器插件
- **⚡ 实时控制**: 1000Hz高频实时控制循环，保证控制精度和响应速度
- **🛡️ 多重安全**: 位置、速度、力矩三重安全限制机制
- **🤖 智能识别**: 自动识别关节类型（髋关节、大腿、小腿）并应用优化参数
- **🔧 配置驱动**: 通过YAML文件配置，支持运行时参数调整
- **📊 实时反馈**: 完整的状态监控和调试接口

## 🏗️ 项目结构

```
unitree_legged_control/
├── CMakeLists.txt                    # 构建配置文件
├── package.xml                       # ROS包依赖声明
├── unitree_controller_plugins.xml    # 插件注册配置
├── include/
│   ├── joint_controller.h            # 关节控制器头文件
│   └── unitree_joint_control_tool.h  # 控制工具函数库
└── src/
    └── joint_controller.cpp          # 关节控制器核心实现
```

## 🔧 依赖要求

### 系统环境
- **Ubuntu**: 18.04+ (推荐 20.04)
- **ROS**: Melodic / Noetic
- **Gazebo**: 8.0+

### ROS依赖包
```bash
# 核心依赖
controller_interface     # ROS控制器接口框架
hardware_interface      # 硬件抽象接口
pluginlib              # 插件系统支持
realtime_tools         # 实时工具库
unitree_legged_msgs    # Unitree机器人消息定义

# 基础依赖
roscpp                 # ROS C++接口
```

### 安装依赖
```bash
# ROS Melodic
sudo apt-get install ros-melodic-controller-interface \
                     ros-melodic-gazebo-ros-control \
                     ros-melodic-joint-state-controller \
                     ros-melodic-effort-controllers \
                     ros-melodic-realtime-tools

# ROS Noetic
sudo apt-get install ros-noetic-controller-interface \
                     ros-noetic-gazebo-ros-control \
                     ros-noetic-joint-state-controller \
                     ros-noetic-effort-controllers \
                     ros-noetic-realtime-tools
```

## 🚀 编译与安装

### 1. 克隆项目
```bash
cd ~/catkin_ws/src
git clone <repository_url>
```

### 2. 安装依赖
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 📋 使用方法

### 1. 基本配置

创建控制器配置文件 `robot_control.yaml`:

```yaml
# 机器人控制器配置示例
go2_gazebo:
  # 关节状态发布器
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 1000

  # 前左腿控制器配置
  FL_hip_controller:
    type: unitree_legged_control/UnitreeJointController
    joint: FL_hip_joint
    pid: {p: 100.0, i: 0.0, d: 5.0}    # 髋关节参数
    
  FL_thigh_controller:
    type: unitree_legged_control/UnitreeJointController
    joint: FL_thigh_joint
    pid: {p: 300.0, i: 0.0, d: 8.0}    # 大腿关节参数
    
  FL_calf_controller:
    type: unitree_legged_control/UnitreeJointController
    joint: FL_calf_joint
    pid: {p: 300.0, i: 0.0, d: 8.0}    # 小腿关节参数

  # 其他三条腿的配置...
  # FR (前右), RL (后左), RR (后右)
```

### 2. Launch文件集成

```xml
<!-- robot_control.launch -->
<launch>
  <!-- 加载控制器配置 -->
  <rosparam file="$(find your_package)/config/robot_control.yaml" command="load"/>
  
  <!-- 启动控制器管理器 -->
  <node pkg="controller_manager" type="spawner" name="controller_spawner"
        args="joint_state_controller
              FL_hip_controller FL_thigh_controller FL_calf_controller
              FR_hip_controller FR_thigh_controller FR_calf_controller
              RL_hip_controller RL_thigh_controller RL_calf_controller
              RR_hip_controller RR_thigh_controller RR_calf_controller"/>
</launch>
```

### 3. 启动控制器

```bash
# 启动Gazebo仿真环境
roslaunch unitree_gazebo normal.launch rname:=go2

# 启动控制器
roslaunch your_package robot_control.launch

# 监控控制器状态
rostopic echo /go2_gazebo/FL_hip_controller/state
```

## 🎮 控制接口

### 输入接口

**控制命令**: `unitree_legged_msgs/MotorCmd`
```bash
# 发送控制命令示例
rostopic pub /go2_gazebo/FL_hip_controller/command unitree_legged_msgs/MotorCmd \
"mode: 10
q: 0.0
Kp: 100.0
dq: 0.0
Kd: 5.0
tau: 0.0"
```

**力反馈**: `geometry_msgs/WrenchStamped` (可选)
```bash
# 力传感器反馈
rostopic pub /go2_gazebo/FL_hip_controller/joint_wrench geometry_msgs/WrenchStamped \
"wrench:
  torque:
    x: 0.0
    y: 0.0
    z: 0.0"
```

### 输出接口

**状态反馈**: `unitree_legged_msgs/MotorState`
```bash
# 监控关节状态
rostopic echo /go2_gazebo/FL_hip_controller/state

# 输出示例:
# q: 0.1234        # 当前位置 (弧度)
# dq: 0.0567       # 当前速度 (弧度/秒)
# tauEst: 2.345    # 估计力矩 (牛·米)
```

## ⚙️ 控制模式详解

### 1. PMSM模式 (正常控制)
```cpp
mode: 0x0A (10)  // 永磁同步电机模式
```

**特性**:
- 支持位置-速度-力矩复合控制
- 可动态调整控制参数
- 实时安全限制保护

**控制算法**:
```
τ = Kp × (q_des - q_cur) + Kd × (dq_des - dq_cur) + τ_ff
```

### 2. BRAKE模式 (制动模式)
```cpp
mode: 0x00 (0)   // 制动模式
```

**特性**:
- 关闭位置控制
- 高阻尼制动
- 紧急停止保护

### 3. 控制停止标志
```cpp
q: 2.146E+9      // 位置控制停止标志
dq: 16000.0      // 速度控制停止标志
```

## 🔧 参数调优指南

### PID参数推荐值

| 关节类型 | Kp值 | Ki值 | Kd值 | 应用场景 |
|---------|------|------|------|----------|
| **Hip (髋关节)** | 100.0 | 0.0 | 5.0 | 稳定性优先，中等增益 |
| **Thigh (大腿)** | 300.0 | 0.0 | 8.0 | 高增益，强力输出 |
| **Calf (小腿)** | 300.0 | 0.0 | 8.0 | 高精度，足端控制 |

### 参数调优策略

1. **保守参数** (稳定优先)
   ```yaml
   pid: {p: 50.0, i: 0.0, d: 2.0}
   ```

2. **标准参数** (平衡性能)
   ```yaml
   pid: {p: 100.0, i: 0.0, d: 5.0}
   ```

3. **激进参数** (响应优先)
   ```yaml
   pid: {p: 200.0, i: 0.0, d: 10.0}
   ```

### rqt图形化调参 (可选)

启用rqt调参功能:
```cpp
// 在 joint_controller.cpp 中取消注释
#define rqtTune
```

```bash
# 启动rqt参数调节界面
rosrun rqt_reconfigure rqt_reconfigure
```

## 🐛 调试与故障排除

### 常见问题

**1. 控制器加载失败**
```bash
# 检查插件注册
rospack plugins --attrib=plugin controller_interface

# 检查YAML配置语法
rosparam load robot_control.yaml
```

**2. 关节震荡**
- 降低PID增益: `Kp↓`, `Kd↓`
- 检查机械结构阻尼
- 验证URDF物理参数

**3. 响应迟缓**
- 增加比例增益: `Kp↑`
- 检查实时性能
- 验证控制频率设置

**4. 安全限制触发**
```bash
# 检查URDF关节限制
rostopic echo /robot_description | grep -A5 -B5 "limit"
```

### 调试工具

**控制器状态监控**:
```bash
# 实时监控所有关节状态
rosrun rqt_plot rqt_plot /*/state/q /*/state/dq /*/state/tauEst

# 监控控制器管理器
rosservice call /controller_manager/list_controllers
```

**性能分析**:
```bash
# 检查控制频率
rostopic hz /go2_gazebo/FL_hip_controller/state

# 检查延迟
rostopic echo -p /go2_gazebo/FL_hip_controller/state | head -20
```

## 🏗️ 架构设计

### 继承关系
```cpp
UnitreeJointController : public controller_interface::Controller<EffortJointInterface>
```

### 核心组件
- **硬件接口层**: `hardware_interface::JointHandle`
- **实时通信层**: `realtime_tools::RealtimeBuffer`
- **控制算法层**: `ServoCmd` + `computeTorque()`
- **安全保护层**: 多重限制函数

### 数据流
```
ROS Command → RealtimeBuffer → Control Algorithm → Safety Limits → Hardware Interface
     ↑                                                                        ↓
State Publisher ← RealtimePublisher ← State Feedback ← Sensor Reading ← Joint State
```

## 🤝 在项目中的作用

### 模块层级定位
```
Layer 5: unitree_navigation    (应用集成层)
Layer 4: unitree_move_base     (导航规划层)  
Layer 3: unitree_controller    (中层控制层)
Layer 2: unitree_legged_control (底层控制层) ← 本模块
Layer 1: unitree_legged_real   (硬件抽象层)
Layer 0: unitree_legged_msgs   (消息基础层)
```

### 集成关系
- **上游模块**: `unitree_controller`, `unitree_gazebo`
- **下游依赖**: `unitree_legged_msgs`, ROS Control框架
- **协作模块**: `joint_state_controller`, `robot_state_publisher`

## 📚 相关文档

- [ROS Control官方文档](http://wiki.ros.org/ros_control)
- [Gazebo ROS Control插件](http://gazebosim.org/tutorials?tut=ros_control)
- [Unitree机器人开发文档](https://www.unitree.com/)
- [realtime_tools使用指南](http://wiki.ros.org/realtime_tools)

## 📄 许可证

本项目使用 [MPL-2.0](LICENSE) 许可证。

## 🙋‍♂️ 支持与贡献

如有问题或建议，请提交 [Issue](issues) 或 [Pull Request](pulls)。

---

<div align="center">

**🤖 让四足机器人动起来！**

*Made with ❤️ by Unitree Robotics*

</div>