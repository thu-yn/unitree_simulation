# unitree_legged_real

## 📖 简介

`unitree_legged_real` 是Unitree四足机器人与ROS之间的核心通信桥梁，实现了从仿真环境到真实机器人的无缝部署。该包为Unitree B1机器人提供了完整的ROS接口，支持高层运动控制和底层关节控制两种模式。

**版本信息**: SDK v3.8.3 (支持B1机器人)

## 🎯 主要功能

- **🔄 双向通信**: ROS消息 ↔ Unitree UDP协议的实时转换
- **🎮 双控制模式**: 支持高层运动控制和底层关节控制
- **🏗️ 多架构支持**: 自动适配x86_64和ARM64架构
- **⚡ 实时性能**: 500Hz高频控制循环，确保实时响应
- **🛡️ 安全机制**: 完善的控制模式切换和安全保护

## 📁 项目结构

```
unitree_legged_real/
├── CMakeLists.txt              # 构建配置文件
├── package.xml                 # ROS包依赖声明
├── launch/
│   └── real.launch            # 启动文件
├── include/
│   └── convert.h              # ROS消息与SDK数据转换函数
├── src/exe/                   # 可执行程序源码
│   ├── ros_udp.cpp           # 🔑 核心通信桥梁程序
│   ├── example_walk.cpp      # 高层控制示例
│   └── example_position.cpp  # 底层控制示例
└── unitree_legged_sdk/        # Unitree官方SDK
    ├── lib/cpp/              # SDK库文件
    │   ├── amd64/           # x86_64架构库
    │   └── arm64/           # ARM64架构库
    └── include/             # SDK头文件
```

## 🚀 快速开始

### 环境要求

- **操作系统**: Ubuntu 20.04
- **ROS版本**: ROS Noetic
- **支持架构**: x86_64 / ARM64
- **网络**: 与机器人在同一局域网

### 安装步骤

1. **克隆项目到工作空间**
```bash
cd ~/catkin_ws/src
git clone <project_url>
```

2. **编译项目**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 基本使用

#### 1. 启动通信桥梁

**高层控制模式** (推荐新手使用)
```bash
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

**底层控制模式** (需要专业操作)
```bash
roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
```

#### 2. 运行示例程序

**高层控制示例** - 机器人运动演示
```bash
# 确保机器人站立在地面上
rosrun unitree_legged_real example_walk
```

**底层控制示例** - 关节位置控制
```bash
# ⚠️ 警告：执行前必须完成以下安全操作
# 1. 按 L2+A 让机器人坐下
# 2. 按 L1+L2+start 进入底层控制模式  
# 3. 确保机器人悬挂固定
rosrun unitree_legged_real example_position
```

## 🎮 控制模式详解

### 高层控制模式 (High Level)

**特点**: 用户友好，安全可靠
- **适用场景**: 日常导航、基础步态控制
- **控制对象**: 整体运动(速度、方向、步态)
- **安全要求**: 机器人站立在地面上
- **网络配置**: 机器人IP `192.168.123.220:8082`

**示例运动序列**:
```cpp
// 起立准备
cmd.mode = 6;

// 前进行走
cmd.mode = 2;
cmd.velocity[0] = 0.3;  // 前进速度
cmd.yawSpeed = 0.2;     // 转向速度

// 姿态调整
cmd.mode = 1;
cmd.euler[0] = 0.3;     // Roll倾斜
```

### 底层控制模式 (Low Level)

**特点**: 精确控制，专业应用
- **适用场景**: 算法研究、自定义步态开发
- **控制对象**: 20个关节的精确位置/速度/力矩
- **安全要求**: 机器人必须悬挂固定
- **网络配置**: 机器人IP `192.168.123.10:8007`

**安全操作流程**:
```bash
1. L2 + A          # 让机器人坐下
2. L1 + L2 + start # 进入底层控制模式
3. 物理悬挂        # 确保机器人安全固定
4. 运行控制程序    # 执行底层控制代码
```

**关节控制示例**:
```cpp
// 设置关节控制参数
low_cmd.motorCmd[FR_0].mode = 0x0A;    // 位置控制模式
low_cmd.motorCmd[FR_0].q = target_pos; // 目标位置
low_cmd.motorCmd[FR_0].Kp = 20.0;      // 位置增益
low_cmd.motorCmd[FR_0].Kd = 2.0;       // 速度增益
```

## 🔧 核心组件说明

### 1. ros_udp.cpp - 通信桥梁核心

**功能**: ROS消息与UDP协议的双向转换器

**主要特性**:
- 🔄 实时双向数据转换
- 🧵 多线程并发通信(发送/接收)
- ⚡ 500Hz高频控制循环
- 🛡️ 通信协议安全校验

**数据流向**:
```
ROS应用 → ROS话题 → ros_udp → UDP → 机器人硬件
机器人硬件 → UDP → ros_udp → ROS话题 → ROS应用
```

### 2. convert.h - 数据转换库

**功能**: ROS消息格式与Unitree SDK数据结构的转换

**转换函数**:
```cpp
// ROS消息 → SDK命令
UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_legged_msgs::HighCmd &msg);
UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd &msg);

// SDK状态 → ROS消息  
unitree_legged_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state);
unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state);
```

### 3. 示例程序

#### example_walk.cpp - 高层控制演示
**运动序列**:
1. **起立准备** (0-4s): 机器人自动调整到站立状态
2. **姿态测试** (4-28s): Roll/Pitch/Yaw三轴倾斜演示
3. **运动控制** (28-42s): 前进、转向、侧移组合运动
4. **步态展示** (42-50s): 不同步态类型演示

#### example_position.cpp - 底层控制演示
**控制阶段**:
1. **初始化** (0-20ms): 记录当前关节位置
2. **平滑过渡** (20ms-5s): 从当前位置过渡到目标位置
3. **正弦运动** (5s+): 基于正弦函数的周期性运动

## 📡 ROS接口

### 话题 (Topics)

#### 高层控制
```bash
# 订阅话题 (接收控制命令)
/high_cmd  [unitree_legged_msgs/HighCmd]

# 发布话题 (发送状态反馈)
/high_state [unitree_legged_msgs/HighState]
```

#### 底层控制  
```bash
# 订阅话题 (接收控制命令)
/low_cmd   [unitree_legged_msgs/LowCmd]

# 发布话题 (发送状态反馈)
/low_state [unitree_legged_msgs/LowState]
```

### 消息类型

#### HighCmd - 高层控制命令
```cpp
uint8 mode                    # 运动模式 (0:空闲 1:站立 2:行走)
uint8 gaitType               # 步态类型
float32[2] velocity          # 线性速度 [vx, vy]
float32 yawSpeed             # 角速度
float32[3] euler             # 目标姿态角 [roll, pitch, yaw]
float32 bodyHeight           # 身体高度
float32 footRaiseHeight      # 足部抬起高度
```

#### LowCmd - 底层控制命令
```cpp
MotorCmd[20] motorCmd        # 20个电机控制命令
uint8 levelFlag              # 控制级别标志
```

#### MotorCmd - 电机控制命令
```cpp
uint8 mode                   # 控制模式 (0x0A:位置控制)
float32 q                    # 目标位置 (rad)
float32 dq                   # 目标速度 (rad/s)  
float32 tau                  # 前馈力矩 (N·m)
float32 Kp                   # 位置增益
float32 Kd                   # 速度增益
```

## 🌐 网络配置

### 默认网络设置
```bash
# 高层控制
机器人IP: 192.168.123.220
机器人端口: 8082
本地端口: 8090

# 底层控制  
机器人IP: 192.168.123.10
机器人端口: 8007
本地端口: 8091
```

### 网络连接检查
```bash
# 检查与机器人的连接
ping 192.168.123.220  # 高层控制
ping 192.168.123.10   # 底层控制

# 检查端口占用
netstat -an | grep 8090
netstat -an | grep 8091
```

## ⚠️ 安全注意事项

### 高层控制安全要求
1. ✅ **地面站立**: 确保机器人站立在平坦地面上
2. ✅ **周围空间**: 保证机器人周围有足够的活动空间
3. ✅ **遥控器准备**: 随时准备使用遥控器进行紧急停止

### 底层控制安全要求
1. 🔴 **物理固定**: 机器人必须悬挂或牢固固定
2. 🔴 **操作顺序**: 严格按照 L2+A → L1+L2+start 的顺序操作
3. 🔴 **参数限制**: 谨慎设置关节位置和PID参数
4. 🔴 **监控状态**: 持续监控关节状态，防止异常运动

### 紧急停止方法
```bash
# 方法1: 遥控器紧急停止
按住遥控器上的紧急停止按钮

# 方法2: 程序停止
Ctrl + C 终止ROS程序

# 方法3: 断电保护
紧急情况下直接断开机器人电源
```

## 🛠️ 故障排除

### 常见问题

#### 1. 连接失败
**症状**: 无法与机器人建立通信
**解决方案**:
```bash
# 检查网络连接
ping 192.168.123.220

# 检查IP配置
ifconfig

# 重启网络服务
sudo systemctl restart NetworkManager
```

#### 2. 编译错误
**症状**: catkin_make编译失败
**解决方案**:
```bash
# 检查架构匹配
echo $CMAKE_SYSTEM_PROCESSOR

# 清理编译缓存
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make clean

# 重新编译
catkin_make
```

#### 3. 权限错误
**症状**: UDP套接字权限不足
**解决方案**:
```bash
# 使用sudo权限运行
sudo rosrun unitree_legged_real ros_udp HIGHLEVEL

# 或者调整用户权限
sudo usermod -a -G dialout $USER
```

#### 4. 控制无响应
**症状**: 发送命令后机器人无反应
**解决方案**:
```bash
# 检查机器人状态
rostopic echo /high_state

# 检查命令发送
rostopic pub /high_cmd unitree_legged_msgs/HighCmd [...]

# 重启通信桥梁
rosnode kill /ros_udp
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

### 调试工具

#### 1. 话题监控
```bash
# 监控状态话题
rostopic echo /high_state
rostopic echo /low_state

# 监控命令话题
rostopic echo /high_cmd  
rostopic echo /low_cmd

# 检查话题频率
rostopic hz /high_state
```

#### 2. 网络调试
```bash
# 监控UDP数据包
sudo tcpdump -i eth0 port 8082

# 检查网络统计
netstat -su
```

#### 3. 系统监控
```bash
# 检查CPU使用率
top

# 检查ROS节点状态
rosnode list
rosnode info /ros_udp
```

## 🤝 与其他模块的集成

### 与仿真环境的兼容性
```bash
# 仿真模式 (使用unitree_gazebo)
roslaunch unitree_gazebo normal.launch

# 真实机器人模式 (使用unitree_legged_real)  
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

### 与导航系统的集成
```bash
# 启动导航系统
roslaunch unitree_navigation unitree_navigation.launch

# 启动真实机器人接口
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

### 自定义控制器开发
```cpp
// 示例：自定义速度控制器
class CustomController {
    ros::Publisher cmd_pub;
    ros::Subscriber state_sub;
    
    void stateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
        // 处理状态反馈
        unitree_legged_msgs::HighCmd cmd;
        cmd.mode = 2;
        cmd.velocity[0] = 0.5;  // 前进
        cmd_pub.publish(cmd);
    }
};
```

## 📚 参考资料

- [Unitree Robotics 官方文档](https://www.unitree.com/)
- [ROS Noetic 官方教程](http://wiki.ros.org/noetic)
- [Unitree SDK GitHub](https://github.com/unitreerobotics/unitree_legged_sdk)

## 📄 许可证

本项目遵循 MPL-2.0 许可证。详细信息请参阅 LICENSE 文件。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request 来改进本项目！

---

**⚡ 快速提示**: 初次使用建议先在仿真环境中熟悉控制逻辑，然后再部署到真实机器人上。安全第一！