# Unitree执行器SDK

## 简介

本库用于PC与电机控制板之间的通信，使用户能够通过PC控制电机。该库包含Linux和Windows版本，其中Linux版本还支持ROS。我们提供了C、C++和Python的使用示例。

## 主要特性

- 🖥️ **跨平台支持**: 支持Linux、Windows系统
- 🤖 **ROS集成**: Linux版本完整支持ROS
- 🔧 **多语言支持**: 提供C、C++、Python示例代码
- ⚡ **实时控制**: 支持电机位置、速度、力矩的实时控制
- 🔌 **串口通信**: 基于串口的可靠通信协议
- 🆔 **电机管理**: 内置电机ID修改工具

## 控制模式

- **模式0**: 空闲模式(free)
- **模式5**: 开环缓慢转动(Open loop slow turning)
- **模式10**: 闭环控制(Close loop control)

## 系统依赖

- [CMake](http://www.cmake.org) (版本 2.8.3 或更高)
- Linux系统需要sudo权限用于串口访问
- Windows系统建议使用MinGW编译环境

## 目录结构

```
unitree_actuator_sdk/
├── lib/                    # 库文件目录
│   ├── libUnitree_motor_SDK_Linux32.so    # Linux 32位库
│   ├── libUnitree_motor_SDK_Linux64.so    # Linux 64位库
│   └── libUnitree_motor_SDK_Win64.dll     # Windows 64位库
├── include/                # 头文件目录
│   ├── LSerial.h          # 串口操作函数声明
│   ├── motor_msg.h        # 电机通信命令结构
│   └── motor_ctrl.h       # 编码解码函数
├── ChangeID_Tool/         # 电机ID修改工具
│   ├── Linux/             # Linux版本工具
│   └── Windows/           # Windows版本工具
├── src/                   # C/C++示例源码
│   ├── check.c            # 完整功能的C示例(推荐)
│   └── check.cpp          # C++示例
├── script/                # Python示例源码
│   ├── typedef.py         # Python数据结构定义
│   └── check.py           # Python示例程序
├── unitree_motor_ctrl/    # ROS功能包
│   ├── src/               # ROS C++源码
│   ├── script/            # ROS Python脚本
│   ├── CMakeLists.txt     # ROS编译配置
│   └── package.xml        # ROS包描述文件
├── build/                 # 编译目录
└── bin/                   # 可执行文件目录
```

### 核心文件说明

#### /lib 目录
包含Linux和Windows的库文件。如果需要在ARM32/64平台使用SDK，请手动修改CMakeList以选择正确的`.so`文件。

#### /include 目录
- `LSerial.h`: 串口操作函数声明
- `motor_msg.h`: 电机通信命令结构体定义
- `motor_ctrl.h`: 数据编码和解码函数

#### /ChangeID_Tool 目录
包含Linux和Windows版本的电机ID修改工具。请按照可执行文件的指导使用。

#### /src 目录
包含C和C++示例源码。示例程序可以控制电机按指定命令运行指定时间，然后停止。

⚠️ **重要提示**: 只有`check.c`是包含所有功能的完整控制示例。`check.cpp`和Python示例不包含某些注释和重要教程。

#### /script 目录
包含Python示例源码。功能与C/C++示例相同。`typedef.py`声明了Python风格的所有库函数和结构体数据结构，使`check.py`能够正确调用库。

#### /unitree_motor_ctrl 目录
ROS功能包，库文件就是Linux的库文件。

## 使用方法

### Linux下的C/C++

#### 编译
```bash
mkdir build
cd build
cmake ..
make
```

#### 运行
```bash
cd ../bin
sudo ./check_c
```

或者
```bash
cd ../bin
sudo ./check_cpp
```

### Linux下的Python

```bash
cd script
sudo python3 check.py
```

### ROS环境下的C/C++

#### 编译
在catkin工作空间下运行：
```bash
catkin_make
```

#### 运行
由于无法使用`sudo rosrun`，请按以下方式运行：

首先，在一个终端运行：
```bash
roscore
```

在另一个终端运行：
```bash
sudo su
source devel/setup.bash
rosrun unitree_motor_ctrl unitree_motor_ctrl_node
```

### ROS环境下的Python

首先，在一个终端运行：
```bash
roscore
```

在另一个终端运行：
```bash
sudo su
source devel/setup.bash
rosrun unitree_motor_ctrl check.py
```

### Windows下的C/C++

#### 编译
以MinGW为例。首先，在CMake GUI中选择"MinGW Makefiles"并生成makefiles到`build`目录。然后打开cmd.exe，运行：
```bash
cd build
mingw32-make.exe
```

#### 运行
将生成的.exe文件(/bin)和.dll文件(/lib)放到同一目录，双击.exe文件运行。

### Windows下的Python

打开cmd.exe，然后：
```bash
cd script
check.py
```

## 电机控制参数说明

### 发送参数(MOTOR_send)
- `id`: 电机ID
- `mode`: 控制模式
- 0: 空闲模式
- 5: 开环缓慢转动
- 10: 闭环控制
- `T`: 期望电机输出力矩(Nm)
- `W`: 期望电机速度(rad/s)
- `Pos`: 期望电机位置(rad)
- `K_P`: 位置刚度系数
- `K_W`: 速度刚度系数

### 接收参数(MOTOR_recv)
- `motor_id`: 电机ID
- `mode`: 当前工作模式
- `Temp`: 电机温度
- `T`: 当前输出力矩
- `W`: 当前速度
- `Pos`: 当前位置
- `gyro`: 陀螺仪数据
- `acc`: 加速度计数据

## 力矩控制公式

实际传递给控制板的指令力矩为：
```
实际力矩 = K_P × 位置误差 + K_W × 速度误差 + T
```

⚠️ **注意**: 以上参数均为电机本身参数，与减速器无关。

## 故障排除

### 常见问题

1. **权限问题**: Linux下需要使用sudo运行程序以获取串口访问权限
2. **串口设备**: 确保串口设备路径正确(Linux: `/dev/ttyUSB0`, Windows: `COM4`)
3. **库文件路径**: 确保选择了正确的库文件(32位/64位)
4. **ARM平台**: 在ARM32/64平台使用时需要手动修改CMakeList

### 调试建议

- 首先使用完整功能的`check.c`示例进行测试
- 检查电机ID是否正确配置
- 使用ChangeID_Tool修改电机ID如果需要
- 确保电机电源和通信连接正常

## 许可证

请查看LICENSE文件了解详细的许可证信息。

## 支持与贡献

如有问题或建议，请通过以下方式联系：
- GitHub Issues
- Unitree官方技术支持

## 更新日志

- 支持Linux和Windows平台
- 集成ROS支持
- 提供多语言示例代码
- 优化通信协议稳定性