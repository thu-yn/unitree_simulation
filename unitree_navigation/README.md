# Unitree四足机器人导航系统使用指南

## 系统概述

Unitree四足机器人导航系统是一个基于ROS的完整解决方案，用于实现四足机器人的自主建图和导航功能。该系统集成了FAST-LIO激光雷达里程计和ROS导航栈，能够在未知环境中进行高精度建图和自主导航。

## 主要功能

1. **遥控建图模式**：使用遥控器控制机器人移动，同时系统自动构建环境地图
2. **自主导航模式**：在已知地图的情况下，机器人能够自主规划路径并导航到指定目标点
3. **自动检测模式**：系统自动检测是否存在地图，并选择相应的工作模式
4. **路径记录与回放**：支持记录机器人运动轨迹并回放

## 使用方法

### 1. 遥控建图模式

在这种模式下，您可以使用遥控器控制机器人移动，同时系统会自动构建地图：

```bash
roslaunch unitree_navigation mapping_only.launch
```

地图构建完成后，可以保存地图：

```bash
roslaunch unitree_navigation save_map.launch map_name:=my_map
```

### 2. 自主导航模式

在已知地图的情况下，启动导航系统：

```bash
roslaunch unitree_navigation navigation.launch use_existing_map:=true map_name:=my_map
```

### 3. 自动检测模式

系统会自动检测是否存在指定的地图，如果存在则启动导航模式，否则启动建图模式：

```bash
roslaunch unitree_navigation auto_navigation.launch map_name:=my_map
```

### 4. 路径记录与回放

记录机器人运动轨迹：

```bash
# 开始记录路径
rosservice call /start_recording

# 停止记录
rosservice call /stop_recording

# 保存路径
rosservice call /save_path

# 清除路径
rosservice call /clear_path
```

## 系统架构

系统主要由以下几个模块组成：

1. **感知模块**：处理激光雷达数据，生成点云和2D地图
2. **定位模块**：使用FAST-LIO进行激光雷达里程计
3. **规划模块**：使用ROS导航栈进行全局和局部路径规划
4. **控制模块**：将规划结果转换为机器人控制命令

## 启动文件说明

1. **navigation.launch**：主要的导航启动文件，根据参数决定是建图还是导航模式
2. **mapping_only.launch**：仅启动建图功能
3. **save_map.launch**：保存当前地图
4. **auto_navigation.launch**：自动检测地图并选择工作模式
5. **load_map.launch**：加载已有地图

## 参数配置

主要配置文件位于`config`目录下：

- `costmap_common_params.yaml`: 代价地图通用参数
- `global_costmap_params.yaml`: 全局代价地图参数
- `local_costmap_params.yaml`: 局部代价地图参数
- `base_local_planner_params.yaml`: 局部规划器参数

## 常见问题

1. **机器人无法连接**
   - 检查机器人IP地址是否正确
   - 确保网络连接正常

2. **导航精度不高**
   - 调整代价地图参数
   - 检查足迹计算是否准确

3. **建图质量不佳**
   - 减慢机器人移动速度
   - 调整FAST-LIO参数

## 服务和话题

### 主要服务

- `/robot_stand`: 让机器人站立
- `/robot_sit`: 让机器人坐下
- `/start_recording`: 开始记录路径
- `/stop_recording`: 停止记录路径
- `/save_path`: 保存记录的路径
- `/clear_path`: 清除记录的路径
- `/save_map`: 保存当前地图

### 主要话题

- `/cmd_vel`: 速度控制命令
- `/odom`: 里程计数据
- `/processed_cloud`: 处理后的点云数据
- `/map`: 地图数据
- `/move_base_simple/goal`: 导航目标点

## 系统要求

- ROS Noetic
- PCL库
- Unitree SDK
- FAST-LIO

## 系统依赖
- ROS Noetic/Melodic
- FastLIO (用于定位和点云处理)

## 功能说明
本项目提供两种建图模式：
1. 遥控建图：操作员通过遥控器控制机器人运动，同时进行环境建图
2. 自主导航建图：机器人自主探索环境并构建地图

## 注意事项
- 本项目使用FastLIO作为定位和点云数据源
- 确保FastLIO正确配置并运行
- 地图默认保存在~/maps目录下