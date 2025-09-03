# CMake最低版本要求
cmake_minimum_required(VERSION 2.8.3)

# 项目名称定义 - Unitree四足机器人腿部控制模块
project(unitree_legged_control)

# 查找ROS catkin构建系统所需的依赖包
find_package(catkin REQUIRED COMPONENTS
    controller_interface    # ROS控制器接口框架 - 提供标准控制器基类
    hardware_interface      # 硬件抽象接口 - 定义关节句柄和硬件接口
    pluginlib               # 插件系统支持 - 实现控制器的动态加载
    roscpp                  # ROS C++接口 - 基础ROS功能
    realtime_tools          # 实时工具库 - 提供实时安全的数据结构
    unitree_legged_msgs     # Unitree机器人消息定义 - 电机命令和状态消息
)

# catkin包配置 - 定义包的依赖关系和导出信息
catkin_package(
    CATKIN_DEPENDS              # 运行时依赖的catkin包
    unitree_legged_msgs         # Unitree消息包 - 核心通信协议
    controller_interface        # 控制器接口 - ROS Control框架标准
    hardware_interface          # 硬件接口 - 关节控制抽象层
    pluginlib                   # 插件库 - 支持控制器动态加载
    roscpp                      # ROS C++核心库
    INCLUDE_DIRS include        # 导出头文件目录
    LIBRARIES ${PROJECT_NAME}   # 导出库名称
)

# 包含头文件目录
include_directories(
    include                     # 项目本地头文件目录
    ${catkin_INCLUDE_DIRS}      # catkin依赖包的头文件目录
    ${Boost_INCLUDE_DIR}        # Boost库头文件目录
)

# 链接库目录设置
link_directories(
    ${catkin_LIB_DIRS}          # catkin依赖包的库目录
    lib                         # 项目本地库目录
)

# 构建unitree_legged_control库 - 核心关节控制器实现
add_library( unitree_legged_control 
    src/joint_controller.cpp    # 关节控制器实现文件 - 包含UnitreeJointController类
)

# 添加消息生成依赖 - 确保unitree_legged_msgs的C++消息已生成
add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

# 链接库配置
target_link_libraries(unitree_legged_control 
    ${catkin_LIBRARIES}              # catkin依赖的所有库
    unitree_joint_control_tool       # Unitree关节控制工具库 - 包含控制算法
)