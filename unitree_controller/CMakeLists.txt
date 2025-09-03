#################################################################################
# 文件名: CMakeLists.txt
# 功能描述: unitree_controller包的CMake构建配置文件
# 
# 主要作用:
# 1. 定义包的构建规则和编译选项
# 2. 管理依赖库的查找和链接
# 3. 创建静态库和可执行文件目标
# 4. 配置安装规则和包导出信息
# 
# 在整个项目中的作用:
# - 🔴 核心构建文件 - 定义如何编译和链接unitree_controller包
# - 依赖管理 - 确保所有必需的库和头文件被正确找到
# - 目标定义 - 创建库文件和可执行程序
# 
# 构建目标:
# - unitree_controller: 核心控制库(静态库)
# - unitree_servo: 主伺服控制程序
# - unitree_external_force: 外力扰动测试工具  
# - unitree_move_kinetic: 位置控制器
# 
# 重要性:
# - 没有正确的CMakeLists.txt，包无法被编译
# - 链接错误会导致运行时找不到符号或库
# - 是catkin构建系统的核心输入文件
#################################################################################

# ==================== CMake基本配置 ====================

#
# CMake最低版本要求
# 版本 2.8.3 是ROS Kinetic及更高版本支持的稳定版本
# 提供了catkin构建系统所需的所有基本功能
#
cmake_minimum_required(VERSION 2.8.3)

#
# 项目名称定义
# 必须与package.xml中的包名一致
# 影响生成的库和可执行文件的默认命名
#
project(unitree_controller)

# ==================== ROS包依赖查找 ====================

#
# 查找所需的catkin包
# 这些包提供了编译和运行本包所需的头文件、库文件和CMake配置
#
find_package(catkin REQUIRED COMPONENTS
    controller_manager       # ROS控制器管理框架
    genmsg                  # 消息生成工具(虽然本包不生成消息，但用于兼容性)
    joint_state_controller  # 关节状态控制器
    robot_state_publisher   # 机器人状态发布器
    roscpp                  # ROS C++客户端库
    gazebo_ros             # Gazebo-ROS接口库
    std_msgs               # ROS标准消息类型
    tf                     # 坐标变换库
    geometry_msgs          # 几何消息类型(位置、姿态、速度等)
    unitree_legged_msgs    # 🔴 核心依赖: Unitree机器人专用消息
)

#
# 查找Gazebo仿真器
# REQUIRED: 如果找不到Gazebo，构建将失败
# Gazebo提供物理仿真环境和插件接口
#
find_package(gazebo REQUIRED)

# ==================== Catkin包配置 ====================

#
# 配置catkin包的导出信息
# 定义其他包依赖本包时需要的信息
#
catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs     # 声明对unitree_legged_msgs的依赖
    # 
    # 可以添加其他导出信息:
    # INCLUDE_DIRS include    # 导出头文件目录
    # LIBRARIES ${PROJECT_NAME}  # 导出库文件
    # CFG_EXTRAS ...         # 导出额外的CMake配置
)

# ==================== 头文件和库文件路径配置 ====================

#
# 配置头文件搜索路径
# 编译器将在这些目录中查找#include指定的头文件
#
include_directories(
    include                 # 本包的头文件目录
    ${Boost_INCLUDE_DIR}    # Boost库头文件(catkin依赖)
    ${catkin_INCLUDE_DIRS}  # 所有catkin依赖包的头文件
    ${GAZEBO_INCLUDE_DIRS}  # Gazebo头文件
)

#
# 配置库文件搜索路径
# 链接器将在这些目录中查找库文件
#
link_directories(${GAZEBO_LIBRARY_DIRS})

#
# 设置编译器标志
# 添加Gazebo所需的编译选项和宏定义
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# ==================== 库目标定义 ====================

#
# 创建核心控制库
# 库名: unitree_controller (与项目名相同)
# 类型: 静态库 (默认，也可以是共享库)
# 源文件: src/body.cpp
#
# 功能:
# - 包含机器人基础控制算法的实现
# - 提供PD控制器参数设置
# - 实现平滑运动控制和站立控制
# - 被其他可执行文件链接使用
#
add_library(${PROJECT_NAME}
    src/body.cpp 
)

#
# 添加消息依赖
# 确保在编译本库之前，unitree_legged_msgs的C++消息类型已生成
# 这是处理消息依赖的标准做法
#
add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

#
# 链接核心控制库
# 将必需的库链接到unitree_controller库
#
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}       # 所有catkin依赖包的库
  ${EXTRA_LIBS}            # 额外的库(如果有)
)

# ==================== Gazebo插件库定义 (已注释) ====================

#
# 足端接触检测插件 (当前已注释)
# 功能: 检测机器人足端与地面的接触状态和接触力
# 注释原因: 实际的Gazebo插件在unitree_gazebo包中实现
#
# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#
# 力可视化插件 (当前已注释)  
# 功能: 在Gazebo中可视化显示施加在机器人上的力
# 注释原因: 实际的Gazebo插件在unitree_gazebo包中实现
#
# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# ==================== 可执行文件目标定义 ====================

#
# 外力扰动测试工具
# 可执行文件名: unitree_external_force
# 源文件: src/external_force.cpp
# 
# 功能:
# - 🟡 测试工具 - 通过键盘控制施加外力扰动  
# - 支持脉冲模式和连续模式
# - 测试机器人抗扰动能力和平衡控制
#
# 使用方法: rosrun unitree_controller unitree_external_force
#
add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

#
# 主伺服控制程序
# 可执行文件名: unitree_servo
# 源文件: src/servo.cpp
# 链接库: unitree_controller库 + catkin库
#
# 功能:
# - 🔴 核心控制器 - 机器人主要控制程序
# - 实现多线程消息处理
# - 连接Gazebo仿真与ROS控制系统
# - 让机器人从平躺状态站立起来
#
# 使用方法: rosrun unitree_controller unitree_servo
# 重要性: 这是让机器人工作的主要程序
#
add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

#
# 位置控制器/运动学发布器
# 可执行文件名: unitree_move_kinetic  
# 源文件: src/move_publisher.cpp
#
# 功能:
# - 🟢 开发辅助工具 - 直接控制机器人位置和姿态
# - 支持世界坐标系和机器人坐标系两种模式
# - 提供圆周运动和直线运动演示
# - 适用于SLAM和视觉算法开发测试
#
# 使用方法: rosrun unitree_controller unitree_move_kinetic
# 应用场景: 绕过关节控制器，直接设置机器人在Gazebo中的位置
#
add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES})

# ==================== 构建目标总结 ====================

#
# 构建完成后将生成以下文件:
#
# 1. 库文件:
#    - libunitree_controller.a: 核心控制库(静态库)
#
# 2. 可执行文件:
#    - unitree_servo: 主伺服控制程序 
#    - unitree_external_force: 外力扰动测试工具
#    - unitree_move_kinetic: 位置控制器
#
# 3. 安装位置:
#    - 库文件: catkin_ws/devel/lib/
#    - 可执行文件: catkin_ws/devel/lib/unitree_controller/
#    - 头文件: catkin_ws/devel/include/unitree_controller/
#

# ==================== 可选的安装规则 (未定义) ====================

#
# 如果需要安装到系统目录，可以添加以下规则:
#
# install(TARGETS ${PROJECT_NAME} unitree_servo unitree_external_force unitree_move_kinetic
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )