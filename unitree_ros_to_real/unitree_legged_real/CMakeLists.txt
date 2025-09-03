# 设置CMake最低版本要求
cmake_minimum_required(VERSION 2.8.3)

# 定义项目名称：unitree_legged_real
# 此项目负责连接ROS与Unitree真实机器人硬件
project(unitree_legged_real)

# 添加C++11编译选项，确保使用现代C++特性
add_compile_options(-std=c++11)

# 查找所需的ROS包依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp                 # ROS C++接口库
    geometry_msgs          # 几何消息类型（位置、速度等）
    unitree_legged_msgs    # Unitree机器人专用消息类型（核心依赖）
)

# 创建catkin包，定义包的依赖关系
catkin_package()

# 输出系统架构信息，用于调试
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

# 自动检测系统架构，选择对应的SDK库
# 支持x86_64（开发环境）和ARM64（机器人板载计算机）双架构
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)          # x86_64架构使用amd64库
else()
  set(ARCH arm64)          # 其他架构（主要是ARM64）使用arm64库
endif()

# 链接Unitree SDK库目录（用于编译期搜索头文件/库）
# 使用当前包目录作为基准，避免在Catkin下被解析为工作区根目录
set(UNITREE_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk)
set(UNITREE_SDK_LIB_DIR ${UNITREE_SDK_ROOT}/lib/cpp/${ARCH})
set(UNITREE_SDK_LIB ${UNITREE_SDK_LIB_DIR}/libunitree_legged_sdk.so)
link_directories(${UNITREE_SDK_LIB_DIR})

# 设置额外链接库
# -pthread: 支持多线程操作
# 直接使用SDK动态库的绝对路径，避免链接器找不到 -lunitree_legged_sdk
set(EXTRA_LIBS -pthread ${UNITREE_SDK_LIB})

# 设置编译优化选项
# -O3: 最高级别优化，提升运行性能
# -fPIC: 生成位置无关代码，支持动态链接
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# 设置头文件包含目录
include_directories(
    include                                   # 本项目头文件目录
    ${catkin_INCLUDE_DIRS}                   # catkin相关头文件
    ${UNITREE_SDK_ROOT}/include              # Unitree SDK头文件
)

# =============================================================================
# 编译可执行程序
# =============================================================================

# 编译步行控制示例程序
# 功能：演示高层控制模式的使用，包含各种步态和姿态控制
add_executable(example_walk src/exe/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 编译位置控制示例程序  
# 功能：演示底层控制模式的使用，直接控制关节位置
add_executable(example_position src/exe/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 编译ROS-UDP通信桥梁程序
# 功能：核心组件，负责ROS消息与Unitree SDK之间的双向转换和通信
add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})