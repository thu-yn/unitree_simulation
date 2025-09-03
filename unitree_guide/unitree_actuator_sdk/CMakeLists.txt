# Unitree电机SDK - CMake构建配置文件
# 功能: 配置跨平台编译环境，自动检测系统架构并链接对应的电机控制库
# 支持: Linux 32/64位 和 Windows 64位系统

# CMake最低版本要求 - 确保构建系统兼容性
cmake_minimum_required(VERSION 2.8.3)

# 项目名称定义 - 设置为motor_ctrl，用于电机控制功能
project(motor_ctrl)

# 头文件包含目录设置
# 将include目录添加到编译器搜索路径，使源代码能找到所需头文件
include_directories(include)

# 库文件链接目录设置  
# 将lib目录添加到链接器搜索路径，用于查找Unitree电机SDK库文件
link_directories(lib)

# 系统架构自动检测 - 检测32位或64位系统架构
# 默认设置为32位，后续根据实际情况调整
set(BITNESS 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # CMAKE_SIZEOF_VOID_P为8表示64位系统（指针大小为8字节）
    set(BITNESS 64)
endif()

# 跨平台库文件选择 - 根据操作系统和架构选择对应的SDK库
IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    # Linux系统: 根据架构选择32位或64位动态链接库
    IF(BITNESS EQUAL 64)
        set(EXTRA_LIBS libUnitree_motor_SDK_Linux64.so)
    ELSEIF(BITNESS EQUAL 32)
        set(EXTRA_LIBS libUnitree_motor_SDK_Linux32.so)
    ENDIF()
ELSEIF(CMAKE_SYSTEM_NAME MATCHES "Windows")
    # Windows系统: 使用64位动态链接库（DLL文件）
    set(EXTRA_LIBS libUnitree_motor_SDK_Win64.dll)
ENDIF()   

# C语言示例程序构建
# 生成check_c可执行文件，用于演示C语言电机控制接口
add_executable(check_c src/check.c)
target_link_libraries(check_c ${EXTRA_LIBS})

# 三电机控制示例程序（已注释）
# 可用于同时控制多个电机的高级示例
# add_executable(check3motor src/check3motor.c)
# target_link_libraries(check3motor ${EXTRA_LIBS})

# C++示例程序构建
# 生成check_cpp可执行文件，用于演示C++电机控制接口
add_executable(check_cpp src/check.cpp)
target_link_libraries(check_cpp ${EXTRA_LIBS})

# 可执行文件输出路径设置
# 将生成的可执行文件统一放置在项目根目录的bin文件夹中
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 构建说明:
# 1. 自动检测系统类型和架构，选择合适的SDK库文件
# 2. 支持C和C++两种编程语言的示例程序
# 3. 统一的输出目录管理，便于程序的查找和执行
# 4. 跨平台兼容设计，同一套代码可在Linux和Windows上编译