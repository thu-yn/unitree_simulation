/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**
 * @file PyPlot.h
 * @brief C++封装的Python matplotlib绘图接口库
 * 
 * 【文件作用总览】
 * 本文件实现了一个C++封装的Python matplotlib绘图接口，为机器人控制系统提供强大的实时数据可视化功能。
 * 主要功能包括：
 * 
 * 1. **实时数据可视化**：支持多图表、多曲线的实时数据绘制
 * 2. **多数据源支持**：兼容数组、Eigen矩阵、std::vector等多种数据格式
 * 3. **自动时间管理**：提供自动时间戳功能，便于时序数据分析
 * 4. **灵活的图表管理**：支持图表的动态创建、命名和显示
 * 5. **调试辅助功能**：提供数据打印和查询接口
 * 
 * 【应用场景】
 * - 控制算法调试：实时观察控制量、状态量的变化趋势
 * - 系统性能分析：监控关节角度、速度、扭矩等运行参数
 * - 传感器数据监控：可视化IMU、力传感器、编码器等传感器数据
 * - 步态分析：展示机器人行走过程中的运动学和动力学特征
 * - 实验数据记录：保存和展示实验过程中的关键数据
 * 
 * 【技术特点】
 * - 基于成熟的matplotlib库，绘图功能丰富
 * - C++模板设计，支持多种数值类型
 * - 内存管理安全，自动处理资源释放
 * - 接口设计直观，易于集成到现有代码中
 * - 支持批量显示，便于数据对比分析
 */

#ifndef PYPLOT_H
#define PYPLOT_H

#include <map>
#include <thread>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "thirdParty/matplotlibcpp.h"  // 第三方matplotlib C++接口
#include "common/timeMarker.h"          // 时间标记工具

namespace plt = matplotlibcpp;  // 为方便使用，创建matplotlib命名空间别名

/**
 * @brief 曲线数据结构
 * 
 * 用于存储单条曲线的x-y坐标数据，是绘图系统中的基本数据单元。
 * 每个Curve对象代表图表中的一条曲线，包含该曲线的所有数据点。
 */
struct Curve{
    std::vector<double> x;  // X轴数据点序列（通常表示时间或索引）
    std::vector<double> y;  // Y轴数据点序列（通常表示测量值或计算结果）

    /**
     * @brief 打印指定X值附近的数据点
     * 
     * @param xRough 目标X值，用于查找最接近的数据点
     * @param pointNum 要打印的连续数据点数量
     * 
     * 功能说明：
     * - 在x数组中查找第一个大于xRough的位置
     * - 从该位置开始打印pointNum个连续的(x,y)数据对
     * 
     * 使用场景：
     * - 调试时查看特定时间点附近的数据
     * - 分析数据异常或突变点
     * - 验证数据记录的准确性
     */
    void printXY(double xRough, int pointNum){
        for(int i(0); i<x.size(); ++i){
            if(xRough < x[i]){
                for(int j(0); j < pointNum; ++j){
                    std::cout << "  X: " << x[i+j] << ", Y: " << y[i+j] << std::endl;
                }
                break;
            }
        }
    }
};

/**
 * @brief 绘图对象结构
 * 
 * 代表一个完整的图表，可以包含多条曲线。每个Plot对象管理：
 * - 多条曲线数据
 * - 曲线标签和命名
 * - 图表标题
 * - 曲线名称到ID的映射关系
 */
struct Plot{
    std::vector<Curve*> curves;          // 曲线指针数组，存储该图表的所有曲线
    std::vector<std::string> labels;     // 曲线标签数组，与curves一一对应
    std::string plotName;                // 图表名称/标题
    int curveCount;                      // 曲线数量

    std::map<std::string, int> curveName2ID;  // 曲线名称到索引的映射表

    /**
     * @brief Plot构造函数
     * 
     * @param name 图表名称
     * @param count 曲线数量
     * @param labelVec 曲线标签向量
     * 
     * 构造过程：
     * 1. 初始化图表基本信息
     * 2. 为每条曲线创建Curve对象
     * 3. 建立曲线名称到ID的映射关系
     */
    Plot(std::string name, int count, std::vector<std::string> labelVec)
        :plotName(name), curveCount(count), labels(labelVec){
        for(int i(0); i < count; ++i){
            curveName2ID.insert(std::pair<std::string, int>(labels[i], i));
            curves.push_back(new Curve());
        }
    }

    /**
     * @brief Plot析构函数
     * 
     * 负责释放所有动态分配的Curve对象，防止内存泄漏
     */
    ~Plot(){
        for(int i(0); i < curveCount; ++i){
            delete curves[i];
        }
    }

    /**
     * @brief 获取相对于开始时间的当前时间戳
     * 
     * @param startT 开始时间戳（微秒）
     * @return double 相对时间（秒）
     * 
     * 功能：
     * - 计算当前系统时间与起始时间的差值
     * - 将微秒转换为秒作为X轴坐标
     * - 为实时绘图提供时间基准
     */
    double getX(long long startT){
        return (double)(getSystemTime() - startT) * 1e-6;
    }

    /**
     * @brief 打印指定曲线在特定X值附近的数据点
     * 
     * @param curveName 曲线名称
     * @param xRough 目标X值
     * @param pointNum 打印的数据点数量
     * 
     * 提供调试接口，可以按曲线名称查看特定区域的数据
     */
    void printXY(std::string curveName, double xRough, int pointNum){
        std::cout << "[DEBUG] Plot: " << plotName << ", Curve: " << curveName << std::endl;
        curves[curveName2ID[curveName]]->printXY(xRough, pointNum);
    }
};

/**
 * @brief Python绘图接口类
 * 
 * 这是一个C++封装的Python matplotlib绘图接口，主要用于：
 * - 实时数据可视化
 * - 机器人运行状态监控
 * - 控制算法调试和分析
 * - 实验数据记录和展示
 * 
 * 设计特点：
 * - 支持多图表管理
 * - 支持多曲线绘制
 * - 提供模板函数支持多种数据类型
 * - 集成时间管理功能
 * - 提供丰富的数据添加接口
 */
class PyPlot{
public:
    /**
     * @brief PyPlot构造函数
     * 
     * 初始化绘图系统，设置初始状态为未启动
     */
    PyPlot();
    
    /**
     * @brief PyPlot析构函数
     * 
     * 清理所有Plot对象，释放动态分配的内存
     */
    ~PyPlot();
    
    /**
     * @brief 添加新的绘图对象（带自定义标签）
     * 
     * @param plotName 图表名称，作为图表的唯一标识
     * @param curveCount 曲线数量
     * @param labelVec 曲线标签向量，长度应等于curveCount
     * 
     * 功能：
     * - 创建新的Plot对象
     * - 建立图表名称到ID的映射
     * - 为每条曲线分配存储空间
     * 
     * 注意：如果plotName已存在，程序会报错并退出
     */
    void addPlot(std::string plotName, int curveCount, std::vector<std::string> labelVec);
    
    /**
     * @brief 添加新的绘图对象（自动生成标签）
     * 
     * @param plotName 图表名称
     * @param curveCount 曲线数量
     * 
     * 自动为每条曲线生成数字标签："1", "2", "3", ...
     * 适用于不需要特定标签名称的场景
     */
    void addPlot(std::string plotName, int curveCount);
    
    /**
     * @brief 显示单个图表
     * 
     * @param plotName 要显示的图表名称
     * 
     * 功能：
     * - 创建新的matplotlib图形窗口
     * - 设置图表标题
     * - 绘制所有曲线
     * - 添加图例
     * - 显示图表
     */
    void showPlot(std::string plotName);
    
    /**
     * @brief 显示多个指定的图表
     * 
     * @param plotNameVec 要显示的图表名称向量
     * 
     * 为每个图表创建独立的窗口进行显示
     */
    void showPlot(std::vector<std::string> plotNameVec);
    
    /**
     * @brief 显示所有图表并退出程序
     * 
     * 显示当前系统中所有的图表，每个图表占用一个独立窗口。
     * 注意：该函数调用后程序会退出（exit(0)）
     * 
     * 使用场景：
     * - 程序结束时的数据展示
     * - 批量查看所有收集的数据
     */
    void showPlotAll();

    /**
     * @brief 调试用数据打印接口
     * 
     * @param plotName 图表名称
     * @param curveName 曲线名称
     * @param xRough 目标X坐标
     * @param pointNum 打印的数据点数量，默认为1
     */
    void printXY(std::string plotName, std::string curveName, double xRough, int pointNum = 1);

    // ============== 数据添加接口系列 ==============
    // 提供多种重载函数，支持不同的数据源和时间戳方式

    /**
     * @brief 添加单个数值（自动时间戳）
     * 
     * @param plotName 图表名称
     * @param value 数值
     * 
     * 使用当前系统时间作为X坐标，适用于单曲线实时数据记录
     */
    void addFrame(std::string plotName, double value);
    
    /**
     * @brief 添加单个数值（指定时间戳）
     * 
     * @param plotName 图表名称
     * @param x X坐标值（通常为时间）
     * @param value Y坐标值
     */
    void addFrame(std::string plotName, double x, double value);

    /**
     * @brief 添加数组数据（自动时间戳）
     * 
     * @param plotName 图表名称
     * @param valueArray 数值数组指针
     * 
     * 模板函数，支持任意数值类型的数组
     * 数组长度应等于图表的曲线数量
     */
    template <typename T>
    void addFrame(std::string plotName, T* valueArray);
    
    /**
     * @brief 添加数组数据（指定时间戳）
     * 
     * @param plotName 图表名称
     * @param x X坐标值
     * @param valueArray 数值数组指针
     */
    template <typename T>
    void addFrame(std::string plotName, double x, T* valueArray);

    /**
     * @brief 添加Eigen矩阵数据（自动时间戳）
     * 
     * @param plotName 图表名称
     * @param vec Eigen矩阵或向量引用
     * 
     * 支持Eigen库的各种矩阵和向量类型
     * 在机器人学中广泛用于位置、速度、力等向量数据的可视化
     */
    template <typename T>
    void addFrame(std::string plotName, const Eigen::MatrixBase<T> &vec);
    
    /**
     * @brief 添加Eigen矩阵数据（指定时间戳）
     * 
     * @param plotName 图表名称
     * @param x X坐标值
     * @param vec Eigen矩阵或向量引用
     */
    template <typename T>
    void addFrame(std::string plotName, double x, const Eigen::MatrixBase<T> &vec);

    /**
     * @brief 添加std::vector数据（自动时间戳）
     * 
     * @param plotName 图表名称
     * @param vec std::vector引用
     */
    template <typename T>
    void addFrame(std::string plotName, const std::vector<T> &vec);
    
    /**
     * @brief 添加std::vector数据（指定时间戳）
     * 
     * @param plotName 图表名称
     * @param x X坐标值
     * @param vec std::vector引用
     */
    template <typename T>
    void addFrame(std::string plotName, double x, const std::vector<T> &vec);

private:
    /**
     * @brief 检查并初始化计时系统
     * 
     * 如果是第一次调用，记录起始时间戳
     * 后续的自动时间戳都基于这个起始时间计算
     */
    void _checkStart();
    
    int _plotCount = 0;                                    // 当前图表数量
    std::map<std::string, int> _plotName2ID;              // 图表名称到ID的映射
    std::vector< Plot* > _plots;                          // 图表指针数组
    long long _pointNum;                                   // 数据点计数（似乎未使用）
    
    /**
     * @brief 根据图表名称获取Plot对象指针
     * 
     * @param plotName 图表名称
     * @return Plot* 对应的Plot对象指针
     * 
     * 如果图表不存在，程序会报错并退出
     */
    Plot* _getPlotPtr(std::string plotName);
    
    bool start;        // 计时系统启动标志
    long long startT;  // 起始时间戳（微秒）
};

// ============== 内联函数实现 ==============

inline PyPlot::PyPlot(){
    start = false;  // 初始化时计时系统未启动
}

inline PyPlot::~PyPlot(){
    // 释放所有动态分配的Plot对象
    for(int i(0); i < _plotCount; ++i){
        delete _plots[i];
    }
}

inline void PyPlot::_checkStart(){
    if(!start){
        start = true;
        startT = getSystemTime();  // 记录系统启动时的时间戳
    }
}

inline void PyPlot::printXY(std::string plotName, std::string curveName, double xRough, int pointNum){
    _plots[_plotName2ID[plotName]]->printXY(curveName, xRough, pointNum);
}

inline void PyPlot::addPlot(std::string plotName, int curveCount, std::vector<std::string> labelVec){
    if(_plotName2ID.count(plotName) == 0){
        // 图表名称不存在，创建新图表
        _plotName2ID.insert(std::pair<std::string, int>(plotName, _plotCount));
        ++_plotCount;
        _plots.push_back( new Plot(plotName, curveCount, labelVec) );
    }else{
        // 图表名称已存在，报错退出
        std::cout << "[ERROR] Already has same Plot: " << plotName << std::endl;
        exit(-1);
    }
}

inline void PyPlot::addPlot(std::string plotName, int curveCount){
    // 自动生成数字标签
    std::vector<std::string> label;
    for(int i(0); i < curveCount; ++i){
        label.push_back(std::to_string(i+1));
    }
    addPlot(plotName, curveCount, label);
}

inline void PyPlot::showPlot(std::string plotName){
    Plot* plot = _getPlotPtr(plotName);
    plt::figure();                    // 创建新图形窗口
    plt::title(plot->plotName);       // 设置标题
    
    // 绘制所有曲线
    for(int i(0); i < plot->curveCount; ++i){
        plt::named_plot(plot->labels[i], plot->curves[i]->x, plot->curves[i]->y);
    }
    plt::legend();  // 添加图例
    plt::show();    // 显示图形
}

inline void PyPlot::showPlot(std::vector<std::string> plotNameVec){
    // 遍历所有指定的图表名称
    for(std::vector<std::string>::iterator itName = plotNameVec.begin(); itName != plotNameVec.end(); ++itName){
        plt::figure();
        Plot* plot = _plots[_plotName2ID[*itName]];
        plt::title(plot->plotName);
        
        for(int i(0); i < plot->curveCount; ++i){
            plt::named_plot(plot->labels[i], plot->curves[i]->x, plot->curves[i]->y);
        }
        plt::legend();
    }
    plt::show();
}

inline void PyPlot::showPlotAll(){
    // 显示所有图表
    for(int i(0); i < _plotCount; ++i){
        plt::figure();
        Plot* plot = _plots[i];
        plt::title(plot->plotName);
        
        for(int j(0); j < plot->curveCount; ++j){
            plt::named_plot(plot->labels[j], plot->curves[j]->x, plot->curves[j]->y);
        }
        plt::legend();
    }
    plt::show();
    exit(0);  // 显示完所有图表后退出程序
}

inline Plot* PyPlot::_getPlotPtr(std::string plotName){
    if(_plotName2ID.count(plotName) == 0){
        // 图表不存在，报错退出
        std::cout << "[ERROR] Plot " << plotName << " does not exist" << std::endl;
        exit(-1);
    }else{
        return _plots[_plotName2ID[plotName]];
    }
}

inline void PyPlot::addFrame(std::string plotName, double value){
    _checkStart();  // 确保计时系统已启动
    Plot* plot = _getPlotPtr(plotName);
    addFrame(plotName, plot->getX(startT), value);  // 使用自动时间戳
}

inline void PyPlot::addFrame(std::string plotName, double x, double value){
    Plot* plot = _getPlotPtr(plotName);
    
    // 向第一条曲线添加数据点
    plot->curves[0]->x.push_back(x);
    plot->curves[0]->y.push_back(value);
}

// ============== 模板函数实现 ==============

template <typename T>
inline void PyPlot::addFrame(std::string plotName, T* valueArray){
    _checkStart();
    Plot* plot = _getPlotPtr(plotName);
    addFrame(plotName, plot->getX(startT), valueArray);
}

template <typename T>
inline void PyPlot::addFrame(std::string plotName, double x, T* valueArray){
    Plot* plot = _getPlotPtr(plotName);

    // 向所有曲线添加数据点
    for(int i(0); i < plot->curveCount; ++i){
        plot->curves[i]->x.push_back(x);
        plot->curves[i]->y.push_back(valueArray[i]);
    }
}

template <typename T>
inline void PyPlot::addFrame(std::string plotName, const Eigen::MatrixBase<T> &vec){
    _checkStart();
    Plot* plot = _getPlotPtr(plotName);
    addFrame(plotName, plot->getX(startT), vec);
}

template <typename T>
inline void PyPlot::addFrame(std::string plotName, double x, const Eigen::MatrixBase<T> &vec){
    Plot* plot = _getPlotPtr(plotName);

    // 从Eigen向量中提取数据添加到曲线
    for(int i(0); i < plot->curveCount; ++i){
        plot->curves[i]->x.push_back(x);
        plot->curves[i]->y.push_back(vec(i));
    }
}

template <typename T>
inline void PyPlot::addFrame(std::string plotName, const std::vector<T> &vec){
    _checkStart();
    Plot* plot = _getPlotPtr(plotName);
    addFrame(plotName, plot->getX(startT), vec);
}

template <typename T>
inline void PyPlot::addFrame(std::string plotName, double x, const std::vector<T> &vec){
    Plot* plot = _getPlotPtr(plotName);

    // 从std::vector中提取数据添加到曲线
    for(int i(0); i < plot->curveCount; ++i){
        plot->curves[i]->x.push_back(x);
        plot->curves[i]->y.push_back(vec[i]);
    }
}

#endif // PYPLOT_H