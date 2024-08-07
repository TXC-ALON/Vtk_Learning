#ifndef _libslic3r_h_
#define _libslic3r_h_

// this needs to be included early for MSVC (listing it in Build.PL is not enough)
// 头文件包含顺序不可乱动，否则编译报错!!!!!
#include <fstream>
#include <ostream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <queue>
#include <sstream>
#include <vector>
#include <exception>

// 第三方库声明统一放这里!!!!!
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <iconv.h>
#define BOOST_SPIRIT_THREADSAFE // 大幅影响性能？
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/function.hpp>
#include <Eigen/Dense>
// outputdebugview如果不用，可删除此头文件
#include <windows.h>
#include <psapi.h>
#pragma comment(lib, "psapi.lib")

// FIXME This epsilon value is used for many non-related purposes:
//  For a threshold of a squared Euclidean distance,
//  for a trheshold in a difference of radians,
//  for a threshold of a cross product of two non-normalized vectors etc.
#define EPSILON 1e-4
#define EPSILON_1 1e-7
// Scaling factor for a conversion from coord_t to coordf_t: 10e-6
// This scaling generates a following fixed point representation with for a 32bit integer:
// 0..4294mm with 1nm resolution
#define SCALING_FACTOR 0.000001
// RESOLUTION, SCALED_RESOLUTION: Used as an error threshold for a Douglas-Peucker polyline simplification algorithm.
#define RESOLUTION 0.0125
// #define TUQI_RESOLUTION 0.05
//  #define TUQI_RESOLUTION 0.2
#define SCALED_RESOLUTION_SPT (RESOLUTION / SCALING_FACTOR)
#define SCALED_RESOLUTION (500)
#define SCALED_RESOLUTION_ARRANGE (scale_(0.8))

#define PI 3.141592653589793238
// When extruding a closed loop, the loop is interrupted and shortened a bit to reduce the seam.
#define LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER 0.15
// Maximum perimeter length for the loop to apply the small perimeter speed.
#define SMALL_PERIMETER_LENGTH (6.5 / SCALING_FACTOR) * 2 * PI
#define INSET_OVERLAP_TOLERANCE 0.4
#define EXTERNAL_INFILL_MARGIN 3
#define scale_(val) ((val) / SCALING_FACTOR)
#define unscale(val) ((val)*SCALING_FACTOR)
#define SCALED_EPSILON scale_(EPSILON)
typedef long coord_t;
typedef double coordf_t;

// 预估p3d文件打印时间的参数
#define trackingError_IntelliCube14 0.24f // 0.24ms
#define trackingError_IntelliScan20 0.32f // 0.32ms
#define trackingError_ScancubeIII14 0.15f // 0.15ms
#define trackingError_Execute trackingError_ScancubeIII14

// #define JumpSpeed 7000.0f
#define jumpDelay (trackingError_Execute * 2.0f / 0.01f)
#define markDelay (trackingError_Execute * 1.0f / 0.01f)
#define polygonDelay (trackingError_Execute * 0.5f / 0.01f)
// Laser Delay参数以0.5us为单位，因此需要除以0.0005, 增加的0.01是一个10us补偿，对XY2-100协议设置，由于10us是很小的数值，就统一增加。
#define laserOndelay ((trackingError_Execute * 0.6f + 0.01f) / 0.0005f)
#define laserOffdelay ((trackingError_Execute * 1.2f + 0.01f) / 0.0005f)

/* Implementation of CONFESS("foo"): */
#ifdef _MSC_VER
#define CONFESS(...) confess_at(__FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGINFO(...) loginfo_at(__FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#else
#define CONFESS(...) confess_at(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOGINFO(...) loginfo_at(__FILE__, __LINE__, __func__, __VA_ARGS__)
#endif

void confess_at(const char *file, int line, const char *func, const char *pat, ...);
void loginfo_at(const char *file, int line, const char *func, const char *pat, ...);

#define SLIC3R_VERSION GetSoftVersion().c_str()
std::string GetSoftVersion();
int GetMajorVison();
int GetMinorVison();
/* End implementation of CONFESS("foo"): */
int CodeConvert(const char *pFromCode, const char *pToCode, const char *pInBuf, size_t *iInLen, char *pOutBuf, size_t *iOutLen);
int Encode_String(std::string &FilePath);
int Decode_String(std::string &FilePath);
std::wstring Utf8ToUnicode(const std::string &strUTF8);
std::string UnicodeToUtf8(const std::wstring &strUnicode);
std::wstring StringToWString(const std::string &str);
std::string WStringToString(const std::wstring &wstr);
std::string ConverToHexString(std::string &_str);
std::wstring ConverToHexWString(std::wstring &_str);
// 获取系统运行路径
std::string GetRunPath();

//
// void Log_ProcessMemory(std::string _info);

namespace Slic3r
{

    // 轴
    enum Axis
    {
        X = 0,
        Y,
        Z
    };

    // 将一个vector融入另一个vector末尾
    template <class T>
    inline void append_to(std::vector<T> &dst, const std::vector<T> &src)
    {
        dst.insert(dst.end(), src.begin(), src.end());
    }

    // 线程的执行体模板
    template <class T>
    void
    _parallelize_do(std::queue<T> *queue, boost::mutex *queue_mutex, boost::function<void(T)> func)
    {
        LOGINFO("Thread ID [%I64u] queue[%d] +++++Begin+++++", GetCurrentThreadId(), queue->size());
        while (true)
        {
            T i;
            {
                boost::lock_guard<boost::mutex> l(*queue_mutex); // 智能锁  自动析构
                if (queue->empty())
                {
                    LOGINFO("Thread ID [%I64u] queue empty!! +++++Return+++++", GetCurrentThreadId());
                    return;
                }
                i = queue->front();
                queue->pop();
            }
            // std::cout << "  Thread " << boost::this_thread::get_id() << " processing item " << i << std::endl;
            func(i);                                  // 线程真正执行体
            boost::this_thread::interruption_point(); // 线程执行中断点
        }
        LOGINFO("Thread ID [%I64u] -----End------", GetCurrentThreadId());
    }

    // 创建线程
    template <class T>
    void
    parallelize(std::queue<T> queue, boost::function<void(T)> func,
                boost::thread_group *&worker_p,
                int threads_count = boost::thread::hardware_concurrency() / 2)
    {
        if (threads_count == 0)
            threads_count = 2;
        boost::mutex queue_mutex;    // 互斥量
        boost::thread_group workers; // 线程组
        LOGINFO("threads workers create...");
        for (int i = 0; i < std::min(threads_count, (int)queue.size()); i++)
            workers.add_thread(new boost::thread(&_parallelize_do<T>, &queue, &queue_mutex, func));
        worker_p = &workers;
        LOGINFO("waitting for threads...");
        workers.join_all(); // 等待所有线程结束
        worker_p = NULL;
        LOGINFO("threads workers finish...");
    }

    // 线程入口函数
    template <class T>
    void
    parallelize(T start, T end, boost::function<void(T)> func,
                boost::thread_group *&worker_p,
                int threads_count = boost::thread::hardware_concurrency() / 2)
    {
        if (threads_count == 0)
            threads_count = 2;
        std::queue<T> queue;
        for (T i = start; i <= end; ++i)
            queue.push(i);
        parallelize(queue, func, worker_p, threads_count);
    }

    //
    void Turn_On_Log();
    void Turn_Off_Log();
    void Init_Special_CyMesh();
    void Init_CmMesh();
} // namespace Slic3r

using namespace Slic3r;

#endif
