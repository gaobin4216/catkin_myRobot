#ifndef _POSITION_H
#define _POSITION_H  //防止头文件被多次包含

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <string>
#include <array>
#include <limits>
#include <vector>
#include <std_msgs/Float64.h>
#include <boost/array.hpp>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
using namespace std;

#define MY_ID 0x07
#define CAN_DLC 8
#define CAN_ID_BASE 0x600
#define FEEDBACK_ID 1415
#define FEEDBACK_ID_BASE 0x580
namespace bip = boost::interprocess;

namespace position
{
    class PositionControl
    {
        public:
            PositionControl(ros::NodeHandle& nh) ;//构造函数
            void writePulsesPositionsToSharedMemory(bip::managed_shared_memory& segment, const std::vector<std::vector<int32_t>>& pulses_positions);//写入共享内存
            void trajectoryCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr& msg);//接受moveit轨迹
            void trajectoryCallback();//接受预设轨迹
            void readPulsesPositionsFromSharedMemory();
            void feedbackCallback(const can_msgs::Frame::ConstPtr& msg);
            void setSharedMemorySegment(bip::managed_shared_memory* segment);
 
            void SlaveEnable();
            void jointposition_mode();
            void jointenable1();
            void jointenable2();
            void jointenable3();
            void velocityadd();
            void velocitydiff();
            void velocityset();
            void jointstart1();
            void jointstart2();


            
        private:
            const int PULSES_PER_REVOLUTION = 65536; // 电机每转脉冲数/圈
            const double MECHANICAL_RATIO = 101.0;      // 机械传动比

            ros::NodeHandle& nh_;
            ros::Publisher pub_;

            bool isAtTarget = false; // 标志变量，表示电机是否到达目标位置
            size_t currentPointIndex =0; // 当前处理的轨迹点索引
            std::vector<std::vector<double>> positions;  //运动位置
            std::vector<std::vector<int32_t>> pulsesPositions; //运动位置的脉冲数
            
            // 定义共享内存中的容器类型
            typedef bip::allocator<int32_t, bip::managed_shared_memory::segment_manager> ShmemAllocator;
            typedef std::vector<int32_t, ShmemAllocator> MyVector;
            typedef std::vector<MyVector, ShmemAllocator> VectorOfVectors;

            int32_t radiansToPulses(double radians);
            std::vector<std::vector<int32_t>> convertPositionsToPulses(const std::vector<std::vector<double>>& positions);
            void intToFourBytes(int32_t value, int32_t& byte3, int32_t& byte2, int32_t& byte1, int32_t& byte0);
            void positionSet(const std::vector<std::vector<int32_t>>& pulsesPositions);
            bip::managed_shared_memory* sharedMemorySegment = nullptr; // 初始化为空指针
    };
}
#endif