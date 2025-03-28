#include <ros/ros.h>
#include <position_control/position.h>
#include <cmath>

//position命名空间下PositionControl类的函数实现
namespace position//命名空间，申明作用域
{
    //构造函数定义，初始化列表传入节点句柄，后缀_表示私有成员变量，sharedMemorySegment指针初始化为nullptr
    PositionControl::PositionControl(ros::NodeHandle& nh) : nh_(nh), sharedMemorySegment(nullptr) {}
    // 设置共享内存段，将在该节点处理好的数组放入，以便其他节点读取
    void PositionControl::setSharedMemorySegment(bip::managed_shared_memory* segment) 
    {
        //检查传入的共享内存段是否有效，检查内存段完整性
        if (segment != nullptr && segment->check_sanity())  {
            sharedMemorySegment = segment;
        } 
        else {
            throw std::runtime_error("Invalid shared memory segment");
        }
    }
    //写入预设轨迹或者读取轨迹
    void PositionControl::trajectoryCallback()
    {
        // 定义一个二维数组来存储所有轨迹点的关节位置，内层 vector 仅存储单个 double 值
        std::vector<std::vector<double>> positions;
        /*for(int i=0;i<360;i++){
            positions.push_back({M_PI*pow(-1,i)/2}); 
        }*/
       
        positions.push_back({0}); 
        positions.push_back({M_PI/6});  
        positions.push_back({M_PI/3}); 
        positions.push_back({M_PI/2}); 
        positions.push_back({M_PI*2/3}); 
        positions.push_back({M_PI*5/6}); 
        positions.push_back({M_PI}); 
        positions.push_back({M_PI*5/6}); 
        positions.push_back({M_PI*2/3}); 
        positions.push_back({M_PI/2}); 
        positions.push_back({M_PI/3}); 
        positions.push_back({M_PI/6}); 
        positions.push_back({M_PI/4}); 
        positions.push_back({0});
        

        // 获取二维数组的行数和列数
        size_t num_points = positions.size(); // 行数，即点的数量
        size_t num_positions_per_point = positions.size();
        if (!positions.empty()) 
        {
            num_positions_per_point = positions[0].size(); // 列数，即每个点的关节位置数量
        }

        // 打印二维数组的维数
        ROS_INFO("Number of points : %zu", num_points);
        ROS_INFO("Number of joint positions at each point : %zu", num_positions_per_point);
        
        // 将装有位置信息的二维数组转换为脉冲数
        pulsesPositions = convertPositionsToPulses(positions);

        // 将脉冲数写入共享内存
        if (sharedMemorySegment) 
        {
            writePulsesPositionsToSharedMemory(*sharedMemorySegment, pulsesPositions);
            ROS_INFO("Data written to shared memory.");
        } 
        else 
        {
            ROS_ERROR("Shared memory segment not initialized.");
        }

    };
    // 将弧度转换为脉冲数(电机的位置单位是cnt)
    int32_t PositionControl::radiansToPulses(double radians) 
    {
        double pulses = radians * (PULSES_PER_REVOLUTION / (2.0 * M_PI)) * MECHANICAL_RATIO;
        return static_cast<int32_t>(pulses);
    };
    // 将装有位置信息的二维数组转换为脉冲数
    std::vector<std::vector<int32_t>> PositionControl::convertPositionsToPulses(const std::vector<std::vector<double>>& positions) 
    {
        std::vector<std::vector<int32_t>> pulses_positions(positions.size(), std::vector<int32_t>(positions[0].size()));
        for (size_t i = 0; i < positions.size(); ++i) {
            for (size_t j = 0; j < positions[i].size(); ++j) {
                pulses_positions[i][j] = radiansToPulses(positions[i][j]);
            }
        }
        return pulses_positions;
    };
    //将脉冲数写入共享内存
    void PositionControl::writePulsesPositionsToSharedMemory(bip::managed_shared_memory& segment, const std::vector<std::vector<int32_t>>& pulsesPositions) 
    {
        try{
            // 创建分配器和共享内存结构
            ShmemAllocator alloc_inst(segment.get_segment_manager());
            VectorOfVectors* pulsesPositions_shmem = segment.construct<VectorOfVectors>("PulsesPositions")(alloc_inst);

            // 将数据从标准向量复制到共享内存向量
            for (const auto& inner_vec : pulsesPositions) {
                MyVector* inner_shmem_vec = segment.construct<MyVector>(bip::anonymous_instance)(alloc_inst);
                for (const auto& value : inner_vec) {
                    inner_shmem_vec->push_back(value);
                    }
                pulsesPositions_shmem->push_back(*inner_shmem_vec);
                }
            } 
        catch (const bip::interprocess_exception& ex){
            ROS_ERROR("写入共享内存时出错: %s", ex.what());
        }
    };    
}

int main(int argc, char *argv[]) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "GetTrajectory");
    // 创建节点句柄
    ros::NodeHandle nh;

    // 删除已存在的共享内存段，以免影响新的创建
    bip::shared_memory_object::remove("MySharedMemory");
    // 创建共享内存段
    bip::managed_shared_memory sharedMemorySegment(
        bip::open_or_create, //模式：存在则打开，否则创建
        "MySharedMemory", //共享内存标识名（需唯一）
        1024 * 1024 * 2); // 大小：2MB，单位字节数，2M的bytes

    //调用参数构造函数，将节点句柄传入
    position::PositionControl positionControl(nh);
    //设置构造函数的内存段
    positionControl.setSharedMemorySegment(&sharedMemorySegment);
    //写入预设轨迹或者读取轨迹
    positionControl.trajectoryCallback();
    return 0;
}