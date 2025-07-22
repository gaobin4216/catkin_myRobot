#include <cstdlib>
#include "ros/ros.h"
#include <iostream>
#include <cstring>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <manipulator_control/target_position.h>

// CAN通信相关常量定义
#define SDO_RESPONSE_TIMEOUT_MS 5 // SDO响应超时时间(毫秒)
#define SDO_RETRY_COUNT 3         // 单次指令最大重试次数
#define COMMAND_DELAY_US 10000    // 指令间最小间隔(微秒)
#define PI 3.14159265358979323846 // 圆周率
#define MIN_MOVE 50               // 最小运动检测阈值(脉冲数)
#define UNITS 7                   // 电机/关节数量
#define MIN_100_P 0.95            // 运动完成百分比阈值(90%)

class PDO_listen
{
private:
    std::vector<bool> motor_enabled_; // 跟踪电机使能状态
    int sockfd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 私有辅助函数：带重试机制的帧发送
    bool sendFrameWithRetry(uint32_t id, uint8_t *data, uint8_t dlc, bool extended = false)
    {
        for (int retry = 0; retry < SDO_RETRY_COUNT; retry++)
        {
            if (sendFrame(id, data, dlc, extended))
            {
                return true;
            }
        }
        std::cerr << "发送失败，超过最大重试次数(" << SDO_RETRY_COUNT << ")" << std::endl;
        return false;
    }

public:
    // 参数定义
    std::vector<double> MECHANICAL_RATIO = {101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 81.0};                                                                 // 电机减速比
    std::vector<double> TF1_RATIO = {549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 440893.0 / 29.9}; // 电机输出端每度对应的输入脉冲值
    std::vector<double> VELOCITY_RATIO = {28.08, 28.08, 28.08, 28.08, 28.08, 28.08, 22.527};                                                                 // 电机输出端1度每秒对应的输入速度值
    std::vector<double> EFFORT_RATIO = {3.3, 3.3, 49, 49, 49, 49, 8.6};                                                                                      // 单位mNM,读数乘以它就是力矩

    // 电机状态
    std::mutex data_mutex_; // 加锁，防止数据被同时读写
    std::vector<int> position = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> angle = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> velocity = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> effort = {0, 0, 0, 0, 0, 0, 0};

    // 构造函数，创建并初始化 CAN 总线，在制定的can接口上（默认can0）初始化一个socketcan通信通道,在构造函数中初始化使能状态
    PDO_listen(const char *interface = "can0") : motor_enabled_(UNITS, false)
    {
        // 创建 socket
        if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            throw std::runtime_error("Socket创建失败: " + std::string(strerror(errno)));
        }

        // 绑定到 CAN 接口
        strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
        if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
        {
            close(sockfd);
            throw std::runtime_error("I/O控制失败: " + std::string(strerror(errno)));
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(sockfd);
            throw std::runtime_error("绑定失败: " + std::string(strerror(errno)));
        }

        std::cout << "CAN总线在" << interface << "上初始化完成" << std::endl;
    }

    // 发送 CAN 帧（基础版），成功发送会返回true
    bool sendFrame(uint32_t id, uint8_t *data, uint8_t dlc, bool extended = false)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = id;
        if (extended)
            frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = std::min(dlc, (uint8_t)8); // 严格限制数据长度

        memcpy(frame.data, data, frame.can_dlc);

        ssize_t writeResult = write(sockfd, &frame, sizeof(frame)); // 返回帧大小
        if (writeResult != sizeof(frame))
        {
            std::cerr << "CAN帧发送失败: " << strerror(errno) << std::endl;
            return false;
        }
        usleep(COMMAND_DELAY_US);
        return true;
    }

    // 接收 CAN 帧（增强版）
    bool receiveFrame(struct can_frame &frame, int timeout_ms = SDO_RESPONSE_TIMEOUT_MS)
    {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(sockfd, &readSet);

        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(sockfd + 1, &readSet, NULL, NULL, &timeout);
        if (ret < 0)
        {
            std::cerr << "Select错误: " << strerror(errno) << std::endl;
            return false;
        }
        if (ret == 0)
        {
            return false;
        }

        ssize_t readResult = read(sockfd, &frame, sizeof(frame));
        if (readResult != sizeof(frame))
        {
            std::cerr << "CAN帧读取失败: " << strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    // PDO控制电机运动到指定位置
    void pdo_move(int RequestId, uint32_t position_p)
    {
        // 只在需要时改变使能状态
        int motor_idx = RequestId - 0x201;
        if (!motor_enabled_[motor_idx])
        {
            // 下使能
            uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
            if (!sendFrameWithRetry(RequestId, data, sizeof(data)))
            {
                throw std::runtime_error("下使能失败");
            }
            // 上使能
            uint8_t enable_data[] = {0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00};
            if (!sendFrameWithRetry(RequestId, enable_data, sizeof(enable_data)))
            {
                throw std::runtime_error("上使能失败");
            }
            motor_enabled_[motor_idx] = true;
        }

        // 命令触发
        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position_p, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {byte3, byte2, byte1, byte0, 0x1f, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(RequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("目标位置发送失败");
        }
    }

    // 添加关闭时禁用所有电机的函数
    void disable_all_motors()
    {
        uint8_t disable_data[] = {0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
        for (int i = 0; i < UNITS; ++i)
        {
            if (motor_enabled_[i])
            {
                sendFrameWithRetry(i + 0x201, disable_data, sizeof(disable_data));
                motor_enabled_[i] = false;
            }
        }
    }

    // 将客户端发来的指令通过pdo发给电机
    void client_out(const std::vector<double> &data)
    {
        std::vector<int> temp_positions(UNITS);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < UNITS; i++)
            {
                temp_positions[i] = data[i] * TF1_RATIO[i];
            }
        }

        for (int i = 0; i < UNITS; i++)
        {
            if (std::abs(temp_positions[i] - position[i]) > MIN_MOVE)
            {
                pdo_move(i + 0x201, temp_positions[i]);
            }
        }
    }

    // 发送sync同步帧0x080，单向广播​​，不要求接收方回复，​​SYNC 帧本身不要求回复​​，但会触发节点发送 PDO
    void send_sync()
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = 0x080;
        frame.can_dlc = 0;
        write(sockfd, &frame, sizeof(frame));
    }

    // 16进制四字节整数转换为16进制的四个单字节整数，小端序
    void intToFourBytes(int32_t value, uint8_t &byte3, uint8_t &byte2, uint8_t &byte1, uint8_t &byte0)
    {
        byte0 = (value >> 24) & 0xFF; // 最高位字节，存在小端0x00，最右边
        byte1 = (value >> 16) & 0xFF;
        byte2 = (value >> 8) & 0xFF;
        byte3 = value & 0xFF; // 最低位字节
    }

    // 16进制的四个单字节整数转换为16进制四字节整数
    int32_t FourBytesToint(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0)
    {
        return (static_cast<int32_t>(byte0) << 24) |
               (static_cast<int32_t>(byte1) << 16) |
               (static_cast<int32_t>(byte2) << 8) |
               static_cast<int32_t>(byte3);
    }

    // 将返回的脉冲和速度解析
    void decode_posi_velo(struct can_frame &frame)
    {
        if (frame.can_id < 0x200 && frame.can_id > 0x180)
        {
            int id = frame.can_id - 0x181;                                                                                  // 计算电机ID-1
            position[id] = FourBytesToint(frame.data[0], frame.data[1], frame.data[2], frame.data[3]);                      // 实际角度对应的脉冲
            angle[id] = double(position[id]) / TF1_RATIO[id];                                                               // 实际角度
            velocity[id] = FourBytesToint(frame.data[4], frame.data[5], frame.data[6], frame.data[7]) / VELOCITY_RATIO[id]; // 实际的速度，单位：度/s
        }
    }

    // 将返回的力矩解析
    void decode_effr(struct can_frame &frame)
    {
        if (frame.can_id < 0x300 && frame.can_id > 0x280)
        {
            int id = frame.can_id - 0x281;
            int16_t temp = FourBytesToint(frame.data[0], frame.data[1], 0, 0); // 力矩实际只有16位
            effort[id] = temp * EFFORT_RATIO[id] / 1000;
        }
    }

    // 输出反馈的数据
    void print_out()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ROS_INFO_STREAM("position:" << position[0] << " " << position[1] << " " << position[2] << " "
                                    << position[3] << " " << position[4] << " " << position[5] << " " << position[6]);
        ROS_INFO_STREAM("angle: " << angle[0] << " " << angle[1] << " " << angle[2] << " "
                                  << angle[3] << " " << angle[4] << " " << angle[5] << " " << angle[6]);
        ROS_INFO_STREAM("velocity: " << velocity[0] << " " << velocity[1] << " " << velocity[2] << " "
                                     << velocity[3] << " " << velocity[4] << " " << velocity[5] << " " << velocity[6]);
        ROS_INFO_STREAM("effort: " << effort[0] << " " << effort[1] << " " << effort[2] << " "
                                   << effort[3] << " " << effort[4] << " " << effort[5] << " " << effort[6]);
    }

    // 析构函数
    ~PDO_listen()
    {
        if (sockfd >= 0)
        {
            close(sockfd);
            sockfd = -1;
        }
    };
};

class Server
{
private:
    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"}; // 关节名称
    sensor_msgs::JointState joint_state;                                                                           // 关节状态消息

    std::atomic<bool> running_{true}; // 线程运行标志
    std::thread listener_thread_;     // 监听线程
    std::thread controller_thread_;   // 控制线程

public:
    Server()
    {        
        joint_state.position.resize(UNITS, 0.0);
        joint_state.velocity.resize(UNITS, 0.0);
        joint_state.effort.resize(UNITS, 0.0);
    }
    PDO_listen tpdo{"can0"};                                     // 实例化PDO_listen类为tpdo，// CAN通信对象
    std::vector<double> target_position = {0, 0, 0, 0, 0, 0, 0}; // 目标位置，关节角度
    int complete_number = 0;                                     // 完成运动的关节数
    std::atomic<bool> new_data_received_{false};                 // 原子变量，无需锁保护,新数据接收标志

    // 目标位置回调函数
    void arrayCallback1(const manipulator_control::target_position::ConstPtr &msg)
    {
        for (int i = 0; i < UNITS; i++)
        {
            target_position[i] = msg->data[i];
        }
        new_data_received_ = true; // 新数据已接收
    }

    // 检查运动完成情况
    void motionState_check()
    {
        int temp_complete = 0;
        for (int i = 0; i < UNITS; i++)
        {
            std::lock_guard<std::mutex> lock(tpdo.data_mutex_);
            // 检查是否到位
            if (std::abs(target_position[i] - tpdo.angle[i]) <= MIN_MOVE / tpdo.TF1_RATIO[i])
            {
                temp_complete++;
            }
        }
        complete_number = temp_complete;
    }

    // 监听线程函数
    void listenerThreadFunc()
    {
        ros::Rate rate(50); // 监听频率
        ros::NodeHandle nh;
        ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10); // 关节状态发布器
        while (running_ && ros::ok())
        {
            can_frame frame = {0};
            tpdo.send_sync(); // 发送SYNC触发PDO传输

            // 接收并解析14个PDO帧(7个电机×2种PDO)，为了保险，增加超时，默认5，这里10
            for (int i = 0; i < 14; i++)
            {
                std::lock_guard<std::mutex> lock(tpdo.data_mutex_);
                if (tpdo.receiveFrame(frame))
                {
                    tpdo.decode_posi_velo(frame);
                    tpdo.decode_effr(frame);
                }
            }

            // 输出状态
            tpdo.print_out();

            // 发布状态     
            joint_state.name = joint_names;       
            for (int i = 0; i < UNITS; i++)
            {
                std::lock_guard<std::mutex> lock(tpdo.data_mutex_);
                joint_state.position[i] = tpdo.angle[i] / 180 * PI;
                joint_state.velocity[i] = tpdo.velocity[i] / 180 * PI;
                joint_state.effort[i] = tpdo.effort[i];
            }
            joint_state.header.stamp = ros::Time::now();            
            joint_state_pub.publish(joint_state);
            rate.sleep(); // 保持频率
        }
    }

    // 控制线程函数
    void controllerThreadFunc()
    {
        ros::Rate rate(50); // 频率
        ros::NodeHandle nh;
        ros::Subscriber sub;
        std::atomic<bool> need_subscribe{true}; // 订阅标志

        while (running_ && ros::ok())
        {
            // 处理订阅
            if (need_subscribe)
            {
                need_subscribe = false; // 暂时无需订阅
                sub = nh.subscribe("target_array", 10, &Server::arrayCallback1, this);
            }
            ros::spinOnce(); // 触发回调

            // 处理新数据，控制电机
            if (new_data_received_)
            {
                new_data_received_ = false;
                tpdo.client_out(target_position);
                sub.shutdown(); // 取消订阅
            }

            // 运动监控
            motionState_check();

            // 检查运动完成情况
            if (complete_number == UNITS)
            {
                complete_number = 0;   // 完成运动的关节数
                need_subscribe = true; // 运动完成后准备重新订阅
            }

            rate.sleep(); // 保持频率
        }
    }

    // 启动服务器
    void run()
    {
        listener_thread_ = std::thread(&Server::listenerThreadFunc, this);
        controller_thread_ = std::thread(&Server::controllerThreadFunc, this);
        std::cout << "服务器已启动，监听线程和控制线程同步运行" << std::endl;
    }

    // 停止服务器
    void stop()
    {
        running_ = false; // 设置停止标志
        if (listener_thread_.joinable())
            listener_thread_.join();
        if (controller_thread_.joinable())
            controller_thread_.join();
        ros::shutdown();
    }

    // 析构函数
    ~Server()
    {
        stop();                    // 析构时确保线程停止
        tpdo.disable_all_motors(); // 析构时下使能所有电机
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_server"); // 初始化节点
    ros::NodeHandle nh;                    // 确保ROS完全初始化
    Server server;
    server.run();

    ros::spin(); // 处理回调函数
    return 0;
}
