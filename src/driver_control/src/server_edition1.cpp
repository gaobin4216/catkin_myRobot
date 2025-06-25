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
#define SDO_RESPONSE_TIMEOUT_MS 5 // 调整为更合理的响应超时时间
#define SDO_RETRY_COUNT 3         // 单次指令最大重试次数
#define COMMAND_DELAY_US 10000    // 指令间最小间隔10ms
#define PI 3.14159265358979323846
#define MIN_MOVE 100
#define REPEAT 100
#define UNITS 7
#define MIN_100_P 90
#include "driver_control/target_position.h"
#include "driver_control/target_answer.h"

class PDO_listen
{
private:
    int sockfd;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int transmission_type = 1;         // 设定为手收到1个sync后触发pdo
    int PULSES_PER_REVOLUTION = 65536; // 电机（或编码器）旋转一整圈（360度）时产生的脉冲信号数量

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

    // 私有辅助函数：验证接收帧是否匹配预期
    bool validateResponseFrame(const struct can_frame &frame, uint32_t expectedId)
    {
        if ((frame.can_id & CAN_EFF_MASK) != expectedId)
        {
            std::cerr << "错误：接收到非预期的CAN ID，期望0x" << std::hex << expectedId
                      << " 实际0x" << (frame.can_id & CAN_EFF_MASK) << std::dec << std::endl;
            return false;
        }
        if (frame.can_dlc < 4)
        {
            std::cerr << "错误：接收到无效的SDO响应，数据长度不足" << std::endl;
            return false;
        }
        return true;
    }

    // 输出vector内容
    void cout_vector(const std::vector<int> &out)
    {
        for (int i = 0; i < 6; i++)
        {
            std::cout << out[i] << " ";
        }
        std::cout << out[6] << std::endl;
    }
    void cout_vector(const std::vector<double> &out)
    {
        for (int i = 0; i < 6; i++)
        {
            std::cout << out[i] << " ";
        }
        std::cout << out[6] << std::endl;
    }

public:
    // 构造函数，创建并初始化 CAN 总线，在制定的can接口上（默认can0）初始化一个socketcan通信通道
    PDO_listen(const char *interface = "can0")
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

    // 参数定义
    std::vector<double> MECHANICAL_RATIO = {101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 81.0};                                                                  // 电机减速比
    std::vector<double> TF1_RATIO = {549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 549756.0 / 29.9, 440893.0 / 29.9}; // 电机输出端每度对应的输入脉冲值
    std::vector<double> VELOCITY_RATIO = {28.056, 28.056, 28.056, 28.056, 28.056, 28.056, 22.5};                                                              // 电机输出端1度每秒对应的输入速度值
    std::vector<double> EFFORT_RATIO = {3.3, 3.3, 49, 49, 49, 49, 8.6};                                                                                       // ？单位NM,读数乘以它就是力矩
    std::vector<int> position = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> angle = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> velocity = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> effort = {0, 0, 0, 0, 0, 0, 0};
    // std::vector<double> CURRENT_RATIO = {6.1, 6.1, 5.0, 5.0, 5.0, 5.0, 3.6};
    //  std::vector<double> EFFORT_RATIO = {2.0 / 1000 * 0.118, 2.0 / 1000 * 0.118, 9.8 / 1000 * 0.096, 9.8 / 1000 * 0.096, 9.8 / 1000 * 0.096, 9.8 / 1000 * 0.096, 5.3 / 1000 * 0.089};
    //   std::vector<double> EFFORT_RATIO = {6.1 / 1000, 6.1 / 1000, 5 / 1000, 5 / 1000, 5 / 1000, 5 / 1000, 3.6 / 1000};
    // std::vector<int> bias_p = {0, 0, 0, 0, 0, 0, 0};
    /*std::vector<double> acc = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> de_acc = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> ctrl_p = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> ctrl_i = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> ctrl_d = {0, 0, 0, 0, 0, 0, 0};*/

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

    // 发送 PDO模式的CAN 帧，无返回值
    void send_pdo_Frame(uint32_t id, uint8_t *data, uint8_t dlc, bool extended = false)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = id;
        if (extended)
            frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = std::min(dlc, (uint8_t)8); // 严格限制数据长度

        memcpy(frame.data, data, frame.can_dlc);
        write(sockfd, &frame, sizeof(frame));
    }

    // 将客户端发来的指令通过pdo发给电机
    void client_out(const std::vector<double> &data)
    {
        int temp_position;
        for (int i = 0; i < UNITS; i++)
        {
            temp_position = data[i] * TF1_RATIO[i] / 1; // 将目标的角度转换为脉冲数。
            // std::cout << temp_position << std::endl;
            if (std::abs(temp_position - position[i]) > MIN_MOVE)
            {
                // temp_position = temp_position + position[i];
                pdo_move(i + 0x201, temp_position);
                std::cout << std::hex << i + 0x201 << std::dec << "  " << temp_position << std::endl;
            }
        }
        ROS_INFO("Move end:");
        // sleep(20);
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
            // std::cerr << "接收超时（" << timeout_ms << "ms）" << std::endl;
            return false;
        }

        ssize_t readResult = read(sockfd, &frame, sizeof(frame));
        if (readResult != sizeof(frame))
        {
            std::cerr << "CAN帧读取失败: " << strerror(errno) << std::endl;
            return false;
        }
        // if ((int)frame.data[0]==96) std::cout<<"order complete"<<std::endl;
        return true;
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
    int FourBytesToint(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0)
    {
        int i1 = int(byte0);
        // std::cout<<i1<<std::endl;
        i1 = i1 * 16 * 16 + int(byte1);
        // std::cout<<i1<<std::endl;
        i1 = i1 * 16 * 16 + int(byte2);
        // std::cout<<i1<<std::endl;
        i1 = i1 * 16 * 16 + int(byte3);
        // std::cout<<i1<<std::endl;
        return i1;
    }

    // 将返回的脉冲和速度解析
    void decode_posi_velo(struct can_frame &frame)
    {
        if (frame.can_id < 0x200 && frame.can_id > 0x180)
        {
            int id = frame.can_id - 0x181;
            position[id] = FourBytesToint(frame.data[0], frame.data[1], frame.data[2], frame.data[3]); // 实际角度对应的脉冲
            // std::cout<<int(byte3)<<std::endl;
            // position[id] = position[id] + bias_p[id];
            angle[id] = double(position[id]) / TF1_RATIO[id];                                                               // 实际角度
            velocity[id] = FourBytesToint(frame.data[4], frame.data[5], frame.data[6], frame.data[7]) / VELOCITY_RATIO[id]; // 实际的速度，单位：度/s
        }
    }

    // 将返回的力矩析
    void decode_effr(struct can_frame &frame)
    {
        if (frame.can_id < 0x300 && frame.can_id > 0x280)
        {
            int id = frame.can_id - 0x281;
            int16_t temp = FourBytesToint(frame.data[0], frame.data[1], 0, 0);
            effort[id] = temp * EFFORT_RATIO[id] / 1000;

            /*ctrl_p[id] = FourBytesToint(frame.data[2], frame.data[3], 0, 0);
            ctrl_i[id] = FourBytesToint(frame.data[4], frame.data[5], 0, 0);
            ctrl_d[id] = FourBytesToint(frame.data[6], frame.data[7], 0, 0);*/
        }
    }
    /*
    void decode_acc_de_acc(struct can_frame &frame)
    {
        if (frame.can_id < 0x400 && frame.can_id > 0x380)
        {
            int id = frame.can_id - 0x381;
            acc[id] = FourBytesToint(frame.data[0], frame.data[1], frame.data[2], frame.data[3]) / TF1_RATIO[id];
            de_acc[id] = FourBytesToint(frame.data[4], frame.data[5], frame.data[6], frame.data[7]) / TF1_RATIO[id];
        }
    }
    */
    // PDO控制电机运动到指定位置
    void pdo_move(int RequestId, uint32_t position_p)
    {
        uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
        for (int i = 0; i < REPEAT; i++)
        {

            send_pdo_Frame(RequestId, data, sizeof(data));
        }
        send_sync();

        uint8_t data3[] = {0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00};
        for (int i = 0; i < REPEAT; i++)
        {
            send_pdo_Frame(RequestId, data3, sizeof(data3));
        }
        send_sync();

        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position_p, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {byte3, byte2, byte1, byte0, 0x1f, 0, 0, 0};
        send_pdo_Frame(RequestId, data4, sizeof(data4));

        for (int i = 0; i < REPEAT; i++)
        {
            send_pdo_Frame(RequestId, data4, sizeof(data4));
        }
        send_sync();
    }

    // 输出反馈的数据
    void print_out()
    {
        std::cout << "position:  ";
        cout_vector(position);
        std::cout << "angle:  ";
        cout_vector(angle);
        std::cout << "velocity:  ";
        cout_vector(velocity);
        std::cout << "effort:  ";
        cout_vector(effort);
        /*std::cout << "acc:  ";
        cout_vector(acc);
        std::cout << "de_acc:  ";
        cout_vector(de_acc);*/
        /*std::cout << "P:  ";
        cout_vector(ctrl_p);
        std::cout << "I:  ";
        cout_vector(ctrl_i);
        std::cout << "D:  ";
        cout_vector(ctrl_d);*/
        std::cout << std::endl;
    }
};

class Server
{
private:
    std::mutex data_mutex_;
    std::vector<std::string> joint_names = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7"};

public:
    Server() {}
    PDO_listen tpdo{"can0"};
    std::vector<double> previous_position = {0, 0, 0, 0, 0, 0, 0};
    int complete_number = UNITS; // 7
    bool new_data_received_ = false;
    std::vector<double> delta_data_ = {0, 0, 0, 0, 0, 0, 0};

    void one_by_one_divide()
    {
        complete_number = 0;
        for (int i = 0; i < UNITS; i++)
        {
            // std::cout << delta_data_[i] << "@@@" << previous_position[i] << std::endl;
            if (delta_data_[i] < MIN_MOVE / tpdo.TF1_RATIO[i])
            {
                complete_number = complete_number + 1;
            }
            else
            {
                if (std::abs((tpdo.angle[i] - previous_position[i]) / delta_data_[i]) / 0.01 > MIN_100_P)
                {
                    complete_number = complete_number + 1;
                }
            }
        }
        // std::cout << "one_by_one_divide complete!" << std::endl;
    }

    void arrayCallback1(const driver_control::target_position::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < 7; i++)
        {
            delta_data_[i] = msg->data[i];
            previous_position[i] = tpdo.angle[i];
        }
        new_data_received_ = true;
    }

    void processData()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (new_data_received_)
        {
            ROS_INFO("New data received:");
            /*for(int i = 0; i < 7; i++) {
                ROS_INFO("Element %d: %f", i, latest_data_[i]);
            }*/
            tpdo.client_out(delta_data_);
            new_data_received_ = false;
        }
        else
        {
            ROS_DEBUG("No new data received, continuing normal operation...");
            // 这里可以添加服务端自己的处理逻辑
        }
    }

    void run(int frequency)
    {
        ros::NodeHandle nh;
        struct can_frame frame;
        std::vector<double> joint_positions(joint_names.size(), 0.0);
        std::vector<double> joint_velocities(joint_names.size(), 0.0);
        std::vector<double> joint_efforts(joint_names.size(), 0.0);

        // 发布关节状态
        ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

        // 订阅轨迹点
        bool need_subscribe = true; // 控制订阅的标志位
        ros::Subscriber sub;        // 声明订阅对象（不立即初始化）

        // ros::Subscriber sub = nh.subscribe("target_array", 10, &Server::arrayCallback1, this);
        //  ros::ServiceServer service = nh.advertiseService("target_array", &Server::arrayCallback1, this);

        ros::Rate loop_rate(frequency); // 10Hz
        ROS_INFO("server is running...");
        int pre_complete_num = UNITS;

        while (ros::ok())
        {
            // 按需重新订阅
            if (need_subscribe)
            {
                sub = nh.subscribe("target_array", 10, &Server::arrayCallback1, this);
                need_subscribe = false; // 重置标志位
                ROS_INFO("Resubscribed to target_array");
            }

            // Create JointState message
            sensor_msgs::JointState joint_state;

            // Set header with current time
            joint_state.header.stamp = ros::Time::now();

            // Set joint names
            joint_state.name = joint_names;

            tpdo.send_sync();
            for (int i = 0; i < 21; i++)
            {
                if (tpdo.receiveFrame(frame))
                {
                    tpdo.decode_posi_velo(frame);
                    tpdo.decode_effr(frame);
                    // tpdo.decode_acc_de_acc(frame);
                }
            }
            tpdo.print_out();
            // Set joint positions (modify with your actual values)
            for (size_t i = 0; i < joint_positions.size(); ++i)
            {
                
                joint_positions[i] = tpdo.angle[i] / 180 * PI;
                joint_velocities[i] = tpdo.velocity[i] / 180 * PI;
                joint_efforts[i] = tpdo.effort[i];
                if(i%2==0){
                    joint_positions[i] = -joint_positions[i];
                }
            }
            joint_state.position = joint_positions;
            joint_state.velocity = joint_velocities;
            joint_state.effort = joint_efforts;

            // Publish the message,会在中间加入0
            joint_state_pub.publish(joint_state);

            ros::spinOnce();

            if (new_data_received_)
            {
                processData();
                new_data_received_ = false; // 重置接收标志
                // 取消当前订阅但不删除对象
                sub.shutdown();
                need_subscribe = true; // 标记需要重新订阅
            }
            if (pre_complete_num < UNITS && complete_number == UNITS)
            {
                need_subscribe = true; // 满足条件时标记需要重新订阅
                // ros::Subscriber sub = nh.subscribe("target_array", 10, &Server::arrayCallback1, this);
            }
            one_by_one_divide();
            pre_complete_num = complete_number;

            loop_rate.sleep();
            std::cout << new_data_received_ << "  " << pre_complete_num << "  " << need_subscribe << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    Server server;
    server.run(100);
    return 0;
}
