// 初始化CAN通信，设置PDO参数，可简单测试控制通信
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

#define SDO_RESPONSE_TIMEOUT_MS 500 // 调整为更合理的响应超时时间
#define COMMAND_DELAY_US 10000      // 指令间最小间隔10ms

class CANBus
{
private:
    int sockfd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    int transmission_type = 1; // 传输值，收到设定值个 SYNC 后才会触发 PDO
    int InhibitTime = 1;       // 抑制禁止时间，即定义两个连续 PDO 传输的最小间隔时间，避免数据竞争总线的问题。单位 100us。

    /* 这两个辅助函数适用于对通信要求严格的SDO通信 */
    // 私有辅助函数：带重试机制的帧发送
    bool sendFrameWithRetry(uint32_t id, uint8_t *data, uint8_t dlc, bool extended = false)
    {
        for (int retry = 0; retry < 3; retry++)
        {
            if (sendFrame(id, data, dlc, extended))
            {
                return true;
            }
        }
        std::cerr << "发送失败，超过最大重试次数(" << 3 << ")" << std::endl;
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

public:
    // 构造函数，CAN接口名称默认值为 "can0"
    CANBus(const char *interface = "can0")
    {
        // 创建 socket
        if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            throw std::runtime_error("Socket创建失败: " + std::string(strerror(errno)));
        }

        // 绑定到 CAN 接口
        strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);

        // 获取指定网络接口的索引号（ifindex）
        if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
        {
            close(sockfd);
            throw std::runtime_error("I/O控制失败: " + std::string(strerror(errno)));
        }

        // 配置 Socket 地址​​
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        // 将 Socket (sockfd) 绑定到指定的 CAN 接口（通过 addr）

        if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(sockfd);
            throw std::runtime_error("绑定失败: " + std::string(strerror(errno)));
        }

        std::cout << "CAN总线在" << interface << "上初始化完成" << std::endl;
    }

    // 发送 CAN 帧（基础版），发送 CAN 帧（基础版），成功发送会返回true
    bool sendFrame(uint32_t id, uint8_t *data, uint8_t dlc, bool extended = false)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = id;
        if (extended)
            frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = std::min(dlc, (uint8_t)8); // 严格限制数据长度

        memcpy(frame.data, data, frame.can_dlc);

        ssize_t writeResult = write(sockfd, &frame, sizeof(frame)); // 返回帧大小，返回值是实际写入的字节数
        if (writeResult != sizeof(frame))
        {
            std::cerr << "CAN帧发送失败: " << strerror(errno) << std::endl;
            return false;
        }
        usleep(COMMAND_DELAY_US);
        return true;
    }

    // 接收 CAN 帧（增强版），接收 CAN 帧（增强版），等待并接收一帧，在限定的超时间隔内成功接收会返回true
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
            std::cerr << "接收超时（" << timeout_ms << "ms）" << std::endl;
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
        byte0 = (value >> 24) & 0xFF; // 最高位字节，，存在小端0x00，最右边
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

    // 初始化函数，设置节点控制模式，1-轮廓位置模式，2-轮廓速度模式，0x606000
    void init(uint8_t model)
    {
        struct can_frame frame;           // 定义一个 CAN 帧结构体
        memset(&frame, 0, sizeof(frame)); // 将结构体内存清零

        // 设备初始化之后，自动进入预操作状态
        // 0x80，预操作状态，可以SDO，操作状态才可以PDO，以防万一，这里主动用NMT将电机置于预操作状态,NMT广播
        uint8_t data[] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // NMT 管理，0x000
        if (!sendFrameWithRetry(0x000, data, sizeof(data)))
        {
            throw std::runtime_error("远程节点进入预操作状态失败");
        }

        // 设置7个关节的控制模式
        for (int i = 1; i < 8; i++)
        {
            uint32_t sdoRequestId = 0x600 + i;  // SDO请求ID
            uint32_t sdoResponseId = 0x580 + i; // SDO响应ID
            uint8_t data1[] = {0x2F, 0x60, 0x60, 0x00, model, 0x00, 0x00, 0x00};
            if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
            {
                throw std::runtime_error("模式设置指令发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("模式设置未收到有效响应");
            }
        }
    }

    // 设置速度0x6061、加速度0x6083和减速度0x6084
    void set_velo_acc_dacc(int acce_max = 0x10BE, int de_acce_max = 0x10BE, int velo_max = 0x10BE)
    {
        struct can_frame frame;           // 定义一个 CAN 帧结构体
        memset(&frame, 0, sizeof(frame)); // 将结构体内存清零

        uint8_t byte3, byte2, byte1, byte0;
        // 模式设置（添加响应验证）
        for (int i = 1; i < 8; i++)
        {
            uint32_t sdoRequestId = 0x600 + i;          // SDO请求ID（根据实际协议可能需要调整）
            uint32_t sdoResponseId = 0x600 + i - 0x080; // SDO响应ID（假设标准从站响应ID为0x580+节点ID）

            intToFourBytes(acce_max, byte3, byte2, byte1, byte0);
            uint8_t data2[] = {0x23, 0x83, 0x60, 0x00, byte3, byte2, byte1, byte0};
            if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
            {
                throw std::runtime_error("加速度设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("加速度设置未收到有效响应");
            }

            intToFourBytes(de_acce_max, byte3, byte2, byte1, byte0);
            uint8_t data3[] = {0x23, 0x84, 0x60, 0x00, byte3, byte2, byte1, byte0};
            if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
            {
                throw std::runtime_error("减速度设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("减速度设置未收到有效响应");
            }

            intToFourBytes(velo_max, byte3, byte2, byte1, byte0);
            uint8_t data4[] = {0x23, 0x81, 0x60, 0x00, byte3, byte2, byte1, byte0};
            if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
            {
                throw std::runtime_error("最大速度设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("最大速度设置未收到有效响应");
            }
        }
    }

    /*PDO相关函数*/
    // ​​同步传输​​：等待 SYNC 信号后才发送/执行（同步驱动）。
    /*在复杂系统中（如多关节机器人），多个设备需要 ​​严格同步​​ 执行指令。
    ​​示例​​：
        主站发送 PDO1（关节1目标位置）
        主站发送 PDO2（关节2目标位置）
        主站发送 ​​SYNC​​ → 所有关节 ​​同时执行​​ 运动指令
    */
    void send_sync()
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = 0x080;
        frame.can_dlc = 0;
        write(sockfd, &frame, sizeof(frame));
    }

    // 删除指定节点的PDO映射配置，每个驱动器pdo有四个通道，0.1.2.3
    void del_pdo(uint8_t node_id, uint8_t pdo_id)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id;
        uint8_t channel_id = pdo_id - 1;

        // 清除RPDO映射参数，0x1600，00，01，02，03
        uint8_t data7[] = {0x2F, channel_id, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("指令发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("未收到有效响应");
        }
        // 清除TPDO映射参数，0x1A00，00，01，02，03
        uint8_t data8[] = {0x2F, channel_id, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data8, sizeof(data8)))
        {
            throw std::runtime_error("指令发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("未收到有效响应");
        }
    }

    // 使用NMT复位所有节点到初始状态（例如设备上电时的默认状态），然后节点自动进入预操作状态，nmt是广播无回馈。
    void prepare_pdo()
    {
        uint8_t data6[] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data6, sizeof(data6)))
        {
            throw std::runtime_error("PDO复位节点失败");
        }
    }

    // 开始远程节点，进入操作状态，nmt是广播无回馈。
    void pdo_set_up()
    {
        uint8_t data5[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data5, sizeof(data5)))
        {
            throw std::runtime_error("PDO远程节点开启失败");
        }
    }

    // TPDO配置，电机端
    void make_tpdo_bridge(uint8_t node_id, uint8_t tpdo_id, const std::vector<int> &orders)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        uint8_t byte3, byte2, byte1, byte0;
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id;

        int cob_id = 16 * 16 * tpdo_id + 8 * 16 + node_id; // tpdo通道1，0x180+node_id，通道2，0x280+node_id，....
        uint8_t channel_id = tpdo_id - 1;                  // 0x1800,tpdo1通道配置，0x1801,tpdo2通道配置

        /*通信参数*/
        // 去使能TPDO，COB-ID的​​最高位（Bit 31）​​用于控制TPDO的使能状态：​0​​：TPDO启用（默认状态）。​​1​​：TPDO禁用。0x80000000
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data1[] = {0x23, channel_id, 0x18, 0x01, byte3, byte2, 0x00, 0x80};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 设置TPDO的传输类型，表示PDO在接收到对应数量的​​SYNC同步帧​​后触发发送，这里设置为1
        intToFourBytes(transmission_type, byte3, byte2, byte1, byte0);
        uint8_t data2[] = {0x2F, channel_id, 0x18, 0x02, byte3, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 设置抑制禁止时间，（最小发送间隔），这里设置为1，单位100us
        intToFourBytes(InhibitTime, byte3, byte2, byte1, byte0);
        uint8_t data6[] = {0x2B, channel_id, 0x18, 0x03, byte3, byte2, 0, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data6, sizeof(data6)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        /*映射参数*/
        // 清除原有映射
        uint8_t data7[] = {0x2F, channel_id, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 设置映射参数数量
        uint8_t data9[] = {0x2F, channel_id, 0x1A, 0x00, sizeof(orders), 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data9, sizeof(data9)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 动态添加映射参数
        uint8_t data3[] = {0x23, channel_id, 0x1A, 0x01, 0x00, 0x00, 0x00, 0x00};
        for (int i = 0; i < orders.size(); i++)
        {
            intToFourBytes(orders[i], byte3, byte2, byte1, byte0);
            data3[3] = i + 1;
            data3[4] = byte3;
            data3[5] = byte2;
            data3[6] = byte1;
            data3[7] = byte0;
            if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
            {
                throw std::runtime_error("CAN信号发送失败");
            }
        }

        // 重新启用TPDO（清除COB - ID最高位）
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {0x23, channel_id, 0x18, 0x01, byte3, byte2, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }
    }

    // RPDO配置，上位机
    void make_rpdo_bridge(uint8_t node_id, uint8_t tpdo_id, const std::vector<int> &orders)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        uint8_t byte3, byte2, byte1, byte0;
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id;

        int cob_id = 16 * 16 * (tpdo_id + 1) + node_id; // rpdo通道1，0x200+node_id，通道2，0x300+node_id，....
        uint8_t channel_id = tpdo_id - 1;               // 0x1400,tpdo1通道配置，0x1401,tpdo2通道配置

        /*配置RPDO通信参数*/
        // 禁止RPDO， COB-ID 的最高位（Bit 31）置 1，禁用 RPDO
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data1[] = {0x23, channel_id, 0x14, 0x01, byte3, byte2, 0x00, 0x80};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 设置RPDO的传输类型
        intToFourBytes(transmission_type, byte3, byte2, byte1, byte0);
        uint8_t data2[] = {0x2F, channel_id, 0x14, 0x02, byte3, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        /*配置RPDO通信参数*/
        // 清除原有映射
        uint8_t data7[] = {0x2F, channel_id, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }
        // 设置映射数量
        uint8_t data9[] = {0x2F, channel_id, 0x16, 0x00, sizeof(orders), 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data9, sizeof(data9)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }

        // 动态添加新映射​
        uint8_t data3[] = {0x23, channel_id, 0x16, 0x00 /*子索引*/, 0x00, 0x00, 0x00, 0x00};
        for (int i = 0; i < orders.size(); i++)
        {
            intToFourBytes(orders[i], byte3, byte2, byte1, byte0);
            data3[3] = i + 1;
            data3[4] = byte3;
            data3[5] = byte2;
            data3[6] = byte1;
            data3[7] = byte0;
            if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
            {
                throw std::runtime_error("CAN信号发送失败");
            }
        }

        // 重新启用RPDO（清除COB - ID最高位）
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {0x23, channel_id, 0x14, 0x01, byte3, byte2, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("CAN信号发送失败");
        }
    }

    // PDO控制电机运动到指定位置
    void pdo_move(int pdoRequestId, uint32_t position)
    {
        // 下使能
        uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(pdoRequestId, data, sizeof(data)))
        {
            throw std::runtime_error("下使能失败");
        }

        // 上使能
        uint8_t data3[] = {0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(pdoRequestId, data3, sizeof(data3)))
        {
            throw std::runtime_error("上使能失败");
        }
        // 命令触发
        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {byte3, byte2, byte1, byte0, 0x1f, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(pdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("目标位置发送失败");
        }
    }

    /*SDO相关函数*/
    // SDO位置控制模型，node_id为节点ID，position为目标位置，钛虎的sdo可以直接6040改变状态，不用0x000使用NMT开启节点
    void position_move_model(int node_id, int position)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        uint32_t sdoRequestId = node_id;          // SDO请求ID（根据实际协议可能需要调整）
        uint32_t sdoResponseId = node_id - 0x080; // SDO响应ID（假设标准从站响应ID为0x580+节点ID）

        // 最好按照顺序
        //  下使能
        uint8_t data1[] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("启动指令发送失败");
        }
        // 上使能
        uint8_t data2[] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("启动指令发送失败");
        }
        // 目标位置设置
        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position, byte3, byte2, byte1, byte0);
        uint8_t data[] = {0x23, 0x7A, 0x60, 0x00, byte3, byte2, byte1, byte0};
        if (!sendFrameWithRetry(sdoRequestId, data, sizeof(data)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        // 命令触发
        uint8_t data3[] = {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
        {
            throw std::runtime_error("启动指令发送失败");
        }
    }

    // 监听位置SDO，返回位置值
    int32_t listen_position_SDO(int node_id)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        uint32_t sdoRequestId = node_id;
        uint32_t sdoResponseId = node_id - 0x080;

        uint8_t data1[] = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("位置读取请求发送失败");
        }
        if (!receiveFrame(frame))
        {
            throw std::runtime_error("启动指令未收到有效响应");
        }
        return FourBytesToint(frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    }

    // 析构函数
    ~CANBus()
    {
        if (sockfd >= 0)
            close(sockfd);
        std::cout << "CAN总线各项参数配置完成" << std::endl;
    }
};

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "robot_can_init");
    std::string name = "can0";
    int mode = 1;                               // 轮廓位置模式
    int acc = 500, de_acc = 500, velo = 500; // 速度参数
    // 从终端命令行获取两个加数
    if (argc != 6)
    {
        ROS_INFO("default init");
    }
    else
    {
        ROS_INFO("Param init");
        name.assign(argv[1]);
        mode = std::stoi(argv[2]);
        acc = std::stoi(argv[3]);
        de_acc = std::stoi(argv[4]);
        velo = std::stoi(argv[5]);
    }
    CANBus can(name.c_str());                 // 实例对象
    can.init(mode);                           // 初始化设置控制模式
    can.set_velo_acc_dacc(acc, de_acc, velo); // 设置速度、加速度

    // 清除原有映射参数，后面复位其实也会清除，这里都先留着吧
    for (int i = 1; i <= 7; i++)
    {
        for (int j = 1; j <= 4; j++)
            can.del_pdo(i, j);
    }
    // 复位节点，复位后会清除所有通信参数和映射参数，进入预操作状态
    can.prepare_pdo();

    // 设置PDO映射参数
    std::vector<int> TPDO1_index = {0x60640020, 0x606C0020}; // TPDO1，实际位置0x6064，实际速度0x606C
    std::vector<int> TPDO2_index = {0x60770010};             // TPDO2，实际扭矩0x6077
    std::vector<int> RPDO1_index = {0x607A0020, 0x60400010}; // RPDO1，目标位置0x607A、控制字0x6040，6-7-15
    for (int i = 1; i <= 7; i++)
    {
        can.make_tpdo_bridge(i, 1, TPDO1_index); // 通道1，反馈位置速度
        can.make_tpdo_bridge(i, 2, TPDO2_index); // 通道2，反馈扭矩
        can.make_rpdo_bridge(i, 1, RPDO1_index); // 通道1，发送位置
    }

    // 开启远程节点，进入操作状态
    can.pdo_set_up();

    /*测试代码*/
    /*PDO控制电机*/
    /*for (int i = 1; i <= 7; i++)
    {
        can.pdo_move(0x200 + i,0);
    }
    can.send_sync(); // 同步触发，即使不发这个电机也可以运动。sync主要是同步一次tpdo反馈信号，即电机反馈信号。
    */
   
    /*SDO控制电机，读位置延迟较重*/
    /*
    int node_id = 0x601;                               // 假设节点ID为0x601
    int target_position =0;                           // 目标位置为5000000
    can.position_move_model(node_id, target_position); // 设置目标位置为5000000
    while (true)
    {
        if (abs(can.listen_position_SDO(node_id) - target_position) > 50)
        {
            std::cout << "当前位置: " << can.listen_position_SDO(node_id) << "，距离目标位置: "
                      << (can.listen_position_SDO(node_id) - target_position) << "，继续调整..." << std::endl;
            usleep(100000); // 等待100ms
            continue;
        }
        else
        {
            std::cout << "已到达目标位置，当前位置: " << can.listen_position_SDO(node_id) << std::endl;
            break; // 如果当前位置接近目标位置则退出循环
        }

    }
    */

    return 0;
}
