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
#define SDO_RETRY_COUNT 3           // 单次指令最大重试次数
#define COMMAND_DELAY_US 10000      // 指令间最小间隔10ms

class CANBus
{
private:
    int sockfd;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int transmission_type = 1;
    int InhibitTime = 1;

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

public:
    // 构造函数
    CANBus(const char *interface = "can0")
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

    // 发送 CAN 帧（基础版）
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

    // 优化后的整数转4字节函数（修正符号问题）
    void intToFourBytes(int32_t value, uint8_t &byte3, uint8_t &byte2, uint8_t &byte1, uint8_t &byte0)
    {
        byte0 = (value >> 24) & 0xFF; // 最高位字节
        byte1 = (value >> 16) & 0xFF;
        byte2 = (value >> 8) & 0xFF;
        byte3 = value & 0xFF; // 最低位字节
    }

    int32_t FourBytesToint(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0)
    {
        return (static_cast<int32_t>(byte0) << 24) |
               (static_cast<int32_t>(byte1) << 16) |
               (static_cast<int32_t>(byte2) << 8) |
               static_cast<int32_t>(byte3);
    }

    void init(uint8_t model, int acce_max = 0x10BE, int de_acce_max = 0x10BE, int velo_max = 0x10BE)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        uint8_t byte3, byte2, byte1, byte0;

        // 远程节点关闭（添加错误处理）
        uint8_t data[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data, sizeof(data)))
        {
            throw std::runtime_error("初始化-远程节点关闭失败");
        }

        for (int i = 1; i < 8; i++)
        {
            uint32_t sdoRequestId = 0x600 + i;          // SDO请求ID（根据实际协议可能需要调整）
            uint32_t sdoResponseId = 0x600 + i - 0x080; // SDO响应ID（假设标准从站响应ID为0x580+节点ID）
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

    void set_velo_acc_dacc(int acce_max = 0x10BE, int de_acce_max = 0x10BE, int velo_max = 0x10BE)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
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
                throw std::runtime_error("目标位置设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("目标位置设置未收到有效响应");
            }

            intToFourBytes(de_acce_max, byte3, byte2, byte1, byte0);
            uint8_t data3[] = {0x23, 0x84, 0x60, 0x00, byte3, byte2, byte1, byte0};
            if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
            {
                throw std::runtime_error("目标位置设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("目标位置设置未收到有效响应");
            }

            intToFourBytes(velo_max, byte3, byte2, byte1, byte0);
            uint8_t data4[] = {0x23, 0x81, 0x60, 0x00, byte3, byte2, byte1, byte0};
            if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
            {
                throw std::runtime_error("目标位置设置发送失败");
            }
            if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
            {
                throw std::runtime_error("目标位置设置未收到有效响应");
            }
        }
    }

    void send_sync()
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = 0x080;
        frame.can_dlc = 0;
        write(sockfd, &frame, sizeof(frame));
    }

    void speak_position_with_retry(int node_id, int target_position)
    {
        int32_t initial_positon = listen_position_SDO(node_id);
        int32_t current_position = 0;
        int attempt = 0;

        while (true)
        {
            current_position = listen_position_SDO(0x607);
            if (abs(current_position - target_position) < 20)
            {
                std::cout << "已到达目标位置，当前位置: " << current_position << std::endl;
                break;
            }

            position_move_model(0x607, target_position);
            std::cout << "第" << (attempt + 1) << "次发送位置指令成功" << std::endl;
            usleep(100000); // 等待10ms电机移动

            if (listen_position_SDO(0x607) == initial_positon)
            {
                std::cout << "位置未更新,尝试第" << (attempt + 1) << "次失败" << std::endl;
                attempt++;
            }
            else
            {
                std::cout << "位置更新,尝试第" << (attempt + 1) << "次成功" << std::endl;
                break;
            }
        }
    }

    void position_move_model(int node_id, int position)
    {
        struct can_frame frame;
        uint32_t sdoRequestId = node_id;          // SDO请求ID（根据实际协议可能需要调整）
        uint32_t sdoResponseId = node_id - 0x080; // SDO响应ID（假设标准从站响应ID为0x580+节点ID）

        // 控制字设置（示例，其他指令类似添加错误处理）
        uint8_t data4[] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("控制字设置1发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("控制字设置1未收到有效响应");
        }

        // （中间指令类似添加错误处理...）

        // 目标位置设置
        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position, byte3, byte2, byte1, byte0);
        uint8_t data[] = {0x23, 0x7A, 0x60, 0x00, byte3, byte2, byte1, byte0};
        if (!sendFrameWithRetry(sdoRequestId, data, sizeof(data)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("目标位置设置未收到有效响应");
        }

        // 启动指令
        uint8_t data9[] = {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data9, sizeof(data9)))
        {
            throw std::runtime_error("启动指令发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("启动指令未收到有效响应");
        }
    }

    int32_t listen_position_SDO(int node_id)
    {
        struct can_frame frame;
        uint32_t sdoRequestId = node_id;
        uint32_t sdoResponseId = node_id - 0x080;

        uint8_t data1[] = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("位置读取请求发送失败");
        }
        if (!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId))
        {
            throw std::runtime_error("位置读取未收到有效响应");
        }

        return FourBytesToint(frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    }

    void prepare_pdo(uint8_t node_id)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // 远程节点关闭
        uint8_t data5[] = {0x02, node_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data5, sizeof(data5)))
        {
            throw std::runtime_error("PDO远程节点关闭失败");
        }

        // 复位节点
        uint8_t data6[] = {0x82, node_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data6, sizeof(data6)))
        {
            throw std::runtime_error("PDO复位节点失败");
        }
        receiveFrame(frame);
    }

    void make_tpdo_bridge(uint8_t node_id, uint8_t tpdo_id, const std::vector<int> &orders)
    {
        int cob_id = 16 * 16 * tpdo_id + 8 * 16 + node_id;
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id; // SDO请求ID（根据实际协议可能需要调整）
        uint8_t byte3, byte2, byte1, byte0;
        uint8_t channel_id = tpdo_id - 1;
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // 去使能TPDO1
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data1[] = {0x23, channel_id, 0x18, 0x01, byte3, byte2, 0x00, 0x80};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        // 设置TPDO1的传输类型
        intToFourBytes(transmission_type, byte3, byte2, byte1, byte0);
        uint8_t data2[] = {0x2F, channel_id, 0x18, 0x02, byte3, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        intToFourBytes(InhibitTime, byte3, byte2, byte1, byte0);
        uint8_t data6[] = {0x2B, channel_id, 0x18, 0x03, byte3, byte2, 0, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data6, sizeof(data6)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
        /*if(!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId)) {
            throw std::runtime_error("目标位置设置未收到有效响应");
        }*/

        // 清除原有映射
        uint8_t data7[] = {0x2F, channel_id, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        uint8_t data3[] = {0x23, channel_id, 0x1A, 0x01, 0, 0, 0, 0};
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
                throw std::runtime_error("目标位置设置发送失败");
            }
        }

        /* set param 1
        intToFourBytes(0x60410010, byte3, byte2, byte1, byte0);
        uint8_t data3[] = {0x23, channel_id, 0x1A, 0x01, byte3, byte2, byte1, byte0};
        if(!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3))) {
            throw std::runtime_error("目标位置设置发送失败");
        }

        //set param 2
        intToFourBytes(0x60640020, byte3, byte2, byte1, byte0);
        uint8_t data8[] = {0x23, channel_id, 0x1A, 0x02, byte3, byte2, byte1, byte0};
        if(!sendFrameWithRetry(sdoRequestId, data8, sizeof(data8))) {
            throw std::runtime_error("目标位置设置发送失败");
        }*/

        // set the num of param
        uint8_t data9[] = {0x2F, channel_id, 0x1A, 0x00, 2, 0, 0, 0};
        if (!sendFrameWithRetry(sdoRequestId, data9, sizeof(data9)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {0x23, channel_id, 0x18, 0x01, byte3, byte2, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
    }

    void make_rpdo_bridge(uint8_t node_id, uint8_t tpdo_id, const std::vector<int> &orders)
    {
        int cob_id = 16 * 16 * (tpdo_id + 1) + node_id;
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id; // SDO请求ID（根据实际协议可能需要调整）
        uint8_t byte3, byte2, byte1, byte0;
        uint8_t channel_id = tpdo_id - 1;
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // 去使能TPDO1
        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data1[] = {0x23, channel_id, 0x14, 0x01, byte3, byte2, 0x00, 0x80};
        if (!sendFrameWithRetry(sdoRequestId, data1, sizeof(data1)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        // 设置TPDO1的传输类型
        // intToFourBytes(transmission_type, byte3, byte2, byte1, byte0);
        uint8_t data2[] = {0x2F, channel_id, 0x14, 0x02, 1, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        /*intToFourBytes(InhibitTime, byte3, byte2, byte1, byte0);
        uint8_t data6[] = {0x2B, channel_id, 0x14, 0x02, byte3, byte2, 0, 0x00};
        if(!sendFrameWithRetry(sdoRequestId, data6, sizeof(data6))) {
            throw std::runtime_error("目标位置设置发送失败");
        }*/

        /*if(!receiveFrame(frame) || !validateResponseFrame(frame, sdoResponseId)) {
            throw std::runtime_error("目标位置设置未收到有效响应");
        }*/

        // 清除原有映射
        uint8_t data7[] = {0x2F, channel_id, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        uint8_t data3[] = {0x23, channel_id, 0x16, 0x01, 0, 0, 0, 0};
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
                throw std::runtime_error("目标位置设置发送失败");
            }
        }

        /*set param 2
        intToFourBytes(0x607A0020, byte3, byte2, byte1, byte0);
        uint8_t data8[] = {0x23, channel_id, 0x16, 0x02, byte3, byte2, byte1, byte0};
        if(!sendFrameWithRetry(sdoRequestId, data8, sizeof(data8))) {
            throw std::runtime_error("目标位置设置发送失败");
        }*/

        // set the num of param
        uint8_t data9[] = {0x2F, channel_id, 0x16, 0x00, sizeof(orders), 0, 0, 0};
        if (!sendFrameWithRetry(sdoRequestId, data9, sizeof(data9)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        intToFourBytes(cob_id, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {0x23, channel_id, 0x14, 0x01, byte3, byte2, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
    }

    void pdo_set_up(uint8_t node_id)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // 远程节点关闭
        uint8_t data5[] = {0x01, node_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(0x000, data5, sizeof(data5)))
        {
            throw std::runtime_error("PDO远程节点关闭失败");
        }
    }

    void del_pdo(uint8_t node_id, uint8_t tpdo_id)
    {
        uint8_t channel_id = tpdo_id - 1;
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id;
        uint8_t data7[] = {0x2F, channel_id, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }

        uint8_t data8[] = {0x2F, channel_id, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data8, sizeof(data8)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
    }

    void set_bias(uint8_t node_id, int32_t bias)
    {
        uint32_t sdoRequestId = 0x600 + node_id;
        uint32_t sdoResponseId = 0x580 + node_id;
        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(bias, byte3, byte2, byte1, byte0);
        uint8_t data7[] = {0x23, 0x08, 0x20, 0x00, byte3, byte2, byte1, byte0};
        if (!sendFrameWithRetry(sdoRequestId, data7, sizeof(data7)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
    }

    void pdo_move(int sdoRequestId, uint32_t position)
    {
        if (position < 100)
            position = 100;
        send_sync();
        uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data, sizeof(data)))
        {
            throw std::runtime_error("初始化-远程节点关闭失败");
        }
        send_sync();

        uint8_t data2[] = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data2, sizeof(data2)))
        {
            throw std::runtime_error("初始化-远程节点关闭失败");
        }
        send_sync();

        uint8_t data3[] = {0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00};
        if (!sendFrameWithRetry(sdoRequestId, data3, sizeof(data3)))
        {
            throw std::runtime_error("初始化-远程节点关闭失败");
        }
        send_sync();

        uint8_t byte3, byte2, byte1, byte0;
        intToFourBytes(position, byte3, byte2, byte1, byte0);
        uint8_t data4[] = {byte3, byte2, byte1, byte0, 0x1f, 0, 0, 0};
        if (!sendFrameWithRetry(sdoRequestId, data4, sizeof(data4)))
        {
            throw std::runtime_error("目标位置设置发送失败");
        }
        send_sync();
    }

    ~CANBus()
    {
        if (sockfd >= 0)
            close(sockfd);
        std::cout << "CAN总线关闭" << std::endl;
    }
};

int main(int argc, char **argv)
{
    // ROS节点初始化
    std::string name = "can0";
    int mode = 1;
    int acc = 1000;
    int de_acc = 1000;
    int velo = 500;
    ros::init(argc, argv, "robot_can_init_node");
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
        std::cout << acc << std::endl;
    }
    CANBus can(name.c_str());
    can.init(mode);
    can.set_velo_acc_dacc(acc, de_acc, velo);
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 4; j++)
            can.del_pdo(i, j);
    }
    //实际位置、实际速度TPDO1
    std::vector<int> orders_index1 = {0x60640020, 0x606C0020};
    //目标位置、控制子RPDO1
    std::vector<int> orders_index2 = {0x607A0020, 0x60400010};
    //实际电流、位置环PID TPDO2
    std::vector<int> orders_index3 = {0x60770010};
    //轮廓加减加速度 TPDO3
    std::vector<int> orders_index4 = {0x60830020, 0x60840020};
    int i;
    for (i = 0; i < 7; i++)
    {
        can.prepare_pdo(i + 1);
        can.make_tpdo_bridge(i + 1, 1, orders_index1);
        can.make_rpdo_bridge(i + 1, 1, orders_index2);
        can.make_tpdo_bridge(i + 1, 2, orders_index3);
        // can.make_tpdo_bridge(i + 1, 3, orders_index4);
        can.pdo_set_up(i + 1);
    }

    return 0;
}
