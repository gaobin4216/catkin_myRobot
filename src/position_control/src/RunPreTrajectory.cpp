#include <ros/ros.h>
#include <position_control/position.h>

//用来把pulsesPositions数组里存储的各轨迹点的位置信息依次发送给电机，
//该cpp文件也读取了当前电机位置信息，用来判断何时发送下一个轨迹点数据，但在实际控制中是不用这样的
namespace position
{
    //类成员初始化
    PositionControl::PositionControl(ros::NodeHandle& nh) : nh_(nh), pub_(nh_.advertise<can_msgs::Frame>("sent_messages", 100)) {}
    //从共享内存中读取脉冲位置
    void PositionControl::readPulsesPositionsFromSharedMemory() 
    {
        // 打开共享内存段
        bip::managed_shared_memory segment(bip::open_only, "MySharedMemory");

        // 获取共享内存中的向量
        VectorOfVectors* pulsesPositions_shmem = segment.find<VectorOfVectors>("PulsesPositions").first;

        if (pulsesPositions_shmem) {
            // 将数据从共享内存向量复制到标准向量
            pulsesPositions.clear();
            for (const auto& inner_shmem_vec : *pulsesPositions_shmem) 
            {
                std::vector<int> inner_vec(inner_shmem_vec.begin(), inner_shmem_vec.end());
                pulsesPositions.push_back(inner_vec);
            }

            /*if (pulsesPositions.empty() || pulsesPositions[0].size() < 14) 
            {
                ROS_WARN("Each trajectory point must have at least 14 pulse values.");
            } 
            else 
            {
                ROS_INFO("Pulses positions are valid.");
            }*/
        } 
        else 
        {
            ROS_ERROR("Failed to find pulses positions in shared memory.");
        }
    };
    //启动canopen从设备
    void PositionControl::SlaveEnable()
    {
        ROS_INFO("Sending Slave Enable Message");
        can_msgs::Frame can_frame_msg;
        can_frame_msg.id =  0x000;
        can_frame_msg.dlc = CAN_DLC;
        can_frame_msg.data= boost::array<uint8_t, 8>{0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg);//将 can_frame_msg 消息发布到 "sent_messages" 话题。
        ros::Duration(0.5).sleep();
        jointposition_mode();//选择位置模式
    };
   // 选择位置模式
   void PositionControl::jointposition_mode()
   {
       can_msgs::Frame can_frame_msg1;
       can_frame_msg1.id =  CAN_ID_BASE+MY_ID;
       can_frame_msg1.dlc = CAN_DLC;
       can_frame_msg1.data= boost::array<uint8_t, 8>{0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
       ros::Duration(0.5).sleep();
       pub_.publish(can_frame_msg1);
       ros::Duration(0.5).sleep();//确保信息有时间发送
       jointenable1();
   };
    //清除错误
    void PositionControl::jointenable1()
    {
        can_msgs::Frame can_frame_msg2;
        can_frame_msg2.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg2.dlc = CAN_DLC;
        can_frame_msg2.data= boost::array<uint8_t, 8>{0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg2);
        ros::Duration(0.5).sleep();
        jointenable2();//下使能
    };
    //下使能
    void PositionControl::jointenable2()
    {
        can_msgs::Frame can_frame_msg3;
        can_frame_msg3.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg3.dlc = CAN_DLC;
        can_frame_msg3.data= boost::array<uint8_t, 8>{0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg3);
        jointenable3();//使能
    };
    //使能
    void PositionControl::jointenable3()
    {
        can_msgs::Frame can_frame_msg4;
        can_frame_msg4.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg4.dlc = CAN_DLC;
        can_frame_msg4.data= boost::array<uint8_t, 8>{0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg4);
        velocityadd();//加速度设置
    };
    //加速度设置
    void PositionControl::velocityadd()
    {
        can_msgs::Frame can_frame_msg5;
        can_frame_msg5.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg5.dlc = CAN_DLC;
        can_frame_msg5.data= boost::array<uint8_t, 8>{0x23, 0x83, 0x60, 0x00, 0xBE, 0x15, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg5);
        velocitydiff();//减速度设置
    };
    //减速度设置
    void PositionControl::velocitydiff()
    {
        can_msgs::Frame can_frame_msg6;
        can_frame_msg6.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg6.dlc = CAN_DLC;
        can_frame_msg6.data= boost::array<uint8_t, 8>{0x23, 0x84, 0x60, 0x00, 0xBE, 0x15, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg6);
        velocityset();//最大轮廓速度设置
    };
    //最大轮廓速度设置
    void PositionControl::velocityset()
    {
        can_msgs::Frame can_frame_msg7;
        can_frame_msg7.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg7.dlc = CAN_DLC;
        can_frame_msg7.data= boost::array<uint8_t, 8>{0x23, 0x81, 0x60, 0x00, 0xBE, 0x15, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg7);
        jointstart1();//关节上使能
        
    };
    //关节上使能
    void PositionControl::jointstart1()
    {
        can_msgs::Frame can_frame_msg9;
        can_frame_msg9.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg9.dlc = CAN_DLC;
        can_frame_msg9.data= boost::array<uint8_t, 8>{0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg9);
        positionSet(pulsesPositions);//发送位置指令
        
    };
     //将整数转换为2个字节的十六进制数
    void PositionControl::intToFourBytes(int32_t value, int32_t& byte3, int32_t& byte2, int32_t& byte1, int32_t& byte0) 
     {
         // 高8位
         byte0 = static_cast<int32_t>((value >> 24) & 0xFF);
         // 低8位
         byte1 = static_cast<int32_t>(value >> 16& 0xFF);
 
         byte2 = static_cast<int32_t>(value >> 8& 0xFF);
 
         byte3 = static_cast<int32_t>(value & 0xFF);
     };
    //位置设置
    void PositionControl::positionSet(const std::vector<std::vector<int>>& pulses_positions)
    {
        can_msgs::Frame can_frame_msg8;
        can_frame_msg8.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg8.dlc = CAN_DLC;
        can_frame_msg8.data= boost::array<uint8_t, 8>{0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

        // 提取每个轨迹点的脉冲数并转换为两位十六进制数，65535为0xFFFF
        int32_t pulse = pulses_positions[currentPointIndex][0]; // 索引从0开始）
        int32_t byte3, byte2, byte1, byte0;
        intToFourBytes(pulse, byte3, byte2, byte1, byte0);

        // 填充到一维数组的后四位
        can_frame_msg8.data[4] = byte3;
        can_frame_msg8.data[5] = byte2;
        can_frame_msg8.data[6] = byte1;
        can_frame_msg8.data[7] = byte0;

        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg8);

        // 打印发送的数据
        ROS_INFO("Sending CAN Frame Data for Point %zu:", currentPointIndex);
        for (const auto& byte : can_frame_msg8.data) {
            ROS_INFO("  0x%02X", byte);
        }
        
        jointstart2();
    }
    //对于轮廓位置模式，这个设置是必要的，然而对于轮廓速度模式则不需要,设置控制指令，由于控制命令为上升沿触发，要将控制字的bit4切换为off再切到on，伺服才会再次运动
    void PositionControl::jointstart2()
    {
        can_msgs::Frame can_frame_msg11;
        can_frame_msg11.id =  CAN_ID_BASE+MY_ID;
        can_frame_msg11.dlc = CAN_DLC;
        can_frame_msg11.data= boost::array<uint8_t, 8>{0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00};
        ros::Duration(0.5).sleep();
        pub_.publish(can_frame_msg11);
    };

     //接受反馈数据，判断是否更新位置点
     void PositionControl::feedbackCallback(const can_msgs::Frame::ConstPtr& msg)    
     {
         if (msg->id == FEEDBACK_ID && msg->data[1]== 0x64 && msg->data[2]== 0x60)
         {
             // 解析反馈信息，反馈信息表示当前脉冲数
             int32_t current_pulse = (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | msg->data[4];
             ROS_INFO("Current pulse: %d", current_pulse);
             // 这里目标脉冲数为当前轨迹点的第1个脉冲数
             int32_t target_pulse = pulsesPositions[currentPointIndex][0];
             ROS_INFO("Target pulse: %d", target_pulse);
             if (std::abs(current_pulse - target_pulse) <= 500)
             {
                 isAtTarget = true;//这个变量没想好怎么用
                 ROS_INFO("Motor has reached target position for point %zu.", currentPointIndex);
                 // 如果电机到达目标位置，发送下一个轨迹点
                 if (currentPointIndex +1 < pulsesPositions.size())
                 {
                     currentPointIndex++;
                     isAtTarget = false;
                     positionSet(pulsesPositions);
                 } 
                 //不添加这个else函数直接报错退出了
                 else
                 {
                     ROS_INFO("All trajectories completed.");
                 }
             }
             else
             {
                 ROS_INFO("Motor is still moving to target position for point %zu.", currentPointIndex);
             }
         }
     };
};
int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "RunPreTrajectory");
    ros::NodeHandle nh;

    // 创建 PositionControl 对象
    position::PositionControl positionControl(nh);

    // 创建发布者和订阅者
    //发送控制指令（如电机速度、位置指令）到 CAN 总线设备
    ros::Publisher pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);//发送 can_msgs::Frame 类型的消息到话题 "sent_messages"，队列长度为 100
    
    //用于接收 CAN 总线设备的反馈数据（如编码器位置、电机状态），并通过 feedbackCallback 更新控制逻辑
    ros::Subscriber sub = nh.subscribe("received_messages", 100,  &position::PositionControl::feedbackCallback, &positionControl);

    // 从共享内存中读取脉冲位置
    positionControl.readPulsesPositionsFromSharedMemory();

    //启动can设备
    positionControl.SlaveEnable();

    // 创建 CAN 消息
    can_msgs::Frame can_frame_msg10;
    can_frame_msg10.id = CAN_ID_BASE+MY_ID;
    can_frame_msg10.dlc = CAN_DLC;
    can_frame_msg10.data = boost::array<uint8_t, 8>{0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 设置发布频率
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        pub.publish(can_frame_msg10);
        loop_rate.sleep();
        ros::spinOnce();  // 处理订阅的消息
    }
    return 0;
}
