#include "ros/ros.h"
#include "driver_control/target_position.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>

#define UNITS 7 // 机械臂关节数量

class Client
{
private:
    ros::Publisher pub_;         // ROS发布器，用于发送目标位置
    std::vector<double> target_; // 存储目标关节角度（度）

public:
    Client(ros::NodeHandle &nh) : target_(UNITS, 0.0)
    {
        // 初始化发布器，主题为"target_array"，队列大小为10
        pub_ = nh.advertise<driver_control::target_position>("target_array", 10);
    }

    // 设置目标关节角度
    void write(int index, double value)
    {
        if (index >= 0 && index < UNITS)
        {
            target_[index] = value;
        }
    }

    // 生成并发送目标位置消息
    void generateAndSendData()
    {
        driver_control::target_position msg;

        // 填充消息数据
        for (int i = 0; i < UNITS; i++)
        {
            msg.data[i] = target_[i];
        }

        // 发布消息
        pub_.publish(msg);

        // 打印发送的关节角度
        std::cout << "发送关节角度: [";
        for (int i = 0; i < UNITS; i++)
        {
            std::cout << target_[i];
            if (i != UNITS - 1)
                std::cout << ", ";
        }
        std::cout << "] (度)" << std::endl;
    }

    // 等待订阅者连接
    void waitForSubscriber()
    {
        ros::Rate rate(1); // 1Hz频率
        while (ros::ok() && pub_.getNumSubscribers() == 0)
        {
            std::cout << "等待订阅者连接..." << std::endl;
            rate.sleep();
        }
    }
};

// 全局变量，存储接收到的关节轨迹
trajectory_msgs::JointTrajectory g_joint_trajectory;
bool g_newTrajectoryReceived = false; // 标记是否收到新轨迹

//执行目标回调函数，处理从RViz接收到的执行命令

void executionGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg, Client &client)
{
    // 检查轨迹点是否为空
    if (!msg->goal.trajectory.joint_trajectory.points.empty())
    {
        g_joint_trajectory = msg->goal.trajectory.joint_trajectory;
        g_newTrajectoryReceived = true;
        std::cout << "状态: 接收到执行命令，开始处理轨迹" << std::endl;

        // 遍历所有轨迹点
        for (const auto &point : g_joint_trajectory.points)
        {
            // 转换每个关节的角度（弧度->度）并设置
            for (size_t i = 0; i < point.positions.size(); i++)
            {
                double angle_deg = point.positions[i] * 180.0 / M_PI;
                client.write(i, angle_deg);
            }
            // 发送当前轨迹点的关节角度
            client.generateAndSendData();
        }

        g_newTrajectoryReceived = false;
        std::cout << "状态: 轨迹处理完成" << std::endl;
    }
}

// 根据命令行参数规划并执行轨迹
void planAndExecuteFromCommandLine(int argc, char **argv, Client &client)
{
    // 检查参数数量是否正确
    if (argc != UNITS + 1)
    {
        std::cerr << "错误: 参数数量不正确! 用法: " << argv[0]
                  << " <关节1> <关节2> ... <关节7> (度)" << std::endl;
        return;
    }

    // 初始化MoveIt规划组
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // 设置目标关节角度（转换为弧度）
    std::vector<double> joint_target(UNITS);
    for (int i = 0; i < UNITS; i++)
    {
        try
        {
            joint_target[i] = std::stod(argv[i + 1]) * M_PI / 180.0;
            std::cout << "关节 " << (i + 1) << " 目标值: " << argv[i + 1] << " 度"
                      << " (" << joint_target[i] << " 弧度)" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "错误: 第" << (i + 1) << "个参数不是有效的数字" << std::endl;
            return;
        }
    }
    move_group.setJointValueTarget(joint_target);

    // 进行路径规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        std::cerr << "错误: 规划失败!" << std::endl;
        return;
    }

    std::cout << "规划成功，开始发送轨迹点..." << std::endl;

    // 获取规划后的轨迹
    const moveit_msgs::RobotTrajectory &trajectory = my_plan.trajectory_;

    // 遍历所有轨迹点并发送
    for (const auto &point : trajectory.joint_trajectory.points)
    {
        // 转换并设置每个关节的角度
        for (size_t i = 0; i < point.positions.size(); i++)
        {
            double angle_deg = point.positions[i] * 180.0 / M_PI;
            client.write(i, angle_deg);
        }
        // 发送当前轨迹点
        client.generateAndSendData();
    }

    std::cout << "状态: 轨迹执行完成" << std::endl;
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "robot_client");
    ros::NodeHandle nh;

    // 启动异步spinner处理回调
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // 初始化MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // 创建客户端并等待订阅者连接
    Client client(nh);
    client.waitForSubscriber();

    // 订阅执行目标主题，绑定回调函数
    ros::Subscriber execution_sub = nh.subscribe<moveit_msgs::ExecuteTrajectoryActionGoal>(
        "/execute_trajectory/goal", 1,
        boost::bind(executionGoalCallback, _1, boost::ref(client)));

    // 根据参数选择运行模式
    if (argc > 1)
    {
        // 命令行参数模式：直接规划并执行
        planAndExecuteFromCommandLine(argc, argv, client);
    }
    else
    {
        // RViz交互模式：等待用户操作
        std::cout << "模式: 运行在RViz交互模式" << std::endl;
        std::cout << "提示: 在RViz中Plan路径后，点击Execute执行" << std::endl;
        ros::waitForShutdown();
    }

    return 0;
}