#include "ros/ros.h"
#include "driver_control/target_position.h"
#include "driver_control/target_answer.h"
#include <cstdlib>
#include <ctime>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <vector>
#include <map>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#define UNITS 7

/*std::vector<double> current_joints = {0, 0, 0, 0, 0, 0, 0};
void arrayCallback1(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < UNITS; i++)
    {
        current_joints[i] = msg->position[i];
    }
}*/

class Client
{
private:
    ros::Publisher pub_;

public:
    std::vector<double> target = {0, 0, 0, 0, 0, 0, 0};
    Client(ros::NodeHandle &nh)
    {
        pub_ = nh.advertise<driver_control::target_position>("target_array", 10);
        std::srand(std::time(0));
    }

    void write(int index, double value)
    {
        target[index] = value;
    }

    void generateAndSendData()
    {
        driver_control::target_position msg;
        for (int i = 0; i < 7; i++)
        {
            msg.data[i] = target[i];
        }
        pub_.publish(msg);
        ROS_INFO("Sent new 7-element double array");
    }

    void run_with_rate()
    {
        ros::Rate loop_rate(1); // 1Hz

        while (ros::ok())
        {
            if (pub_.getNumSubscribers() > 0)
            {
                generateAndSendData();
            }
            else
            {
                ROS_DEBUG("Waiting for subscribers...");
            }
            loop_rate.sleep();
        }
    }

    void run()
    {
        ros::Rate loop_rate(1); // 1Hz

        while (ros::ok())
        {
            if (pub_.getNumSubscribers() > 0)
            {
                generateAndSendData();
                break;
            }
            else
            {
                ROS_DEBUG("Waiting for subscribers...");
            }
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_extractor_node");
    ros::NodeHandle nh;
    if (argc != 8)
    {
        ROS_INFO("EROOR");
    }
    else
    {
        ROS_INFO("VAILD DATA");
        // 设置规划组
        static const std::string PLANNING_GROUP = "manipulator"; // 修改为你的规划组名称
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        /*std::vector<double> joint_target = {0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < UNITS; i++)
        {
            joint_target[i] = std::stod(argv[i + 1]) * M_PI / 180;
        }
        // 设置目标位置
        move_group.setJointValueTarget(joint_target);
        for (size_t i = 0; i < joint_target.size(); ++i)
        {
            ROS_INFO("joint_target %zu: %.4f rad", i, joint_target[i]);
        }
        */

        geometry_msgs::Pose target_pose;
        target_pose.position.x = std::stod(argv[1]) / 1000;
        target_pose.position.y = std::stod(argv[2]) / 1000;
        target_pose.position.z = std::stod(argv[3]) / 1000;
        target_pose.orientation.x = std::stod(argv[4]);
        target_pose.orientation.y = std::stod(argv[5]);
        target_pose.orientation.z = std::stod(argv[6]);
        target_pose.orientation.w = std::stod(argv[7]);
        move_group.setGoalPositionTolerance(0.001);  // 位置容差(米)
        move_group.setGoalOrientationTolerance(0.1); // 角度容差(弧度)
        move_group.setPoseTarget(target_pose);

        ros::AsyncSpinner spinner(1);
        spinner.start();
        // 规划路径
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            ROS_ERROR("Planning failed!");
            return -1;
        }

        

        // 存储轨迹点的容器 [point_index][joint_index]
        // std::vector<std::vector<double>> full_trajectory;

        // 按关节名称索引的轨迹点
        // std::map<std::string, std::vector<double>> joint_angle_map;

        Client client0(nh);
        const moveit_msgs::RobotTrajectory &trajectory = my_plan.trajectory_;
        // 收集所有轨迹点
        for (size_t point_idx = 0; point_idx < trajectory.joint_trajectory.points.size(); ++point_idx)
        {
            const trajectory_msgs::JointTrajectoryPoint &point =
                trajectory.joint_trajectory.points[point_idx];

            // std::vector<double> point_joint_angles;

            // 按顺序收集关节角度
            for (size_t joint_idx = 0; joint_idx < point.positions.size(); ++joint_idx)
            {
                const std::string &joint_name =
                    trajectory.joint_trajectory.joint_names[joint_idx];

                double angle = point.positions[joint_idx];
                // point_joint_angles.push_back(angle);

                // 按关节名称存储轨迹历史
                // joint_angle_map[joint_name].push_back(angle);

                client0.write(int(joint_idx), angle / M_PI * 180);
                std::cout << angle / M_PI * 180 << "  ";
            }
            std::cout << std::endl;
            client0.run();
        }
    }

    return 0;
}