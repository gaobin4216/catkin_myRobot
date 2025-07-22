#include "ros/ros.h"
#include <manipulator_control/target_position.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>



#define UNITS 7 // 机械臂关节数量

class LinearMovementGenerator
{
private:
    ros::Publisher pub_;
    std::vector<double> target_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;

public:
    LinearMovementGenerator() : move_group_("manipulator"),
                                target_(UNITS, 0.0)
    {
        pub_ = nh_.advertise<manipulator_control::target_position>("target_array", 10);

        // 设置MoveGroup参数
        move_group_.setPlanningTime(5.0);
        move_group_.setNumPlanningAttempts(3);
        move_group_.setMaxVelocityScalingFactor(0.3);
        move_group_.setMaxAccelerationScalingFactor(0.3);
    }

    void write(int index, double value)
    {
        if (index >= 0 && index < UNITS)
            target_[index] = value;
    }

    void sendTarget()
    {
        manipulator_control::target_position msg;
        for (int i = 0; i < UNITS; i++)
            msg.data[i] = target_[i];
        pub_.publish(msg);

        ROS_INFO_STREAM("发送关节角度: " << target_[0] << ", " << target_[1] << ", "
                                         << target_[2] << ", " << target_[3] << ", "
                                         << target_[4] << ", " << target_[5] << ", "
                                         << target_[6]);
    }

    void waitForSubscriber()
    {
        while (ros::ok() && pub_.getNumSubscribers() == 0)
        {
            ROS_INFO("等待订阅者连接...");
            ros::Duration(1.0).sleep();
        }
    }

    void moveLinearXY()
    {
        // 获取当前末端位置
        // 检查所有相关组件
        ROS_INFO("=== MoveIt 状态诊断 ===");
        ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
        ROS_INFO("Planning frame: %s", move_group_.getPlanningFrame().c_str());

        // 试获取关节状态
        sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1.0));
        if (state)
        {
            ROS_INFO("Received joint state with %lu joints", state->position.size());
        }
        else
        {
            ROS_ERROR("No joint state received!");
        }


        // 3. 强制更新 MoveIt 状态并获取位姿
        move_group_.setStartStateToCurrentState(); // 强制同步最新状态
        geometry_msgs::Pose start_pose = move_group_.getCurrentPose().pose;
        ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
        ROS_INFO_STREAM("Current pose: \n"
                        << start_pose);

        // 设置目标位置（X和Y各增加0.1米）
        geometry_msgs::Pose target_pose = start_pose;
        target_pose.position.z = 0.45;
        // target_pose.position.y ;

        // 设置运动目标
        move_group_.setPoseTarget(target_pose);

        // 规划运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("规划成功，开始执行");

            // 执行轨迹
            for (const auto &point : plan.trajectory_.joint_trajectory.points)
            {
                for (size_t i = 0; i < point.positions.size(); i++)
                {
                    double angle_deg = point.positions[i] * 180.0 / M_PI;
                    this->write(i, angle_deg);
                }
                this->sendTarget();
                ros::Duration(0.05).sleep();
            }

            ROS_INFO("运动完成");
        }
        else
        {
            ROS_ERROR("规划失败");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_movement_generator");
    LinearMovementGenerator generator;

    generator.waitForSubscriber();
    generator.moveLinearXY();

    return 0;
}