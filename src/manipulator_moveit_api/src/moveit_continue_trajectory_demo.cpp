#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_revise_trajectory_demo");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    double accScale = 0.8;
    double velScale = 0.8;
    arm.setMaxAccelerationScalingFactor(accScale);
    arm.setMaxVelocityScalingFactor(velScale);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    ros::Duration(1.0).sleep();

    // 获取机器人的起始位置
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(arm.getName());

    std::vector<double> joint_group_positions;
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //设置第一个目标点
    joint_group_positions[0] = -0.06;  // radians
    arm.setJointValueTarget(joint_group_positions);

    // 计算第一条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::core::MoveItErrorCode success = arm.plan(plan1);

    joint_model_group = start_state->getJointModelGroup(arm.getName());    
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    arm.setStartState(*start_state);

    //设置第二个目标点
    joint_group_positions[0] = -0.05;  // radians
    joint_group_positions[1] = -0.03;  // radians
    arm.setJointValueTarget(joint_group_positions);

    // 计算第二条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = arm.plan(plan2);

    //连接两条轨迹
    moveit_msgs::RobotTrajectory trajectory;

    trajectory.joint_trajectory.joint_names = plan1.trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = plan1.trajectory_.joint_trajectory.points;
    for (size_t j = 1; j < plan2.trajectory_.joint_trajectory.points.size(); j++)
    {
        trajectory.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[j]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velScale, accScale);

    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;

    if (!arm.execute(joinedPlan))
    {
        ROS_ERROR("Failed to execute plan");
        return false;
    }

    ros::Duration(1.0).sleep();

    ROS_INFO("Finished");

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    ros::Duration(1.0).sleep();

    ros::shutdown(); 

    return 0;
}
