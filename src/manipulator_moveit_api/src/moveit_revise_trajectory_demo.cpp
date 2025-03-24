#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

//轨迹重定义
void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale)
{
    int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();

    for(int i=0; i<plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/scale;
        
        for(int j=0; j<n_joints; j++)
        {
            plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;
            plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale*scale;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_revise_trajectory_demo");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    ros::Duration(1.0).sleep();


    double targetPose[7] = {0.391410, -0.676384, -0.376217, 0.0,0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(7);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];
    joint_group_positions[6] = targetPose[6];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    ros::Duration(1.0).sleep();

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    ros::Duration(1.0).sleep();


    arm.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   
    
    scale_trajectory_speed(plan, 0.25);

    //让机械臂按照规划的轨迹开始运动。
    if(success)
      arm.execute(plan);
    ros::Duration(1.0).sleep();

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    ros::Duration(1.0).sleep();

    ros::shutdown(); 

    return 0;
}
