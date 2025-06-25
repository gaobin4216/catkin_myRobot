#include "ros/ros.h"
#include "driver_control/target_position.h"
#include <cstdlib>
#include <ctime>
#include <vector>

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
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    Client client0(nh);
    if (argc != 8)
    {
        ROS_INFO("EROOR");
    }
    else
    {
        ROS_INFO("VAILD DATA");
        for (int i = 0; i < 7; i++)
        {
            client0.write(i, std::stod(argv[i + 1]));
        }
        client0.run();
    }

    return 0;
}