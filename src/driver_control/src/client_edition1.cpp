#include "ros/ros.h"
#include "driver_control/target_position.h"
#include "driver_control/target_answer.h"
#include <cstdlib>
#include <ctime>
#include <vector>
#define UNITS 7

class Client
{
private:
    // ros::Publisher pub_;
    ros::ServiceClient client_;

public:
    std::vector<double> delta = {0, 0, 0, 0, 0, 0, 0};
    Client(ros::NodeHandle &nh)
    {
        // pub_ = nh.advertise<driver_control::target_position>("target_array", 10);
        client_ = nh.serviceClient<driver_control::target_answer>("target_array");
        if (!client_.waitForExistence(ros::Duration(5.0)))
        {
            ROS_ERROR("关节控制服务不可用!");
        }
    }

    void write_delta(int index, double value)
    {
        delta[index] = value;
    }

    void generateAndSendData()
    {
        // driver_control::target_position msg;
        driver_control::target_answer srv;
        for (int i = 0; i < UNITS; i++)
        {
            srv.request.delta_angles[i] = delta[i];
        }
        client_.call(srv);
        ROS_INFO("Sent new 7-element double array");
    }

    void run_test()
    {
        /*std::vector<std::vector<double>> actions = {{10, 10, 10, 10, 10, 10, 10},
                                                   {10, 10, 10, 10, 10, 10, 10},
                                                    {10, 10, 10, 10, 10, 10, 10},
                                                   {10, 10, 10, 10, 10, 10, 10}};*/

        std::vector<std::vector<double>> actions = {{-10, -10, 10, -10, -10, -10, -10},
                                                    {-10, -10, 10, -10, -10, -10, -10},
                                                    {-10, -10, 10, -10, -10, -10, -10},
                                                    {-10, -10, 10, -10, -10, -10, -10}};

        for (auto &action : actions)
        {
            std::cout << "@@@@";
            for (int i = 0; i < UNITS; i++)
            {
                write_delta(i, action[i]);
            }
            for (int i = 0; i < UNITS; i++)
            {
                ROS_INFO("Element %d: %f", i, delta[i]);
            }
            generateAndSendData();
        }
    }

    void run_with_rate()
    {
        ros::Rate loop_rate(1); // 1Hz

        while (ros::ok())
        {
            generateAndSendData();
            loop_rate.sleep();
        }
    }

    void run()
    {
        // ros::Rate loop_rate(1); // 1Hz

        while (ros::ok())
        {
            generateAndSendData();
            break;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    Client client(nh);
    if (argc != 8)
    {
        ROS_INFO("DEFAULT DATA");
        // ROS_INFO("EROOR");
        client.run_test();
    }
    else
    {
        ROS_INFO("VAILD DATA");
        for (int i = 0; i < UNITS; i++)
        {
            client.write_delta(i, std::stod(argv[i + 1]));
        }
        client.run();
    }

    return 0;
}