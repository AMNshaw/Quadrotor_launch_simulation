#include "ros/ros.h"
#include <string>
#include <iostream>
#include <gazebo_msgs/DeleteModel.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawnturtles");
    ros::NodeHandle n;
    std::string mav_name;

    // Check if the service is on
    n.getParam("mav_name", mav_name);
    ros::service::waitForService("/gazebo/delete_model");

    // initialize kill turtle service
    ros::ServiceClient client_kill=n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel srv;
    // define parameters in service
    srv.request.model_name = mav_name;
    // call the service
    if (client_kill.call(srv))
    {
        ROS_INFO("Model deleted");
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/delete_model");
    }

    return 0;
}