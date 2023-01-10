#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Mode_switching
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber odometry_sub;
	ros::Subscriber imu_sub;
	ros::Publisher mode_pub;
	ros::ServiceClient client;

	gazebo_msgs::SetModelState iris_state;
	geometry_msgs::Quaternion orientation;
	geometry_msgs::Point position;
	geometry_msgs::Vector3 linear_acc;
	geometry_msgs::Vector3 angular_vel;
	geometry_msgs::Vector3 linear_vel;
	Eigen::Quaterniond gravity_eigen;
	std_msgs::Int32 mode; // 1 for attitude control, 2 for current position mode 

	int falling_count;
	int stable_count;
	bool falling; 

public:
	Mode_switching(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
	~Mode_switching();

	void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg);
	void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void computeGravity();
	void mode_switch();

	void publishMode();
};

Mode_switching::Mode_switching(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
		:nh_(nh),
		 private_nh_(private_nh)
{
	imu_sub = nh_.subscribe("/iris/imu", 1, &Mode_switching::imu_data_cb, this);
	odometry_sub = nh_.subscribe("/iris/ground_truth/odometry", 1, &Mode_switching::odometry_cb, this);
	mode_pub = nh_.advertise<std_msgs::Int32>("/iris/mode", 2);
	client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	iris_state.request.model_state.model_name = "iris";
	client.call(iris_state);

	falling_count = stable_count = 0;
	mode.data = 0;
	falling = false;
}

Mode_switching::~Mode_switching()
{
	mode.data = 0;
	publishMode();
	client.call(iris_state);
}

void Mode_switching::imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	linear_acc = msg->linear_acceleration;
	orientation = msg->orientation;
}

void Mode_switching::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	
	position = msg->pose.pose.position;
	angular_vel = msg->twist.twist.angular;
	linear_vel = msg->twist.twist.linear;
}

void Mode_switching::computeGravity()
{
	Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
	q.normalize();
	Eigen::Quaterniond q_inv(orientation.w, orientation.x, orientation.y, orientation.z);
	Eigen::Quaterniond v(0, linear_acc.x, linear_acc.y, linear_acc.z);

	gravity_eigen = q_inv*v*q;
}

void Mode_switching::mode_switch()
{
	float angular_vel_norm = sqrt(pow(angular_vel.x, 2) + pow(angular_vel.y, 2) + pow(angular_vel.z, 2));
	float linear_vel_norm = sqrt(pow(linear_vel.x, 2) + pow(linear_vel.y, 2) + pow(linear_vel.z, 2));

	if(abs(position.z) < 0.1)
	{
		std::cout << "Ready to launch!" << std::endl;
		mode.data = 0;
	}

	if(gravity_eigen.vec().norm() < 0.5)
		falling_count++;
	else
		falling_count = 0;
	if(falling_count >= 10)
	{
		std::cout << "Switch to attitude mode" << std::endl;
		mode.data = 1;
	}

	
	if(mode.data == 1 && angular_vel_norm < 0.5)
		stable_count++;
	else
		stable_count = 0;
	if(stable_count >= 5)
	{
		std::cout << "Stablize mav to current position" << std::endl;
		mode.data = 2;
	}

	if(mode.data == 2 && linear_vel_norm < 0.5)
	{
		std::cout << "Position mode" << std::endl;
		mode.data = 3;
	}

}

void Mode_switching::publishMode()
{
	mode_pub.publish(mode);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mode_switching_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Rate rate(100);
	Mode_switching MS(nh, private_nh);

	while(ros::ok())
	{
		MS.computeGravity();
		MS.mode_switch();
		MS.publishMode();
		ros::spinOnce();
		rate.sleep();
	}
}