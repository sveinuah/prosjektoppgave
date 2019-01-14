#ifndef ros_api
#define ros_api

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "api/MultirotorRpcLibClient.hpp"

namespace msr {namespace airlib {

void connect(const MultirotorRpcLibClient& client) {
	try {
		client.confirmConnection();

		ROS_INFO("Connedted to multirotor simulation");
	}
	
	catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
	}
}

void screenLoggerCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Got: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {

	MultirotorRpcLibClient client;
	connect(client);
	
	ros::init(argc, argv, "multirotor");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("test_msg", 1000, screenLoggerCallback);

	ros::spin();

}

}} //namespace msr::airlib
#endif // ros_api