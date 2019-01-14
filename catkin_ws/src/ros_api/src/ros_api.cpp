#ifndef ros_api
#define ros_api

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
//#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

namespace msr {namespace airlib {

void connect(MultirotorRpcLibClient& client) {
	try {
		client.confirmConnection();

		ROS_INFO("Connedted to multirotor simulation");
	}
	catch(...) {
		std::cout << "Error!" << std::endl;
	}
	
/*	catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
	}*/
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

	return 0;
}

}} //namespace msr::airlib
#endif // ros_api