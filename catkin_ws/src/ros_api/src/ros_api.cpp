#ifndef ros_api
#define ros_api

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
#include "rpc/client.h"
STRICT_MODE_ON

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include "ros_api.hpp"
#include "api/RpcLibAdapatorsBase.hpp"

namespace msr { namespace airlib {

	// ************ Local Typedefs ******************
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;
	// **********************************************

	RosRpcLibClient::RosRpcLibClient(const std::string& ip_address, uint16_t port, float timeout_sec) : RpcLibClientBase(ip_address, port, timeout_sec) {

	}

	RosRpcLibClient::~RosRpcLibClient() {}

	void RosRpcLibClient::connectAndArm(const std::string& vehicle_name) {
		try {
			confirmConnection();
			enableApiControl(true, vehicle_name);
			armDisarm(true, vehicle_name);
		}
		catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
		}

		ROS_INFO("Connected!");
	}

	void RosRpcLibClient::disconnectAndDisarm(const std::string& vehicle_name) {
		try {
			armDisarm(false, vehicle_name);
			enableApiControl(false, vehicle_name);
		}
		catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
		}

		ROS_INFO("Disconnected!");
	}

}} //namespace msr::airlib

void screenLoggerCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Got: [%s]", msg->data.c_str());
	std::cout << "Woah! Melding!" << std::endl;
}

int main(int argc, char* argv[]) {

	msr::airlib::RosRpcLibClient c;
	c.connectAndArm();

	ROS_INFO("Starting Multirotor node!");
	
	ros::init(argc, argv, "multirotor");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("test_msg", 1000, screenLoggerCallback);
	ros::Publisher pub = n.advertise<sensor_msgs::Image>("/FisheyeImages", 1);


	ros::Rate loop_rate(5);
	while(ros::ok()) {

		cv_bridge::CvImage cv_img;
		cv_img.image = cv::imread("/home/schwung/testbilde.jpg", cv::IMREAD_COLOR);
		cv_img.encoding = "bgr8";

		sensor_msgs::Image ros_img;
		cv_img.toImageMsg(ros_img);

		pub.publish(ros_img);

		ros::spinOnce();
	}
	
	c.disconnectAndDisarm();

	return 0;
}

#endif // ros_api