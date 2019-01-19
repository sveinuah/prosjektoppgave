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
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "ImageCaptureCube.hpp"
#include "fisheye.hpp"

namespace msr { namespace airlib {

	// ************ Local Typedefs ******************
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;
	typedef Eigen::Matrix<float, 4, 4> ProjMat;
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

// ******************** Fisheye realted stuff **********************************

}} //namespace msr::airlib

void screenLoggerCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Got: [%s]", msg->data.c_str());
	std::cout << "Woah! Melding!" << std::endl;
}

int main(int argc, char* argv[]) {

	using namespace msr::airlib;

	RosRpcLibClient c;
	MultirotorRpcLibClient client;
	c.connectAndArm();
	client.confirmConnection();
	client.enableApiControl(true);
	client.armDisarm(true);

	client.takeoffAsync();
	client.hoverAsync()->waitOnLastTask();

	ImageCaptureCube::CubeImageRequest req;

	// TESTING ********************************************
	
	char x;
	std::cout << "Press key!" << std:: endl; std::cin >> x;

	std::vector<FisheyeTransformer::SourceImage> images;

	const vector<ImageResponse>& responses = c.simGetImages(req.capture_requests);
	for (auto response : responses) {

		cv::Mat temp_img(response.height, response.width, CV_8UC4);

		int it = 0;
		int i, j = 0;
		for (auto& data : response.image_data_uint8) {
			i = (it/4)%response.width;
			j = (it/4)/response.width;

			temp_img.at<uchar>(i,j);
			it++;
		}

		VectorMathf::Pose p(response.camera_position, response.camera_orientation);
		FisheyeTransformer::SourceImage img(temp_img, p, response.height, response.width);

		images.push_back(img);
	}

	Eigen::Matrix<float, 4, 4> proj = Eigen::Matrix<float, 4, 4>::Identity();
	FisheyeTransformer::TransformRequest request(images, proj);
	FisheyeTransformer fish(640, 640);
	cv::Mat output = fish.transformAndCombine(request);

	cv::namedWindow("edges",1);
	cv::imshow("edges", output);
	cv::waitKey(0);

	// UNTIL HERE !!! *************************************

	ROS_INFO("Starting Multirotor node!");
	
	ros::init(argc, argv, "multirotor");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("test_msg", 1000, screenLoggerCallback);
	ros::Publisher pub = n.advertise<sensor_msgs::Image>("/FisheyeImages", 1);

	ros::Rate loop_rate(5);

	constexpr int count = 0;
	while(ros::ok()) {

		cv_bridge::CvImage cv_img;
		


		cv_img.encoding = "rgba8";

		sensor_msgs::Image ros_img;
		cv_img.toImageMsg(ros_img);

		pub.publish(ros_img);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	c.disconnectAndDisarm();

	return 0;
}

#endif // ros_api