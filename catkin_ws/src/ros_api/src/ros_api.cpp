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
#include <chrono>
#include <ctime>
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
	//typedef FisheyeTransformer::CameraPosition CameraPosition;
	//typedef FisheyeTransformer::SourceImage SourceImage;
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

	client.takeoffAsync(5)->waitOnLastTask();
	client.moveToZAsync(-0.3, 0.3)->waitOnLastTask();
	//client.hoverAsync()->waitOnLastTask();
	//client.rotateByYawRateAsync(150,10)->waitOnLastTask();
	client.moveToPositionAsync(0.0,-1.5,-0.3,0.5)->waitOnLastTask();
	client.moveByVelocityAsync(0.0,2.0,0.0,10)->waitOnLastTask();
	
	ImageCaptureCube::CubeImageRequest req;
	// TESTING ********************************************
	
	//ImageRequest request("left_center", ImageCaptureBase::ImageType::Scene, false, false);
	//std::vector<ImageRequest> v;
	//v.push_back(request);

	std::vector<SourceImage> images;

	auto start = std::chrono::system_clock::now();
	//const std::vector<ImageResponse>& responses = c.simGetImages(v);
	const std::vector<ImageResponse>& responses = c.simGetImages(req.capture_requests);
	std::vector<cv::Mat*> mat_ptrs;
	std::cout << "Images received: " << responses.size() << std::endl;

	for (ImageResponse response : responses) {

		cv::Mat* temp_img = new cv::Mat;
		*temp_img = cv::Mat(response.height, response.width, CV_8UC4);
		memcpy(temp_img->data, response.image_data_uint8.data(), response.image_data_uint8.size());
		mat_ptrs.push_back(temp_img);

		CameraPosition pos;

		if (response.camera_name == "forward_center") {
			pos = CameraPosition::FRONT;
		}
		else if (response.camera_name == "left_center") {
			pos = CameraPosition::LEFT;
		}
		else if (response.camera_name == "right_center") {
			pos = CameraPosition::RIGHT;
		}
		else if (response.camera_name == "backward_center") {
			pos = CameraPosition::BACK;
		}
		else if (response.camera_name == "down_center") {
			pos = CameraPosition::DOWN;
		}
		else {
			std::cout << "DOES NOT MATCH NAME!!" << std::endl;
		}

		images.emplace_back(SourceImage(*temp_img, pos, temp_img->size().height, temp_img->size().width));
	}

	for (int i = 0; i < mat_ptrs.size(); i++) {
		delete mat_ptrs[i];
	}

	Lens lens(1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
	FisheyeTransformer fish(800, 800, images.at(0).height, images.at(0).width, lens);
	auto conv_time = std::chrono::system_clock::now();
	cv::Mat output = fish.transformAndCombine(images);
	//images.at(0).pos = CameraPosition::DOWN; //For singles
	//cv::Mat output = fish.transformSingle(images.at(0)); //For singles
	auto finished = std::chrono::system_clock::now();

	std::chrono::duration<double> fetch_time = conv_time - start;
	std::chrono::duration<double> transform_time = finished - conv_time;
	std::chrono::duration<double> total = fetch_time + transform_time;

	std::cout << "IMG get: " << fetch_time.count() << "Seconds" << std::endl;
	std::cout << "IMG convert: " << transform_time.count() << "Seconds" << std::endl;
	std::cout << "IMG total: " << total.count() << "Seconds" << std::endl;

	cv::Mat cvt_img;
	cv::cvtColor(output, cvt_img, cv::COLOR_RGBA2BGRA);

/*	cv::namedWindow("edges",1);
	cv::imshow("edges", output);
	cv::waitKey(0);*/

	std::vector<int> params;
	params.push_back(CV_IMWRITE_JPEG_QUALITY);
	params.push_back(95);

	cv::imwrite("/home/schwung/Pictures/fisheye.jpeg", cvt_img, params);

	// UNTIL HERE !!! *************************************

	ROS_INFO("Starting Multirotor node!");
	
	ros::init(argc, argv, "multirotor");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("test_msg", 1000, screenLoggerCallback);
	ros::Publisher pub = n.advertise<sensor_msgs::Image>("/FisheyeImages", 1);

	ros::Rate loop_rate(5);

	while(ros::ok()) {

		cv_bridge::CvImage cv_img;
		cv_img.image = output;
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