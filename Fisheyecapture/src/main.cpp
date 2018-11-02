#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <string>

#define NUM_CAMERAS 7

int main()
{
	using namespace msr::airlib;
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	typedef common_utils::FileSystem FileSystem;

	MultirotorRpcLibClient client;

	const std::string camera_list[NUM_CAMERAS] = 
	{
		"front_center",
		"right_front",
		"right_back",
		"back_center",
		"left_back",
		"left_front",
		"bottom_center"
	};

	vector<ImageRequest> requests;
	vector<ProjectionMatrix> camera_matrices;
	for (i = 0; i < NUM_CAMERAS; i++)
	{
		requests.push_back(ImageRequest(camera_list[i], ImageType::Scene));
		camera_matrices.push_back(CameraInfo().proj_mat);
	};
	requests = const_cast<const vector<ImageRequest>>(requests);
	camera_matrices = const_cast<const vector<ProjectionMatrix>>(camera_matrices);

	std::string folder_path;
	std::cout << "Type in the path to the folders you want to save the images in" << std::endl; std::getline(std::cin, folder_path);

	try
	{
		client.confirmConnection();
		std::cout << "Press enter to arm the drone and take off" << std::endl; std::cin.get();
		client.enableApiControl(true);
		client.armDisarm(true);
		client.takeOffAsync()->waitOnLastTask();
		client.hoverAsync()->waitOnLastTask();

		const vector<ImageResponse>& responses = client.simGetImages(requests);

		for (int i = 0; i < NUM_CAMERAS; i++)
		{
			camera_info[i] = simGetCameraInfo(camera_list[i]); //TODO: Add vehicle name as second argument
		}

		for (ImageResponse& response : responses)
		{
			if (folder_path != "")
			{
				std::string file_path = FileSystem::combine(folder_path, response.camera_name + "_" + std::to_string(response.time_stamp));
				std::ofstream file(file_path + ".png", std::ios::binary);
				file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()), response.image_data_uint8.size());
				file.close();
			}
		}

	}

	catch (rpc::rpc_error& e)
	{
		std::string err = e.get_error().as<std::string>();
		std::cout << "Exeption raised by the API:" << std::endl << err << std::endl;
	}

	return 0;
}