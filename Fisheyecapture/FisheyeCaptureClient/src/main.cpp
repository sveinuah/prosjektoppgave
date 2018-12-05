#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "fisheye.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <iostream>
#include <string>

#define NUM_CAMERAS 7

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

fisheye::ProjectionMatrix projectionMatrixAdaptor(const msr::airlib::ProjectionMatrix& proj_mat)
{
	fisheye::ProjectionMatrix mat;
	for (int i = 0; i < 4; i++) 
	{
		for (int j = 0; j < 4; j++) 
		{
			mat(i,j) = proj_mat.matrix[i][j];
		}
	}
	return mat;
}

int main()
{

	msr::airlib::MultirotorRpcLibClient client;

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

	std::vector<ImageRequest> requests;
	std::vector<fisheye::ProjectionMatrix> camera_matrices;
	for (int i = 0; i < NUM_CAMERAS; i++)
	{
		requests.push_back(ImageRequest(camera_list[i], ImageType::Scene));
		camera_matrices.push_back(fisheye::ProjectionMatrix());
	};

	std::string folder_path;
	std::cout << "Type in the path to the folders you want to save the images in" << std::endl; std::getline(std::cin, folder_path);

	try
	{
		client.confirmConnection();
		std::cout << "Press enter to arm the drone and take off" << std::endl; std::cin.get();
		client.enableApiControl(true);
		client.armDisarm(true);
		client.takeoffAsync()->waitOnLastTask();
		client.hoverAsync()->waitOnLastTask();

		std::cout << "Getting images" <<  std::endl;
		const vector<ImageResponse>& responses = client.simGetImages(requests);

		for (int i = 0; i < NUM_CAMERAS; i++)
		{
			camera_matrices[i] = projectionMatrixAdaptor(client.simGetCameraInfo(camera_list[i]).proj_mat); //TODO: Add vehicle name as second argument
			std::cout << camera_matrices[i] << std::endl << std::endl;
		}

		for (const ImageResponse& response : responses)
		{
			if (folder_path != "")
			{
				std::string file_path = FileSystem::combine(folder_path, response.camera_name + "_" + std::to_string(response.time_stamp));
				std::cout << file_path << std::endl;
				std::ofstream file(file_path + ".png", std::ios::binary);
				file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()), response.image_data_uint8.size());
				file.close();
			}
		}

		client.landAsync()->waitOnLastTask();
		client.armDisarm(false);
		client.enableApiControl(false);

	}

	catch (rpc::rpc_error& e)
	{
		std::string err = e.get_error().as<std::string>();
		std::cout << "Exeption raised by the API:" << std::endl << err << std::endl;
	}

	return 0;
}