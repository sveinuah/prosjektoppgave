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
#include <chrono>

#define LOOP_TIME 100 // Time in seconds
#define LOOP_VELOCITY 3.0f // velocity float 


int main() 
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
    	client.confirmConnection();

    	std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
        client.enableApiControl(true);
        client.armDisarm(true);
        std::cout << "Drone armed" << std::endl;

        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        std::cout << "Press enter to start movement loop" << std::endl
        std::cout << "Looping for " << LOOP_TIME << "seconds. With a velocity of " << LOOP_VELOCITY << " m/s" << std::endl

               
    }

	catch (rpc::rpc_error&  e) {
    	std::string msg = e.get_error().as<std::string>();
    	std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}