#include "apiHeader.hpp"
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
        
        if (!(client.enableApiControl(true))) {
        	throw "API control disabeled condition!";
        }

        client.armDisarm(true);
        std::cout << "Drone armed" << std::endl;

        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        // Main travel loo
        std::cout << "Press enter to start movement loop" << std::endl;
        std::cout << "Looping for " << LOOP_TIME << "seconds. With a velocity of " << LOOP_VELOCITY << " m/s" << std::endl;

        auto position = client.getMultirotorState().getPosition();
        float zPos = position.z();
        const float speed = 3.0f;
        const float duration = 4.0f;

        DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
        YawMode::Zero();

        client.moveByVelocityZAsync(speed, 0, zPos, duration, driveTrain, YawMode)->waitOnLastTask();
        client.moveByVelocityZAsync(0, speed, zPos, duration, driveTrain, YawMode)->waitOnLastTask();
        client.moveByVelocityZAsync(-speed, 0, zPos, duration, driveTrain, YawMode)->waitOnLastTask();
        client.moveByVelocityZAsync(0, -speed, zPos, duration, driveTrain, YawMode)->waitOnLastTask();

        client.moveByVelocityZAsync(0, 0, zPos, duration/2, driveTrain, YawMode)->waitOnLastTask();
        

        // Hover as an end of loop
        client.hoverAsync()->waitOnLastTask();

        // Landing
        std::cout << "Press enter to land" << std::endl;
        client.landAsync()->waitOnLastTask();

        // Disarming
        std::cout << "Press enter to disarm drone" << std::endl;
        client.armDisarm(false);
    }

    catch (const char* e) {
    	std::cout << *e << std::endl;
    }

	catch (rpc::rpc_error&  e) {
    	std::string msg = e.get_error().as<std::string>();
    	std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}