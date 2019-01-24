#ifndef ros_api_hpp
#define ros_api_hpp

#include <string>

#include "api/RpcLibClientBase.hpp"

namespace msr { namespace airlib {

	class RosRpcLibClient : public RpcLibClientBase {

	public:
		RosRpcLibClient(const std::string& ip_address = "localhost", uint16_t port = 41451, float timeout_sec = 60);
		virtual ~RosRpcLibClient();

		void connectAndArm(const std::string& vehicle_name = "");
		void disconnectAndDisarm(const std::string& vehicle_name = "");

		std::vector<ImageCaptureBase::ImageResponse> simGetPerspectiveImages(std::vector<ImageCaptureBase::ImageRequest> req, const std::string& vehicle_name);

};

}} //namespace msr::airsim
#endif //ros_api_hpp