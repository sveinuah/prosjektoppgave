#ifndef IMAGE_CAPTURE_FISHEYE
#define IMAGE_CAPTURE_FISHEYE

#include "common/ImageCaptureBase.hpp"

namespace msr { namespace airlib {

class ImageCaptureCube : public ImageCaptureBase {

public:

	ImageCaptureCube() {}

	struct CubeImageRequest {
		ImageCaptureBase::ImageType image_type = ImageCaptureBase::ImageType::Scene;
		bool pixels_as_float = false;
		bool compress = false;

		std::vector<ImageCaptureBase::ImageRequest> capture_requests;

		CubeImageRequest() {
			for (int i = 0; i < num_cameras; i++) {
				capture_requests.push_back(ImageCaptureBase::ImageRequest(camera_names[i], image_type, pixels_as_float, compress));
			}
		}

		CubeImageRequest(ImageCaptureBase::ImageType type, bool as_float, bool compress_val) {
			for (int i = 0; i < num_cameras; i++) {
				capture_requests.push_back(ImageCaptureBase::ImageRequest(camera_names[i], type, as_float, compress_val));
				image_type = type;
				pixels_as_float = as_float;
				compress = compress_val;
			}
		}

	};

	static void getCameraNames(std::vector<std::string>& vec) {
		for (int i = 0; i < num_cameras; i++) {
			vec.push_back(camera_names[i]);
		}
	} 

private:
	// this must match the names in the camera names defined in AFlyingPawn::getCameras() in FlyingPawn.cpp
	static constexpr int num_cameras = 5;
	static std::string camera_names[num_cameras];
	

};

// Initialize names
std::string ImageCaptureCube::camera_names[ImageCaptureCube::num_cameras] = {"down_center", "forward_center", "left_center", "backward_center", "right_center"};

}} //namespace msr::airlib

#endif // IMAGE_CAPTURE_FISHEYE