#ifndef FISHEYE_TRANSFORM
#define FISHEYE_TRANSFORM

#include <vector>
#include <cstdint>

#include <opencv2/opencv.hpp>

#include "common/VectorMath.hpp"

namespace msr { namespace airlib {

struct UnitSphereCoordinate {
	float theta, phi;
};

struct ImageCoordinate {
	float x, y;
};

struct Pixel {
	int u, v;
};

struct Lens {
	float scale;
	std::vector<float> distortion_params; //Following the polynomial model

	Pixel distortion_center;
	Eigen::Matrix<float, 2, 2> stretch_matrix;

	Lens();
	Lens(const Lens& l);
	Lens(float scale, const std::vector<float>& params);
	Lens(float scale, const std::vector<float>& params, Pixel dist_c, const Eigen::Matrix<float, 2, 2>& stretch_m);
};

class FisheyeTransformer {

public:
	FisheyeTransformer(int height, int width);
	~FisheyeTransformer();

	struct SourceImage {
		cv::Mat image;
		VectorMathf::Pose camera_pose;
		int height; int width;

		SourceImage(cv::Mat img, VectorMathf::Pose pose, int height_val, int width_val) {
			image = img;
			camera_pose = pose;
			height = height_val;
			width = width_val;
		}
	};

	struct TransformRequest {
		std::vector<SourceImage> src_images;
		Eigen::Matrix<float, 4, 4> projectionMatrix;

		TransformRequest(std::vector<SourceImage>& img_vec, const Eigen::Matrix<float, 4, 4>& proj_mat) {
			src_images = img_vec;
			projectionMatrix = proj_mat;
		}
	};

	void transformSingle(const SourceImage& src_img, const VectorMathf::Pose& src_pose, const Eigen::Matrix<float, 4, 4>& src_mat);
	cv::Mat& transformAndCombine(const TransformRequest& req);

private:
	

	void addToImage(const SourceImage& src_img, const VectorMathf::Pose& src_pose, const Eigen::Matrix<float, 4, 4>& src_mat);

	UnitSphereCoordinate calculateSphereCoords(const VectorMathf::Pose& pose, float aspect_ratio, float focal_length, ImageCoordinate feature) const;

	Pixel indexToPixel(int index, int width);
	int pixelToIndex(Pixel p, int width);
	UnitSphereCoordinate pixelToUnitSphere(Pixel p);

	Pixel getDestinationPixel(UnitSphereCoordinate c);

	cv::Mat fisheye_image_;

};
}} //namespace msr::airlib end

#endif /* FISHEYE_TRANSFORM */