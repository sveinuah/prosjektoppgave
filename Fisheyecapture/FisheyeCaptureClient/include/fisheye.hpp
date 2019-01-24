#ifndef FISHEYE_TRANSFORM
#define FISHEYE_TRANSFORM

#include <vector>
#include <cstdint>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace msr { namespace airlib {

struct UnitSphereCoordinate {
	float theta, phi;
};

struct Lens {
	float scale;
	float k1, k2, k3, k4; //Following the polynomial model

	Eigen::Vector2f distortion_center;
	Eigen::Matrix2f stretch_matrix;

	Lens();
	Lens(const Lens& l);
	Lens(float scale, float k1, float k2, float k3, float k4);
	Lens(float scale, float k1, float k2, float k3, float k4, Eigen::Vector2f dist_c, const Eigen::Matrix2f& stretch_m);
	~Lens();

	float distort(float phi);
};

enum class CameraPosition : int {
	DOWN = 0,
	FRONT,
	LEFT,
	BACK,
	RIGHT
};

struct SourceImage {
	cv::Mat image;
	CameraPosition pos;
	int height; int width;

	SourceImage() {}
	SourceImage(cv::Mat img, CameraPosition position, int height_val, int width_val);
	~SourceImage();
};

class FisheyeTransformer {

public:
	FisheyeTransformer(int dest_height, int dest_width, int src_height, int src_width, Lens lens);
	~FisheyeTransformer();

	cv::Mat& transformAndCombine(const std::vector<SourceImage>& src_imgs);
	cv::Mat& transformSingle(const SourceImage& src_img);

private:
	

	void addToImage(const SourceImage& src_img);

	double calculatePhiMax(const Lens& lens) const;

	UnitSphereCoordinate calculateSphereCoords(const Eigen::Vector3f& coord) const;

	void makeCameraRotations();
	void rotateToCameraFrame(Eigen::Vector3f* c, CameraPosition pos);

	cv::Mat fisheye_image_;
	Lens lens_;

	Eigen::Matrix<float,3,3> PixelTransform;
	Eigen::Matrix<float,3,3> InversePixelTransform;

	std::vector<Eigen::Quaternionf> camera_rotations;

};
}} //namespace msr::airlib end

#endif /* FISHEYE_TRANSFORM */