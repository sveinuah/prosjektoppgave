#ifndef FISHEYE_TRANSFORM
#define FISHEYE_TRANSFORM

#include <vector>
#include <cstdint>

#include "common/VectorMath.hpp"

	typedef msr::airlib::VectorMathf VectorMath;
	typedef VectorMath::Pose Pose;

namespace fisheye {

	typedef Eigen::Matrix<float, 4, 4> ProjectionMatrix;
	typedef Eigen::Matrix<float, 3, 4> Matrix34f;
	typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> Vector2f;
	typedef Eigen::Vector3f Vector3f;

class Image {
public:
	Image(const std::vector<uint8_t>& img, unsigned int x, unsigned int y) 
		: image(img), pixels_x(x), pixels_y(y) {}

	Image(unsigned int x, unsigned int y) 
		: pixels_x(x), pixels_y(y)
	{
		image = std::vector<uint8_t>(x*y,0); //all black
	}
	~Image() {}

	std::vector<uint8_t> image;
	unsigned int pixels_x;
	unsigned int pixels_y;
};

class Camera {
public:
	Camera(const Pose& p, unsigned int x, unsigned int y, float fov_h, float fov_v)
		: camera_pose_(p), fov_h_(fov_h), fov_v_(fov_v), image_(Image(x,y)) {}
	~Camera() {}

	void setPose(const Pose& p);
	void setFOV(unsigned int fov_h, unsigned int fov_v);
	void printParameters();

private:
	Pose camera_pose_;
	unsigned int fov_h_;
	unsigned int fov_v_;
	Image image_;
};

class FisheyeTransformer {

public:
	FisheyeTransformer(const Camera& cam) : cam_(cam) {} 
	~FisheyeTransformer() {}

	void combineAndTransform(const std::vector<Image> src_images, const std::vector<Pose>& poses, const std::vector<ProjectionMatrix>& img_matrices, Image& target_img);

private:
	Camera cam_;

	void addToImage(const Image& src_img, const Pose& pose, const ProjectionMatrix& src_mat, Image& target_img);
	Vector2f calculateSphereCoords(const Pose& pose, float aspect_ratio, float focal_length, float img_x, float img_y) const;
};
} //namespace fisheye end

#endif /* FISHEYE_TRANSFORM */