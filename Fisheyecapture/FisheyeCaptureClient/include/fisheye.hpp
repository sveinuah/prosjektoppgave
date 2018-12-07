#ifndef FISHEYE_TRANSFORM
#define FISHEYE_TRANSFORM

#include <vector>
#include <cstdint>

#include "common/VectorMath.hpp"

	typedef msr::airlib::VectorMathf VectorMath;
	typedef VectorMath::Pose Pose;

namespace fisheye {

	typedef Eigen::Matrix<float, 4, 4> ProjectionMatrix;
	typedef Eigen::Matrix<float, 2, 2> Mat2x2;

struct UnitSphereCoordinate {
	float theta, phi;
};

struct ImageCoordinate {
	float x, y;
};

struct PixelCoordinate {
	unsigned int u, v;
};

enum struct Color {
	RED = 0,
	GREEN,
	BLUE,
	ALPHA
};

struct Image {

	std::vector<uint8_t> image;
	unsigned int width;
	unsigned int height;

	Image(const std::vector<uint8_t>& img, unsigned int size_x, unsigned int size_y)
		: image(img), width(size_x), height(size_y) {}

	Image(unsigned int size_x, unsigned int size_y)
		: width(size_x), height(size_y)
	{
		image = std::vector<uint8_t>(size_x * size_y * 4, 0); //all black RGBA color
	}
	~Image() {}

	PixelCoordinate indexToPixel(int index) const
	{
		return {(index/4)%width, (index/4)/width};
	}
};

struct Lens {
	float scale;
	std::vector<float> scaramuzza_params; // Scaramuzza model constants

	PixelCoordinate distortion_center;
	Mat2x2 stretch_matrix;

	Lens();
	Lens(const Lens& l);
	Lens(float scale, const std::vector<float>& params);
	Lens(float scale, const std::vector<float>& params, PixelCoordinate dist_c, const Mat2x2& stretch_m);

	PixelCoordinate Project();
};

class Camera {
public:
	Camera(const Pose& p, unsigned int size_x, unsigned int size_y, float fov_h, float fov_v)
		: camera_pose_(p), fov_h_(fov_h), fov_v_(fov_v), image_(Image(size_x,size_y)) {}

	Camera(const Pose& p, unsigned int size_x, unsigned int size_y, float fov_h, float fov_v, const Lens& l)
		: lens_(Lens(l)), camera_pose_(p), fov_h_(fov_h), fov_v_(fov_v), image_(Image(size_x,size_y)) {}

	~Camera() {}

	void setPose(const Pose& p);
	void setFOV(unsigned int fov_h, unsigned int fov_v);
	void printParameters();

	PixelCoordinate unitSphereToPixel(UnitSphereCoordinate c);
	UnitSphereCoordinate pixelToUnitSphere(PixelCoordinate c);

private:
	Lens lens_;
	Pose camera_pose_;
	unsigned int fov_h_;
	unsigned int fov_v_;
	Image image_;
};

class FisheyeTransformer {

public:
	FisheyeTransformer(const Camera& cam) : c_(cam) {} 
	~FisheyeTransformer() {}

	void combineAndTransform(const std::vector<Image> src_images, const std::vector<Pose>& poses, const std::vector<ProjectionMatrix>& img_matrices);
	void transformSingle(const Image& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat);

private:
	Camera c_;

	void addToImage(const Image& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat);
	UnitSphereCoordinate calculateSphereCoords(const Pose& pose, float aspect_ratio, float focal_length, ImageCoordinate feature) const;
};
} //namespace fisheye end

#endif /* FISHEYE_TRANSFORM */