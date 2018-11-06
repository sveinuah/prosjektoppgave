#ifndef FISHEYE_TRANSFORM
#define FISHEYE_TRANSFORM

#include <vector>
#include <cstdint>

#include "Eigen/Dense"

typedef Eigen::Quaternion<double> Quarterniond;
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector2d Vector2d;

class FisheyeTransformer {
public:
	FisheyeTransformer(Matrix3x4d mat);
	~FisheyeTransformer();

	void setCaptureMatrix(Matrix3x4d mat);
	void setFisheyeMatrix(Matrix3x4d mat);
	Matrix3x4d getCaptureMatrix();
	Matrix3x4d getFisheyeMatrix();

	void transform(Quarterniond orientation, Vector3d position, std::vector<uint8_t>& image_data_uint8, std::vector<uint8_t>& target_image_uint8);

private:
	Matrix3x4d capture_matrix_;
	Matrix3x4d fisheye_matrix_;

	Vector2d getCaptureWorldDirection(Quarterniond orientation, Vector3d position, unsigned int u, unsigned int v);
	Vector2d getFisheyeImagePixels(double phi, double theta);

};

#endif /* FISHEYE_TRANSFORM */