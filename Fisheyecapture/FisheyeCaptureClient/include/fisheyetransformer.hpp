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

class FisheyeTransformer {

public:
	FisheyeTransformer(const ProjectionMatrix& mat);
	~FisheyeTransformer();

	void setCaptureMatrix(ProjectionMatrix mat);
	void setFisheyeMatrix(Matrix34f mat);
	ProjectionMatrix getCaptureMatrix();
	Matrix34f getFisheyeMatrix();

	void transform(Pose pose, std::vector<uint8_t>& image_data_uint8, std::vector<uint8_t>& target_image_uint8);

private:
	ProjectionMatrix capture_matrix_;
	Matrix34f fisheye_matrix_;

	Vector2f getCaptureWorldDirection(Pose pose, unsigned int u, unsigned int v);
	Vector2f getFisheyeImagePixels(float phi, float theta);

};
} //namespace fisheye end

#endif /* FISHEYE_TRANSFORM */