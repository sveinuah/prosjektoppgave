#include "fisheyetransformer.hpp"

#include <iostream>

namespace fisheye {

FisheyeTransformer::FisheyeTransformer(const ProjectionMatrix& mat)
{
	capture_matrix_ = mat;
	fisheye_matrix_ << 	0, 0, 0, 0,
						0, 0, 0, 0,
						0, 0, 0, 0;
}

FisheyeTransformer::~FisheyeTransformer(){}

void FisheyeTransformer::setCaptureMatrix(ProjectionMatrix mat)
{
	capture_matrix_ = mat;
}

void FisheyeTransformer::setFisheyeMatrix(Matrix34f mat)
{
	fisheye_matrix_ = mat;
}

ProjectionMatrix FisheyeTransformer::getCaptureMatrix()
{
	return capture_matrix_;
}

Matrix34f FisheyeTransformer::getFisheyeMatrix()
{
	return fisheye_matrix_;
}

void FisheyeTransformer::transform(Pose pose, std::vector<uint8_t>& src_img, std::vector<uint8_t>& target_img)
{
	std::cout << "Transforming!" << std::endl;
}

Vector2f FisheyeTransformer::getCaptureWorldDirection(Pose pose, unsigned int u, unsigned int v)
{
	Vector2f vect(0.0, 0.0);

	return vect;
}

Vector2f FisheyeTransformer::getFisheyeImagePixels(float phi, float theta)
{
	Vector2f vect(0.0, 0.0);

	return vect;
}
} //namespace fisheye end