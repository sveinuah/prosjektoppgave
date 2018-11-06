#include "fisheyetransformer.hpp"

#include <iostream>

FisheyeTransformer::FisheyeTransformer(Matrix3x4d mat)
{
	capture_matrix_ = mat;
	fisheye_matrix_ << 	0, 0, 0, 0,
						0, 0, 0, 0,
						0, 0, 0, 0;
}

FisheyeTransformer::~FisheyeTransformer(){}

void FisheyeTransformer::setCaptureMatrix(Matrix3x4d mat)
{
	capture_matrix_ = mat;
}

void FisheyeTransformer::setFisheyeMatrix(Matrix3x4d mat)
{
	fisheye_matrix_ = mat;
}

Matrix3x4d FisheyeTransformer::getCaptureMatrix()
{
	return capture_matrix_;
}

Matrix3x4d FisheyeTransformer::getFisheyeMatrix()
{
	return fisheye_matrix_;
}

void FisheyeTransformer::transform(Quarterniond orientation, Vector3d position, std::vector<uint8_t>& src_img, std::vector<uint8_t>& target_img)
{
	std::cout << "Transforming!" << std::endl;
}

Vector2d FisheyeTransformer::getCaptureWorldDirection(Quarterniond orientation, Vector3d position, unsigned int u, unsigned int v)
{
	Vector2d vect(0.0, 0.0);

	return vect;
}

Vector2d FisheyeTransformer::getFisheyeImagePixels(double phi, double theta)
{
	Vector2d vect(0.0, 0.0);

	return vect;
}