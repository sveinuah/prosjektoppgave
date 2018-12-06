#include "fisheye.hpp"

#include <iostream>

namespace fisheye {

Lens::Lens(float scale, const std::vector<float>& params, PixelCoordinate dist_c, const Mat2x2& stretch_m)
	: scale(scale), distortion_center(dist_c), stretch_matrix(stretch_m)
{
	for (auto param : params)
	{
		this->scaramuzza_params.push_back(param);
	}
}

Lens::Lens(float scale, const std::vector<float>& params)
{
	PixelCoordinate dist_c = {0, 0};
	Mat2x2 stretch_m = Mat2x2::Identity();
	Lens(scale, params, dist_c, stretch_m);
}

Lens::Lens(const Lens& l) : Lens(l.scale, l.scaramuzza_params, l.distortion_center, l.stretch_matrix)
{}

Lens::Lens()
{
	std::vector<float> params;
	params.push_back(0.0f); //a0
	params.push_back(1.0f); //a1
	params.push_back(0.0f); //a2
	params.push_back(0.0f); //a3
	Lens(1.0f, params);
}

void Camera::setPose(const Pose& p)
{
	camera_pose_ = p;
}

void Camera::setFOV(unsigned int fov_h, unsigned int fov_v)
{
	fov_h_ = fov_h;
	fov_v_ = fov_v;
}

void Camera::printParameters()
{
	std::cout << "Camera position: ";// << camera_pose_.position << std::endl;
	std::cout << "Camera orientation: ";// << camera_pose_.orientation << std::endl;
	std::cout << "Horizontal field of view: " << fov_h_ << std::endl;
	std::cout << "Vertical field of view: " << fov_v_ << std::endl;
	std::cout << "Image resolution: " << image_.width << "x" << image_.height << std::endl;
}

PixelCoordinate unitSphereToPixel(UnitSphereCoordinate c)
{
	std::cout << "Getting pixel coords for {" << c.theta << ", " << c.phi << "}" << std::endl;
}

UnitSphereCoordinate pixelToUnitSphere(PixelCoordinate c)
{
	std::cout << "Getting unit sphere coords for {" << c.u << ", " << c.v << "}" <<std::endl;
}



void FisheyeTransformer::combineAndTransform(const std::vector<Image> src_images, const std::vector<Pose>& poses, const std::vector<ProjectionMatrix>& img_matrices)
{
	for (std::vector<Image>::size_type i = 0; i < src_images.size(); i++)
	{
		addToImage(src_images[i], poses[i], img_matrices[i]);
	}
}

void FisheyeTransformer::addToImage(const Image& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat)
{
	std::cout << "Adding to target image" << std::endl;
}

UnitSphereCoordinate FisheyeTransformer::calculateSphereCoords(const Pose& pose, float aspect_ratio, float focal_length, ImageCoordinate feature) const
{
	feature.x = feature.x / aspect_ratio;
	const VectorMath::Vector3f img_pos(feature.x, feature.y, focal_length);
	std::cout << "img_pos: " << img_pos << std::endl;
	VectorMath::Vector3f world_pos = VectorMath::transformToWorldFrame(img_pos, pose, true);
	std::cout << "world_pos: " << world_pos << std::endl;
	world_pos.normalize();
	std::cout << "world_pos normalized: " << world_pos << std::endl;

	float theta = std::atan2(world_pos[1], world_pos[0]);
	std::cout << "theta: " << theta << std::endl;
	float phi = std::acos(world_pos[2]);
	std::cout << "phi: " << phi << std::endl;

	return {theta, phi};
}
} //namespace fisheye end