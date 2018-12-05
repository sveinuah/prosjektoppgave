#include "fisheye.hpp"

#include <iostream>

namespace fisheye {


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
	std::cout << "Image resolution: " << image_.pixels_x << "x" << image_.pixels_y << std::endl;
}



void FisheyeTransformer::combineAndTransform(const std::vector<Image> src_images, const std::vector<Pose>& poses, const std::vector<ProjectionMatrix>& img_matrices, Image& target_img)
{
	for (std::vector<Image>::size_type i = 0; i < src_images.size(); i++)
	{
		addToImage(src_images[i], poses[i], img_matrices[i], target_img);
	}
}

void FisheyeTransformer::addToImage(const Image& src_img, const Pose& pose, const ProjectionMatrix& src_mat, Image& target_img)
{
	std::cout << "Adding to target image" << std::endl;
}

Vector2f FisheyeTransformer::calculateSphereCoords(const Pose& pose, const ProjectionMatrix& mat, const float img_x, const float img_y) const
{
	const Vector3f img_pos(img_x, img_y, 1);
	const Vector3f world_pos = VectorMath::transformToWorldFrame(img_pos, pose, true);

	float theta = std::atan2(world_pos[1], world_pos[0]);
	float phi = std::atan2(world_pos[0], world_pos[2] * std::cos(theta));

	return Vector2f{theta, phi};
}
} //namespace fisheye end