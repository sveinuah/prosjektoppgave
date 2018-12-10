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

PixelCoordinate Camera::project(UnitSphereCoordinate c)
{
	float r = std::sin(c.phi); //todo implement lens
	ImageCoordinate img_c;
	img_c.x = r * std::cos(c.theta);
	img_c.y = r * std::sin(c.theta);

	float imgw = static_cast<float>(image_.width);
	float imgh = static_cast<float>(image_.height);
	float u = imgw/2 * img_c.x + imgw/2;
	float v = imgh/2 * img_c.y + imgh/2;
	
	PixelCoordinate p;
	p.u = static_cast<unsigned int>(u);
	p.v = static_cast<unsigned int>(v);
	return {p.u, p.v};
}

UnitSphereCoordinate pixelToUnitSphere(PixelCoordinate c)
{
	std::cout << "Getting unit sphere coords for {" << c.u << ", " << c.v << "}" <<std::endl;
	return {0, 0};
}



void FisheyeTransformer::combineAndTransform(const std::vector<Image> src_images, const std::vector<Pose>& poses, const std::vector<ProjectionMatrix>& img_matrices)
{
	for (std::vector<Image>::size_type i = 0; i < src_images.size(); i++)
	{
		addToImage(src_images[i], poses[i], img_matrices[i]);
	}
	std::cout << "Transformation complete" << std::endl;
}

void FisheyeTransformer::transformSingle(const Image& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat)
{
	addToImage(src_img, src_pose, src_mat);
	std::cout << "Single Transformation complete" << std::endl;
}

void FisheyeTransformer::addToImage(const Image& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat)
{
	float focal_length = src_mat(1,1);
	float aspect_ratio = src_mat(0,0) / focal_length;
	int num_pixels = src_img.width * src_img.height;

	float h = static_cast<float>(src_img.height);
	float w = static_cast<float>(src_img.width);

	for (int i = 0; i < num_pixels; i++)
	{
		PixelCoordinate pixel = src_img.indexToPixel(i*4);
		ImageCoordinate feature;

		feature.x = (static_cast<float>(pixel.u) * 2.0f) / w - 1.0f;
		feature.x = feature.x * aspect_ratio;
		feature.y = (static_cast<float>(pixel.v) * 2.0f) / h - 1.0f;

		UnitSphereCoordinate vec = calculateSphereCoords(src_pose, aspect_ratio, focal_length, feature);
		PixelCoordinate p = c_.project(vec);
		c_.image_.image[c_.image_.pixelToIndex(p)] = src_img.image[i];
		c_.image_.image[c_.image_.pixelToIndex(p) + 1] = src_img.image[i + 1];
		c_.image_.image[c_.image_.pixelToIndex(p) + 2] = src_img.image[i + 2];
		c_.image_.image[c_.image_.pixelToIndex(p) + 3] = src_img.image[i + 3];

		std::cout << "{" << unsigned(src_img.image[i*4]) << ", " << unsigned(src_img.image[i*4+1]) << ", " << unsigned(src_img.image[i*4+2]) << ", " << unsigned(src_img.image[i*4+3]) << "} ";
	}
}

UnitSphereCoordinate FisheyeTransformer::calculateSphereCoords(const Pose& pose, float aspect_ratio, float focal_length, ImageCoordinate feature) const
{
	feature.x = feature.x / aspect_ratio;
	const VectorMath::Vector3f img_pos(feature.x, feature.y, focal_length);
	VectorMath::Vector3f world_pos = VectorMath::transformToWorldFrame(img_pos, pose, true);
	world_pos.normalize();

	float theta = std::atan2(world_pos[1], world_pos[0]);
	float phi = std::acos(world_pos[2]);

	return {theta, phi};
}
} //namespace fisheye end