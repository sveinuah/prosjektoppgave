#include "fisheye.hpp"

#include <iostream>

namespace msr { namespace airlib {

typedef VectorMathf::Pose Pose;
typedef VectorMathf::Vector3f Vector3f;
typedef Eigen::Matrix<float, 2, 2> StretchMatrix;
typedef Eigen::Matrix<float, 4, 4> ProjectionMatrix;
typedef FisheyeTransformer::SourceImage SourceImage;

Lens::Lens(float scale, const std::vector<float>& params, Pixel dist_c, const StretchMatrix& stretch_m)
	: scale(scale), distortion_center(dist_c), stretch_matrix(stretch_m)
{
	for (auto& param : params)
	{
		this->distortion_params.push_back(param);
	}
}

Lens::Lens(float scale, const std::vector<float>& params)
{
	Pixel dist_c = {0, 0};
	StretchMatrix stretch_m = StretchMatrix::Identity();
	Lens(scale, params, dist_c, stretch_m);
}

Lens::Lens(const Lens& l) : Lens(l.scale, l.distortion_params, l.distortion_center, l.stretch_matrix)
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

FisheyeTransformer::FisheyeTransformer(int height, int width) {
	fisheye_image_ = cv::Mat::zeros(height, width, CV_8UC4);
}

FisheyeTransformer::~FisheyeTransformer() {}

cv::Mat FisheyeTransformer::transformAndCombine(const FisheyeTransformer::TransformRequest& req)
{
	for (auto& image : req.src_images)
	{
		addToImage(image, image.camera_pose, req.projectionMatrix);
	}
	std::cout << "Transformation complete" << std::endl;
	return fisheye_image_;
}

void FisheyeTransformer::transformSingle(const SourceImage& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat)
{
	addToImage(src_img, src_pose, src_mat);
	std::cout << "Single Transformation complete" << std::endl;
}


void FisheyeTransformer::addToImage(const SourceImage& src_img, const Pose& src_pose, const ProjectionMatrix& src_mat)
{

	float focal_length = src_mat(1,1);
	float aspect_ratio = src_mat(0,0) / focal_length;
	int num_pixels = src_img.width * src_img.height;

	float h = static_cast<float>(src_img.height);
	float w = static_cast<float>(src_img.width);

	for (int i = 0; i < num_pixels; i++)
	{
		Pixel pixel = indexToPixel(i*4, src_img.width);
		ImageCoordinate feature;

		feature.x = (static_cast<float>(pixel.u) * 2.0f) / w - 1.0f;
		feature.x = feature.x * aspect_ratio;
		feature.y = (static_cast<float>(pixel.v) * 2.0f) / h - 1.0f;

		UnitSphereCoordinate vec = calculateSphereCoords(src_pose, aspect_ratio, focal_length, feature);
		Pixel p = getDestinationPixel(vec);

		fisheye_image_.at<uchar>(p.v, p.u) = src_img.image.at<uchar>(pixel.v, pixel.u);
		fisheye_image_.at<uchar>(p.v, p.u+1) = src_img.image.at<uchar>(pixel.v, pixel.u+2);
		fisheye_image_.at<uchar>(p.v, p.u+2) = src_img.image.at<uchar>(pixel.v, pixel.u+2);
		fisheye_image_.at<uchar>(p.v, p.u+3) = src_img.image.at<uchar>(pixel.v, pixel.u+3);
	}
}

UnitSphereCoordinate FisheyeTransformer::calculateSphereCoords(const Pose& pose, float aspect_ratio, float focal_length, ImageCoordinate feature) const {

	feature.x = feature.x / aspect_ratio;

	const Vector3f img_pos(feature.x, feature.y, focal_length);
	Vector3f world_pos = VectorMathf::transformToWorldFrame(img_pos, pose, true);
	world_pos.normalize();

	float theta = std::atan2(world_pos[1], world_pos[0]);
	float phi = std::acos(world_pos[2]);

	return {theta, phi};
}


Pixel FisheyeTransformer::indexToPixel(int index, int width) {

	return {(index/4)%width, (index/4)/width};
}

int FisheyeTransformer::pixelToIndex(Pixel p, int width) {
		return width * p.v + p.u;
	}

UnitSphereCoordinate FisheyeTransformer::pixelToUnitSphere(Pixel p)
{
	std::cout << "Getting unit sphere coords for {" << p.u << ", " << p.v << "}" <<std::endl;
	return {0, 0};
}

Pixel FisheyeTransformer::getDestinationPixel(UnitSphereCoordinate c)
{
	float r = std::sin(c.phi); //todo implement lens
	ImageCoordinate img_c;
	img_c.x = r * std::cos(c.theta);
	img_c.y = r * std::sin(c.theta);

	float imgw = static_cast<float>(fisheye_image_.size().width);
	float imgh = static_cast<float>(fisheye_image_.size().height);
	float u = imgw/2 * img_c.x + imgw/2;
	float v = imgh/2 * img_c.y + imgh/2;
	
	Pixel p;
	p.u = static_cast<int>(u);
	p.v = static_cast<int>(v);
	return {p.u, p.v};
}

}} //namespace msr::airlib end