#include "fisheye.hpp"

#include <iostream>

namespace msr { namespace airlib {

typedef FisheyeTransformer::CameraPosition CameraPosition;
typedef Eigen::Vector3f Pixel;
typedef Eigen::Vector3f ImageCoordinate;
typedef Eigen::Matrix2f StretchMatrix;

Lens::Lens(float scale, float k1, float k2, float k3, float k4, Eigen::Vector2f dist_c, const StretchMatrix& stretch_m)
	: scale(scale), k1(k1), k2(k2), k3(k3), k4(k4), distortion_center(dist_c), stretch_matrix(stretch_m)
{}

Lens::Lens(float scale, float k1, float k2, float k3, float k4)
{
	Eigen::Vector2f dist_c;
	dist_c << 0,0;
	StretchMatrix stretch_m = StretchMatrix::Identity();
	Lens(scale, k1, k2, k3, k4, dist_c, stretch_m);
}

Lens::Lens(const Lens& l) : Lens(l.scale, l.k1, l.k2, l.k3, l.k4, l.distortion_center, l.stretch_matrix)
{}

Lens::Lens()
{
	Lens(1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
}

Lens::~Lens() {}

float Lens::distort(float phi) {
	return k1* phi; // +k2*pow(phi,2) + k3 * pow(phi,3) + k4 * pow(phi, 4);
}


FisheyeTransformer::SourceImage::SourceImage(cv::Mat img, CameraPosition position, int height_val, int width_val) {
	image = img;
	pos = position;
	height = height_val;
	width = width_val;
}

FisheyeTransformer::FisheyeTransformer(int height, int width, Lens lens) : lens_(lens) {
	fisheye_image_ = cv::Mat::zeros(height, width, CV_8UC4);

	PixelTransform << 	width/2.0f, 0, width/2.0f,
						0, height/2.0f, height/2.0f,
						0,0,1;

	InversePixelTransform = PixelTransform.inverse();
	makeCameraRotations();
}

FisheyeTransformer::~FisheyeTransformer() {}

cv::Mat& FisheyeTransformer::transformAndCombine(const std::vector<SourceImage>& req)
{
	for (auto& image : req)
	{
		addToImage(image);
	}
	std::cout << "Transformation complete" << std::endl;
	return fisheye_image_;
}

cv::Mat& FisheyeTransformer::transformSingle(const SourceImage& src_img)
{
	addToImage(src_img);
	std::cout << "Single Transformation complete" << std::endl;
	return fisheye_image_;
}

void FisheyeTransformer::addToImage(const SourceImage& src_img)
{

	int h = src_img.height;
	int w = src_img.width;

	Pixel p, p_fish;
	ImageCoordinate c, c_fish;
	UnitSphereCoordinate vec;
	float r;

	for (int u = 0; u < w; u++) {
		
		for (int v = 0; v < h; v++) {

			p << u , v, 1;
			c = InversePixelTransform*p;

			rotateToCameraFrame(c, src_img.pos);
			vec = calculateSphereCoords(c);
			
			r = lens_.distort(vec.phi);
			c_fish << r*std::cos(vec.theta) , r*std::sin(vec.theta);

			p_fish = PixelTransform*c_fish;

			fisheye_image_.at<uchar>(p_fish[0], p_fish[1]) = src_img.image.at<uchar>(p[0], p[1]);
			fisheye_image_.at<uchar>(p_fish[0]+1, p_fish[1]) = src_img.image.at<uchar>(p[0]+1, p[1]);
			fisheye_image_.at<uchar>(p_fish[0]+2, p_fish[1]) = src_img.image.at<uchar>(p[0]+2, p[1]);
			fisheye_image_.at<uchar>(p_fish[0]+3, p_fish[1]) = src_img.image.at<uchar>(p[0]+3, p[1]);
		}
	}
}

UnitSphereCoordinate FisheyeTransformer::calculateSphereCoords(ImageCoordinate coord) const {

	float theta = std::atan2(coord[1], coord[0]);
	float sqrt = std::sqrt(pow(coord[0],2) + pow(coord[1],2));
	float phi = std::atan2(sqrt,coord[2]);

	return {theta, phi};
}

void FisheyeTransformer::makeCameraRotations() {
	//Down rotation = no rotation
	Eigen::Quaternionf q(1,0,0,0);
	q.normalize();
	camera_rotations.push_back(q);

	//Front rotation = Rx(0)Ry(-90)
	Eigen::Quaternionf front(0.70710678118, 0, 0.70710678118, 0);
	q.normalize();
	camera_rotations.push_back(front);

	//Left rotation = Rx(90)Ry(-90)
	Eigen::Quaternionf left(0.5, 0.5, -0.5, 0.5);
	q.normalize();
	camera_rotations.push_back(left);

	//Back rotation = rx(180)Ry(-90)
	Eigen::Quaternionf back(0, 0.70710678118, 0, 0.70710678118);
	q.normalize();
	camera_rotations.push_back(back);

	//Right rotation = Rx(-90)Ry(-90)
	Eigen::Quaternionf right(0.5, -0.5, -0.5, -0.5);
	q.normalize();
	camera_rotations.push_back(right);
}

void FisheyeTransformer::rotateToCameraFrame(ImageCoordinate& c, CameraPosition pos) {
	Eigen::Quaternionf q;
	q.w() = 0;
	q.vec() = c;

	int p = static_cast<int>(pos);

	c = (camera_rotations.at(p) * q * camera_rotations.at(p).inverse()).vec();
}

}} //namespace msr::airlib end