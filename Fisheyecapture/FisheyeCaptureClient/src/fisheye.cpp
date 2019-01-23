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

Lens::Lens(float scale, float k1, float k2, float k3, float k4) : scale(scale), k1(k1), k2(k2), k3(k3), k4(k4)
{
	distortion_center << 0,0;
	stretch_matrix = StretchMatrix::Identity();
}

Lens::Lens(const Lens& l) : Lens(l.scale, l.k1, l.k2, l.k3, l.k4, l.distortion_center, l.stretch_matrix)
{}

Lens::Lens() : Lens(1.0f, 0.5f, 0.0f, 0.0f, 0.0f)
{}

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

FisheyeTransformer::FisheyeTransformer(int dest_height, int dest_width, int src_height, int src_width, Lens lens) : lens_(lens) {
	fisheye_image_ = cv::Mat::zeros(dest_height, dest_width, CV_8UC4);

	InversePixelTransform << 	src_width/2.0f, 0, src_width/2.0f,
								0, src_height/2.0f, src_height/2.0f,
								0,0,1;

	InversePixelTransform = InversePixelTransform.inverse().eval();

	float max = static_cast<float>(calculatePhiMax(lens));

	PixelTransform << 	dest_width/(2.0*max), 0, dest_width/2.0,
						0, dest_height/(2.0*max), dest_height/2.0,
						0, 0, 1;


	std::cout << "INV: " << InversePixelTransform << std::endl;
	std::cout << "TRANS: " << PixelTransform << std::endl;
	std::cout << "MAX: " << max << std::endl;
	std::cout << "DIM(w/h): " << src_width << ", " << src_height << std::endl;

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

	for (int u = 0; u < w; u++) {
		
		for (int v = 0; v < h; v++) {

			Pixel p, p_fish;
			p << u , v, 1;
			ImageCoordinate c = InversePixelTransform*p;

			rotateToCameraFrame(&c, src_img.pos);

			UnitSphereCoordinate vec = calculateSphereCoords(c);
			
			float r = lens_.distort(vec.phi);

			ImageCoordinate c_fish;
			c_fish << r*std::cos(vec.theta) , r*std::sin(vec.theta), 1;

			p_fish = PixelTransform*c_fish;

			int final_u = static_cast<int>(p_fish.coeff(0));
			int final_v = static_cast<int>(p_fish.coeff(1));

			fisheye_image_.at<uchar>(final_u, final_v) = src_img.image.at<uchar>(u, v);
		}
	}
}

double FisheyeTransformer::calculatePhiMax(const Lens& lens) const {

	// Calculate in small parts
	double phi_max = 270.0*M_PI/180.0;

	double max = lens.k1 * phi_max;

	double temp = lens.k2*phi_max*phi_max;
	max += temp;

	temp = lens.k3*phi_max*phi_max;
	temp = temp*phi_max;
	max += temp;

	temp = lens.k4*phi_max*phi_max;
	temp = temp*phi_max;
	temp = temp*phi_max;
	max += temp;

	return max;
}

UnitSphereCoordinate FisheyeTransformer::calculateSphereCoords(const ImageCoordinate& coord) const {

	float theta = std::atan2(coord.coeff(1), coord.coeff(0));
	float sqrt = std::sqrt(pow(coord.coeff(0),2) + pow(coord.coeff(1),2));
	float phi = std::atan2(sqrt, coord.coeff(2));

	return {theta, phi};
}

void FisheyeTransformer::makeCameraRotations() {
	//Down rotation = no rotation
	Eigen::Quaternionf q(1,0,0,0);
	q.normalize();
	camera_rotations.push_back(q);

	//Front rotation = Rx(90)Ry(0)
	Eigen::Quaternionf front(0.70710678118, 0.70710678118, 0, 0);
	front.normalize();
	camera_rotations.push_back(front);

	//Left rotation = Rx(90)Ry(-90)
	Eigen::Quaternionf left(0.5, 0.5, -0.5, -0.5);
	left.normalize();
	camera_rotations.push_back(left);

	//Back rotation = rx(90)Ry(180)
	Eigen::Quaternionf back(0, 0, 0.70710678118, 0.70710678118);
	back.normalize();
	camera_rotations.push_back(back);

	//Right rotation = Rx(90)Ry(90)
	Eigen::Quaternionf right(0.5, 0.5, 0.5, 0.5);
	right.normalize();
	camera_rotations.push_back(right);
}

void FisheyeTransformer::rotateToCameraFrame(ImageCoordinate* c, CameraPosition pos) {
	Eigen::Quaternionf q;
	q.w() = 0;
	q.vec() = *c;

	int p = static_cast<int>(pos);

	Eigen::Quaternionf q_rotated = (camera_rotations.at(p) * q * camera_rotations.at(p).inverse());
	*c = q_rotated.vec();
}

}} //namespace msr::airlib end