#pragma once
#include "opencv2/core/core.hpp"
class Coordinate
{
public:
	Coordinate(){}
	Coordinate(float* _plane);
	~Coordinate();
	cv::Size_<float> project(float* points, int len);
	void set(float* _plane);
	//cv::Mat plane;
	cv::Mat R;
	cv::Mat t;
	cv::Mat image;
	float distance(float* plane, std::vector<cv::Vec3f> points);
private:
	double inline norm(float* vec, int len);
	cv::Size_<float> getSize(cv::Mat image, std::vector<cv::Point> points);
	cv::Size_<float> shrink(cv::Mat image, cv::RotatedRect rRect);

};

