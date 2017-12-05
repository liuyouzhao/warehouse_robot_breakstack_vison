#pragma once
#include "opencv2/core/core.hpp"

namespace t3d
{
	void loadAstraParams(std::string path, cv::Mat& L2R, cv::Mat intri[4]);
    void loadAstraParamsD(std::string path, cv::Mat& L2R, cv::Mat intri[4]);
	bool readStringList(const std::string& filename, std::vector<std::string>& list);
	void loadIntri(std::string intriPath, cv::Mat intri[6]);
    void loadIntriD(std::string intriPath, cv::Mat intri[6]);
	void checkCornerList(std::vector<cv::Point2f>& corners);
}
