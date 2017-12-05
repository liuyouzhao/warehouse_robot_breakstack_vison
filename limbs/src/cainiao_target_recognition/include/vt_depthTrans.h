#pragma once
#include "type.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
namespace vt
{
	/*int loadimgdata(const char *pimgdatapath, char **pimgdata);

	bool saveRaw(const char *rawName, const char *imgBuf, int width, int height, int pixBytes);*/
	void mappingDepth2Color(cv::Mat &src, cv::Mat &dst, const float *mat);
	void astraDepth2Color(cv::Mat &src, cv::Mat &dst, const float *mat);
	void framesFusion(std::vector<cv::Mat>& frames, cv::Mat& fusion, int th=3);
    void framesAverage(std::vector<cv::Mat>& frames, cv::Mat& fusion);
	void irMaxRemove(unsigned short* pIr, int resolution, cv::Mat& depthMat);
    void edgeRemove(cv::Mat& depth, int neighbor = 1);
	template <class T>
    VISIONTOOL_API void depth2World(cv::Mat_<T> depth, cv::Mat_<cv::Vec3f> worldMat, float* camera, float* k);

}




