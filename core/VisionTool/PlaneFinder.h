#pragma once
#include <vector>
#include <list>
#include <deque>
#include <utility>
#include "opencv2/core/core.hpp"
class PlaneFinder
{
public:
	PlaneFinder();
	~PlaneFinder();
	std::vector<cv::Vec3f> remainpoint;
	std::deque<cv::Mat> m_planes;
	std::deque< std::vector<cv::Vec3f> > m_pointList;
	cv::Mat m_points;
	std::vector<cv::Vec3f> m_cloud;
	std::vector< std::pair< cv::Mat, std::vector<cv::Vec3f> > > m_plList;


	bool setPoints(const cv::Mat& cloud, cv::Rect window, float near, float far);
	static bool comp(std::pair<int, cv::Mat> i, std::pair<int, cv::Mat> j);
	static cv::Mat approxPlane(const cv::Mat& points);
	void ransacPlane(int num=1000, int k = 10, float threshold = 10, int nearby = 400);
	void ransacPlane2(int num = 1000, int k = 10, float threshold = 10, int nearby = 400);
	static bool isPlane(cv::Mat& plane, cv::Mat& ptOnPlane, cv::Mat& points, int num, float threshold);
	static bool isPlane(cv::Mat& plane, std::vector<cv::Vec3f>& ptOnPlane, std::vector<cv::Vec3f>& points, int num, float threshold);
	static bool setTable(cv::Mat& plane, std::vector<cv::Vec3f>& ptOnPlane, std::vector<cv::Vec3f>& points, int num, float threshold);
	static void removePoints(cv::Mat& plane, std::vector<cv::Vec3f>& ptOnPlane, cv::Mat& plane2, float threshold);
	static void removePoints(cv::Mat_<cv::Vec3f>& mat, cv::Mat& plane, std::vector<int>& ptOnPlane, cv::Mat& plane2, float threshold);

	static bool setTable(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& ptList, std::vector<cv::Vec3f>& points, std::vector<int>& ptOnPlane,
		std::vector<cv::Vec3f>& ptsPlane, int num, float threshold, bool fix = false);
	static bool setTableTemp(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& points, std::vector<int>& ptOnPlane, int num, float threshold);
	static bool setTableFix
        (cv::Mat& plane, std::vector<cv::Vec3f>& ptOnPlane, std::vector<cv::Vec3f>& points, int num, float threshold);

	static float distance(float* point, float* plane, float len);
	void project(float* point, float* plane, float len, float* prjPoint);

	static void planeEva(cv::Mat mat, cv::Mat plane, std::vector<int> ptList, std::vector<cv::Vec3f> pts);
    static std::vector<int> planeVerify(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& ptOnPlane);
    static bool getNorm(cv::Mat_<cv::Vec3f>& mat, int idx, cv::Vec3f& norm);
    static bool estimateNorm(cv::Mat_<cv::Vec3f>& mat, int sz, cv::Mat_<cv::Vec3f>&  norm);
    static void planeVerify2(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat);
};

