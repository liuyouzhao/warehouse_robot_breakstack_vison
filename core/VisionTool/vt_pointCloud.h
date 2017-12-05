#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
namespace vt
{
    void multiFrames(std::vector< cv::Mat_<cv::Vec3f> >& frames, cv::Mat_<cv::Vec3f>& fusion, int th);
    void removePointsOnPlane(cv::Mat_<cv::Vec3f>& world, cv::Mat plane);
    void removePointsUnderPlane(cv::Mat_<cv::Vec3f>& world, cv::Mat plane);
    void estBoxBottom(cv::Mat_<cv::Vec3f>& world);
    cv::Mat estBottom(cv::Mat_<cv::Vec3f>& world, cv::Rect rt);
    void setBorder(cv::Mat_<cv::Vec3f>& world);

    cv::Mat estPlane(std::vector<cv::Vec3f>& pts);
    cv::Mat estPlane(cv::Mat& pts);
#ifdef UNIX
    void estimateNorm(cv::Mat_<cv::Vec3f>& mat, int wz, cv::Mat_<cv::Vec3f>& norm, cv::Mat& mask);
#else
    void estimateNorm(cv::Mat_<cv::Vec3f>& mat, int wz, cv::Mat_<cv::Vec3f>& norm, cv::Mat& mask = cv::Mat());
#endif
    bool estimateNorm(cv::Mat_<cv::Vec3f>& mat, cv::Rect rect, int wz, cv::Mat_<cv::Vec3f>& norm);
    std::vector<int> planeVerify(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat, std::vector<int>& ptOnPlane, float th=0.9);
    cv::Mat approxPlane(const cv::Mat& points);
    void regular(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat, std::vector<int>& pt);
    std::vector<int> normalCluster(cv::Mat_<cv::Vec3f>& world, cv::Mat_<cv::Vec3f>& normal, std::vector<int> obj, cv::Mat plane, float cos = 0.9, float dis = 3.);
    class Cube
    {
    public:
        Cube(){}
        bool init(cv::Mat* plane, cv::Mat_<cv::Vec3f>* world, cv::Mat_<cv::Vec3f>* normal, std::vector<int>* index, int num, float threshold, bool fix);
        void boards();
        void regularVerify(float* len);

        cv::Mat* plane;
        cv::Mat_<cv::Vec3f>* world, *normal;
        std::vector<int> index;
        std::vector<int> planeIdx[3];
        std::vector<int> objIdx, regularIdx[3];
        cv::Mat regularPlane[3], R[3];

        int num;
        float threshold;
        bool fix;
    };

}
