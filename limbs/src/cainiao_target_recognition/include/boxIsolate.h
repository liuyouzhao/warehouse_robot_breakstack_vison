#pragma once
#include "opencv2/core/core.hpp"

class BoxIsolate
{
public:
    BoxIsolate(cv::Mat& depth, int width, int height, int top, int bottom);
    BoxIsolate(cv::Mat& world);
    void init(int width, int height, int top, int bottom);
    void run();
    void filterByRange(float* cx, float* cy, float* theta);


    ~BoxIsolate();

    float findCenterByProjection(float* val, int n, int length);
    void pointsInRange(float* hp, float*vp, std::vector<cv::Point>& candidates);
    void pointsInRange(std::vector<cv::Point>& candidates);
    float rotationTest(std::vector<cv::Point>& candidates, int cx, int cy);
    cv::Mat depth, world;
    int width, height, top, bottom;
    int boxWidth, boxHeight;
    float *vp, *hp;/* cx, cy, theta;*/
    const float pi;
    //std::vector<cv::Point> candidates;
};
void findBox(cv::Mat& depth, int width, int height, int top, int bottom);

namespace vt
{
    float findCenter(float* val, int range, int length);
    cv::Rect boxEst(float* pts, float* plane, int imgWidth, int imgHeight, int height, int error,
        int boxWidth, int boxHeight,std::vector<cv::Mat>& wall, int borderX = 45, int borderY = 15);
}
