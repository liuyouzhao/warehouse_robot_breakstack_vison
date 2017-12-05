#pragma once
#include "type.h"

namespace vt
{   
    VISIONTOOL_API cv::Mat toGray(const cv::Mat& src, int minBorder = 0, int maxBorder = 0, const int back = 50);

    //  Show  disparity  map  in  pseudo  color    
    VISIONTOOL_API cv::Mat pseudoColor(const cv::Mat& src);

}