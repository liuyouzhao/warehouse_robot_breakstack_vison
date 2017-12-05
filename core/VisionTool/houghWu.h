#include "opencv2/core/core.hpp"

void HoughLinesWu(cv::InputArray _image, cv::OutputArray _lines, std::vector<int>& score,
    double rho, double theta, int threshold);
