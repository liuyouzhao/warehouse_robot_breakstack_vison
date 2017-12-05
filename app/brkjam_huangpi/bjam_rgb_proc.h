#ifndef BJAM_RGB_PROC_H
#define BJAM_RGB_PROC_H

#include "opencv2/opencv.hpp"

typedef struct rgb_box_s
{
    float x;
    float y;
    float z;
    float box_width;
    float box_height;
} rgb_box_t;

class bjam_rgb_proc
{
public:
    /**
     * @brief process
     * process to get verticle or horizontal
     * @return
     * 0 ==> verticle
     * 1 ==> horizontal
     */
    static int run( cv::Mat depth,
                    cv::Mat color,
                    cv::Mat binary_image_segmented,
                    cv::Mat world1,
                    cv::Rect rt,
                    cv::Point &px,
                    cv::RotatedRect &rotated_rect_target );

    static cv::Mat rotate_Img( cv::Mat img, float angle, cv::Mat& map_matrix );

    static cv::Point2f trans_point( cv::Point2f p, cv::Mat matrix );

    static cv::Point2f trans_point_back( cv::Point2f p, cv::Mat matrix );

    static inline cv::Mat dist() {  return s_distance_result_image; }

    static inline cv::Mat rgb_image()   {   return s_rgb_image; }

    static inline cv::Mat gui() {   return s_color_gui; }
protected:
    bjam_rgb_proc();

private:
    static cv::Mat             s_distance_result_image;
    static cv::Mat             s_rgb_image;

    static cv::Mat             s_color_gui;
};



#endif // BJAM_RGB_PROC_H
