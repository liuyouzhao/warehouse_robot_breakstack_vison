#ifndef BJAM_DEPTH_PROC_H
#define BJAM_DEPTH_PROC_H

#include <vector>
#include "bjam_state.h"

class PlaneFit2;

#define BDPS bjam_depth_proc::singleton()

/**
 * @brief The bjam_depth_proc class
 *
 * Main segmentation processing class.
 * Global lifecycle, singleton pointer.
 */
class bjam_depth_proc
{
public:
    ~bjam_depth_proc();
    static bjam_depth_proc *singleton();

    /**
     * @brief inform_color_depth_map
     * Set RGB/Depth image, update maps
     * @param color
     * @return
     */
    int inform_color_depth_map(cv::Mat color, cv::Mat depth);

    /**
     * @brief run
     * Main process
     * @return
     */
    int run();

    int recalculate(std::vector<cv::Point> points);

    /**
     * Setters, Getters
     */
    inline void set_sku_size(int w, int h) {    m_sku_siz[0] = w; m_sku_siz[1] = h; }
    inline int screen_width() { return m_screen_width;  }
    inline int screen_height() { return m_screen_height;  }
    inline int has_damn_right() { return m_damn_right;  }
    inline float length_scale() {   return m_length_scale;  }
    inline cv::Mat result_image() { return m_result_image;  }
    inline cv::Mat_< cv::Vec3f > world()  { return m_world; }
    inline cv::Mat_< cv::Vec3f > world1()  { return m_world1; }
    inline cv::Rect region() {  return m_target_region; }
    inline std::vector<float> vpts() {    return m_p_ns->pickPoint; }
    inline PlaneFit2*   ns() {  return m_p_ns;  }
    inline cv::Mat* intri(){return m_intri;}
protected:
    bjam_depth_proc();

    /**
     * @brief s_p_self
     * Global singleton
     */
    static bjam_depth_proc        *s_p_self;

private:
    /**
     * @brief m_world
     * Current world vec data, point cloud
     *
     * @brief m_world1
     * temporary world value, full rect
     */
    cv::Mat_< cv::Vec3f >         m_world;
    cv::Mat_< cv::Vec3f >         m_world1;

    /**
     * @brief m_color
     * RGB image
     */
    cv::Mat                       m_color;
    cv::Mat                       m_rt_color;
    cv::Mat                       m_depth;

    /**
     * @brief m_target_region
     * Target rectangle on the table, later process will go inside the rect.
     */
    cv::Rect                      m_target_region;

    /**
     * @brief m_screen_width&&height
     * 640 X 480 ? maybe
     */
    int                           m_screen_width;
    int                           m_screen_height;

    cv::Mat                       m_camera;
    float                         m_fxy;

    cv::Mat                       m_intri[6];
    cv::Mat                       m_l2r;

    int                           m_sku_siz[2];

    cv::Mat                       m_result_image;
    std::string                   m_info_result;

    /**
     * @brief m_damn_right
     * Mark if PlaneFit2 found a single plane
     */
    int                           m_damn_right;

    /**
     * @brief m_lengthScale
     * size counts in mm(millian-meter)
     * convert mm to pixel:
     * pixel = size / m_lengthScale
     */
    float                         m_length_scale;

    /**
     * @brief m_p_ns
     * Main segmentation pointer
     */
    PlaneFit2                     *m_p_ns;
};

#endif // BJAM_DEPTH_PROC_H
