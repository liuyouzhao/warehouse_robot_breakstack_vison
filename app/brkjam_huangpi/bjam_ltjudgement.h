#ifndef BJAM_LTJUDGEMENT_H
#define BJAM_LTJUDGEMENT_H

#include "bjam_state.h"

class bjam_ltjudgement
{
public:

    static int init_params(float ls, cv::Mat target_image, sku_attribute_t sku);

    /**
     * @brief process
     * process to get verticle or horizontal
     * @return
     * 0 ==> verticle
     * 1 ==> horizontal
     */
    static void run(int &type, int debug = 0);

    static inline cv::Mat get_target_image()   {   return s_target_image;  }

    static inline int left()    {   return s_left;  }
    static inline int top()    {   return s_top;  }

    static inline float scale() {   return s_length_scale;  }
    static inline float confidence() {  return s_confidence;    }

    static cv::Rect get_first_rect() {  return cv::Rect(s_focus_x,
                                                        s_focus_y,
                                                        s_focus_width,
                                                        s_focus_height); };
    /**
     * @brief s_focus_x, s_focus_y
     * Left top focus point, the first RECT corner
     */
    static int                 s_focus_x;
    static int                 s_focus_y;
    static int                 s_focus_width;
    static int                 s_focus_height;

protected:
    bjam_ltjudgement();
private:
    static float               s_length_scale;
    static cv::Mat             s_target_image;

    static int                 s_left;
    static int                 s_top;
    static int                 s_right;
    static int                 s_bottom;

    static float               s_confidence;

    static int                 s_template_pix_width;
    static int                 s_template_pix_height;
    static sku_attribute_t     s_current_sku;
};

#endif // BJAM_LTJUDGEMENT_H
