/*
 * ni_camera.h
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#ifndef NI_CAMERA_H_
#define NI_CAMERA_H_

#include <OpenNI.h>
#include "hal_cam.h"

namespace cainiao_robot
{
/*
 * 3d camera using openni2
 */
class hal_ni_camera3d : public hal_cam
{
public:
    virtual ~hal_ni_camera3d();

    static hal_ni_camera3d *instance();

    virtual int init();
    virtual int deinit();
    virtual int open();
    virtual int close();

    int width() {   return m_depth_width;   }
    int height() {  return m_depth_height;  }

    int read(float *depth, float range);
    int read_raw(float *depth, char *color);

private:
    hal_ni_camera3d();
    int calcdepth(float *depth, float range);

    openni::VideoFrameRef       m_depth_frame;
    openni::VideoFrameRef       m_color_frame;
    openni::Device              m_device;
    openni::VideoStream         m_depth_stream;
    openni::VideoStream         m_color_stream;
    openni::VideoStream         **m_pp_stream;

    openni::VideoMode           m_depth_video_mode;
    openni::VideoMode           m_color_video_mode;

    int                         m_depth_width;
    int                         m_depth_height;
    int                         m_color_width;
    int                         m_color_height;
    float                      *m_histogram;

    static hal_ni_camera3d     *s_p_self;
    static   int               sc_max_depth;
};

} /* namespace cainiao_robot */
#endif /* NI_CAMERA_H_ */
