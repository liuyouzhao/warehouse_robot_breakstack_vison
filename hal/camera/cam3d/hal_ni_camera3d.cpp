/*
 * ni_camera.cpp
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#include "hal_ni_camera3d.h"
#include "hal.h"

namespace cainiao_robot
{
#define MAX_DEPTH 10000
#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480

hal_ni_camera3d     *hal_ni_camera3d::s_p_self;
int               hal_ni_camera3d::sc_max_depth;

static void calculate_histogram(float* pHistogram,
                                    int histogramSize,
                                    const openni::DepthPixel* depth,
                                    const openni::VideoFrameRef& frame,
                                    float range);

hal_ni_camera3d::hal_ni_camera3d()
{
    m_product_id = "ni_camera";
    m_interface_type = "usb";
    hal_ni_camera3d::sc_max_depth = MAX_DEPTH;
    m_histogram = (float*)malloc(sc_max_depth * sizeof(float));
}

hal_ni_camera3d::~hal_ni_camera3d()
{
    hal_ni_camera3d::s_p_self = 0;
    free(m_histogram);
}

hal_ni_camera3d *hal_ni_camera3d::instance()
{
    static hal_ni_camera3d *p = new hal_ni_camera3d();
    hal_ni_camera3d::s_p_self = p;
    return hal_ni_camera3d::s_p_self;
}

int hal_ni_camera3d::init()
{
    openni::Status rc = openni::STATUS_OK;

    rc = openni::OpenNI::initialize();
    printf("initialization:\n%s\n", openni::OpenNI::getExtendedError());

    const char* deviceURI = openni::ANY_DEVICE;
    rc = m_device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("hal_ni_camera3d: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }

    rc = m_depth_stream.create(m_device, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK)
    {
        printf("hal_ni_camera3d: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
        return HAL_STATUS_FAIL;
    }

    rc = m_color_stream.create(m_device, openni::SENSOR_COLOR);
    if (rc != openni::STATUS_OK)
    {
        printf("hal_ni_camera3d: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
        return HAL_STATUS_FAIL;
    }

    m_inited = HAL_ACTIVE;

    return HAL_STATUS_OK;
}

int hal_ni_camera3d::deinit()
{
    openni::OpenNI::shutdown();
    m_inited = HAL_NEGATIVE;
}

int hal_ni_camera3d::open()
{
    openni::Status rc = openni::STATUS_OK;

    m_depth_video_mode = m_depth_stream.getVideoMode();
    m_color_video_mode = m_color_stream.getVideoMode();

    /* set resolution */
    //m_depth_video_mode.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    //m_color_video_mode.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT);

    //m_depth_stream.setVideoMode(m_depth_video_mode);
    //m_color_stream.setVideoMode(m_color_video_mode);

    rc = m_depth_stream.start();
    if (rc != openni::STATUS_OK)
    {
        m_depth_stream.destroy();
        printf("[%s] Couldn't start depth stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        return HAL_STATUS_FAIL;
    }

    rc = m_color_stream.start();
    if (rc != openni::STATUS_OK)
    {
        m_color_stream.destroy();
        printf("[%s] Couldn't start color stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        return HAL_STATUS_FAIL;
    }

    if (!m_depth_stream.isValid() || !m_color_stream.isValid())
    {
        printf("[%s] No valid streams. Exiting\n", __FUNCTION__);
        openni::OpenNI::shutdown();
        return HAL_STATUS_FAIL;
    }

    m_depth_width = m_depth_video_mode.getResolutionX();
    m_depth_height = m_depth_video_mode.getResolutionY();
    m_color_width = m_color_video_mode.getResolutionX();
    m_color_height = m_color_video_mode.getResolutionY();

    m_pp_stream = new openni::VideoStream*[2];

    m_pp_stream[0] = &m_depth_stream;
    m_pp_stream[1] = &m_color_stream;

    return HAL_STATUS_OK;
}

int hal_ni_camera3d::close()
{
    m_depth_stream.destroy();
    m_color_stream.destroy();

    if(m_pp_stream)
    {
        delete[] m_pp_stream;
        m_pp_stream = 0;
    }
}

int hal_ni_camera3d::read(float *depth, float range)
{
    int changedIndex;
    openni::Status rc = openni::OpenNI::waitForAnyStream(m_pp_stream, 2, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        printf("%s Wait failed\n", __FUNCTION__);
        return HAL_STATUS_FAIL;
    }
    switch (changedIndex)
    {
    case 0:
        rc = m_depth_stream.readFrame(&m_depth_frame);
        break;
    case 1:
        rc = m_color_stream.readFrame(&m_color_frame);
        break;
    default:
        printf("%s Error in wait\n", __FUNCTION__);
    }

    if (rc != openni::STATUS_OK)
    {
        printf("%s Error in readFrame\n", __FUNCTION__);
    }

    calcdepth(depth, range);

    return rc;
}

int hal_ni_camera3d::read_raw(float *depth, char *color)
{
    int changedIndex;
    int i = 0, j = 0;
    int w = width();
    int h = height();

    const openni::DepthPixel* darr = NULL;
    const openni::RGB888Pixel* carr = NULL;

    openni::Status rc = openni::OpenNI::waitForAnyStream(m_pp_stream, 1, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        printf("%s Wait failed\n", __FUNCTION__);
        return HAL_STATUS_FAIL;
    }

    while(darr == NULL && carr == NULL)
    {
        switch (changedIndex)
        {
        case 0:
            rc = m_depth_stream.readFrame(&m_depth_frame);
            darr = (const openni::DepthPixel*)m_depth_frame.getData();
            if(darr != NULL)
            {
                openni::Status rc = openni::OpenNI::waitForAnyStream(m_pp_stream, 2, &changedIndex);
                if(changedIndex == 1)
                {
                    rc = m_color_stream.readFrame(&m_color_frame);
                    carr = (const openni::RGB888Pixel*)m_color_frame.getData();
                }
            }
            break;
//        case 1:
//            rc = m_color_stream.readFrame(&m_color_frame);
//            carr = (const openni::RGB888Pixel*)m_color_frame.getData();
//            break;
        default:
            printf("%s Error in wait\n", __FUNCTION__);
        }
        if (rc != openni::STATUS_OK)
        {
            printf("%s Error in readFrame\n", __FUNCTION__);
        }
    }


    // depth obtain value
    for ( i = 0; i < h; i ++) {
        for ( j = 0; j < w; j ++) {
            if(darr)
                depth[i * w + j] = (float) darr[i * w + (w - j)];
            if(carr) {
                color[(i * w + j) * 3 + 0] = carr[i * w + (w - j)].b;
                color[(i * w + j) * 3 + 1] = carr[i * w + (w - j)].g;
                color[(i * w + j) * 3 + 2] = carr[i * w + (w - j)].r;
            }
        }
    }
    return rc;
}

int hal_ni_camera3d::calcdepth(float *depth, float range)
{
    int w = width();
    int h = height();

    if( w != m_depth_frame.getWidth() || h != m_depth_frame.getHeight() )
    {
        printf("ERROR w h %d %d %d %d\n", m_depth_frame.getWidth(), m_depth_frame.getHeight(), w, h);
    }


    if (!m_depth_frame.isValid())
        return -1;

    const openni::DepthPixel* darr = (const openni::DepthPixel*)m_depth_frame.getData();


    calculate_histogram(m_histogram, sc_max_depth, darr, m_depth_frame, range);

    int i = 0, j = 0;
    for ( i = 0; i < h; i ++) {

        for ( j = 0; j < w; j ++) {
            depth[i * w + j] = m_histogram[darr[i * w + j]];
        }
    }
}

static void calculate_histogram(float* pHistogram,
                                    int histogramSize,
                                    const openni::DepthPixel* depth,
                                    const openni::VideoFrameRef& frame,
                                    float range)
{
    const openni::DepthPixel* pDepth = depth;
    // Calculate the accumulative histogram (the yellow display...)
    memset(pHistogram, 0, histogramSize*sizeof(float));
    int restOfRow = frame.getStrideInBytes() / sizeof(openni::DepthPixel) - frame.getWidth();
    int height = frame.getHeight();
    int width = frame.getWidth();

    unsigned int nNumberOfPoints = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x, ++pDepth)
        {
            if (*pDepth != 0)
            {
                pHistogram[*pDepth]++;
                nNumberOfPoints++;
            }
        }
        pDepth += restOfRow;
    }
    for (int nIndex=1; nIndex<histogramSize; nIndex++)
    {
        pHistogram[nIndex] += pHistogram[nIndex-1];
    }
    if (nNumberOfPoints)
    {
        for (int nIndex=1; nIndex<histogramSize; nIndex++)
        {
            pHistogram[nIndex] = (range * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
        }
    }
}

} /* namespace cainiao_robot */
