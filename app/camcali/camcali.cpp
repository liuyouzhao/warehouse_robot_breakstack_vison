#include "vt_io.h"
#include "vt_visual.h"
#include "vt_pointCloud.h"
#include "opencv2/opencv.hpp"
#include "vt_depthTrans.h"
#include <time.h>
#include <algorithm>
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "CameraPara.h"
#include "hal/hal_ni_camera3d.h"
#include "sys/sys_types.h"
#include <pthread.h>

using namespace vt;
using namespace std;
using namespace cv;
using namespace cainiao_robot;

#define _isnan std::isnan

/* Camere related varients */
static int              s_data_width = 0;
static int              s_data_height = 0;
static int              s_data_bytes_siz = 0;
static float            s_data_value_range = 256.f;
static float           *s_p_depth = 0;
static unsigned char  *s_pcbuf = 0;

/* Algorithm related varients */
static int                             s_max_world_cache = 8;
static cv::Mat                         s_box_bottom;
static std::vector< Mat_<Vec3f> >      s_front_worlds;
static std::vector< Mat_<Vec3f> >      s_back_worlds;
static int                             s_world_width = 0;
static int                             s_world_height = 0;
static int                             s_merge_threhold = 10;
static cv::Mat                         s_intri[6];

static int init_hal()
{
    /*(1) Open hal of camera3d*/
    int rc = hal_ni_camera3d::instance()->init();
    if(rc != HAL_STATUS_OK)
    {
        printf("[%s:%d %s]\nFataError! hal_ni_camera3d init failed\n",
                __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    rc = hal_ni_camera3d::instance()->open();

    if(rc != HAL_STATUS_OK)
    {
        printf("[%s:%d %s]\nFataError! hal_ni_camera3d open failed\n",
                __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    s_data_width = hal_ni_camera3d::instance()->width();
    s_data_height = hal_ni_camera3d::instance()->height();

    s_data_bytes_siz = sizeof(float) * s_data_width * s_data_height;
    s_p_depth = (float*) malloc(s_data_bytes_siz);
    s_pcbuf = (unsigned char*) malloc(s_data_width * s_data_height);
    memset(s_pcbuf, 0, s_data_width * s_data_height);

    return 0;
}

static int init_alg_configs()
{
    float bottom[3];

    bool ok = loadBinary("bottom", bottom);
    if(!ok)
    {
        return -1;
    }
    s_box_bottom = Mat(3, 1, CV_32F, bottom);

    t3d::loadIntri("intrinsics.yml", s_intri);
    return 0;
}

static void show_preview_image(Mat_<Vec3f> world)
{
    Mat_<short> depth(world.size(), CV_16S);
    for (int i = 0; i < world.total(); i++)
    {
        float& x = world(i)(2);
        x = x > 255.0f ? 255 : x;
        x = x < 0.0f ? 0.0f : x;
        if (_isnan(x))
            depth(i) = -1000;
        else
            depth(i) = x;
    }
    imshow("show", toGray(depth));
}

static Mat_<Vec3f> depth_data_loading(int width, int height, float *buf, int size)
{
    Mat_<Vec3f> world(height, width);
    Mat_<float> depth(height, width);
    memcpy(depth.data, buf, size * sizeof(float));

    vt::depth2World(depth, world, (float*)s_intri[0].data, (float*)s_intri[4].data);

    if(s_back_worlds.size() >= s_max_world_cache)
    {
        s_back_worlds.erase(s_back_worlds.begin());
    }
    s_back_worlds.push_back(world);

    s_world_width = width;
    s_world_height = height;

    return world;
}

static int run()
{
    int rc = 0;
    Mat_<Vec3f> world;
    rc = hal_ni_camera3d::instance()->read(s_p_depth, s_data_value_range);
    if (rc == HAL_STATUS_OK)
    {
        world = depth_data_loading(s_data_width, s_data_height, s_p_depth, s_data_width*s_data_height);

        /* Show what we have captured */
        show_preview_image(world);

        saveBinary("captured", world.data, world.total() * sizeof(float) * 3);
        Mat plane = estBottom(world, Rect(160, 175, 290, 175));
        saveBinary("bottom_new", plane.data, 12);
        cout << "plane: " << plane;

        printf("\nCalibration Finished! Press ESC to exit.\n");
        usleep(1000 * 10);
        char c = cvWaitKey(33);
        if(c == 27) exit(0);
    }
    else
    {
        printf("[%s:%d %s]\nFataError! hal_ni_camera3d return error.\n",
                    __FILE__, __LINE__, __FUNCTION__);
        exit(1);
    }
}


int main(int argc, char **argv)
{
    init_hal();
    init_alg_configs();
    run();
    return 0;
}
