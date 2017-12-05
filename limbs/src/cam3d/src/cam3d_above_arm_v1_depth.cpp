/* A simple program to show how to set up an X window for OpenGL rendering.
 * X86 compilation: gcc -o -L/usr/X11/lib   main main.c -lGL -lX11
 * X64 compilation: gcc -o -L/usr/X11/lib64 main main.c -lGL -lX11
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hal/hal_ni_camera3d.h"
#include "sys/sys_ros_node_strmsg.h"
#include "sys/sys_ros_node_depth.h"

using namespace cainiao_robot;

#define __NODE_NAME__ "cam3d_above_arm_v1_depth"
#define __CHANNEL_VIEW__ "viewer_cloud3d_listener"
#define __CHANNEL_VIEW_DEPTH__ "/camera/depth_registered/depth"
#define __CHANNEL_CAM3D__ "cam3d_above_arm_v1_listener"
#define __CHANNEL_TO_CRBLLM__ "crbllm_pointcloud_filter_pcl_listener"
#define MAX_MSG_LEN 1024

static int              s_cam3d_need_send_width = 1;
static int              s_data_width = 0;
static int              s_data_height = 0;
static int              s_data_bytes_siz = 0;
static float            s_data_value_range = 256.f;
static char             s_buf_sync_w_h[32] = {0};
static float           *s_p_depth = 0;
static char            *s_p_color = 0;
static unsigned char  *s_pcbuf = 0;

static int init_hal()
{
    /*(1) Open hal of camera3d*/
    int rc = hal_ni_camera3d::instance()->init();
    if(rc != HAL_STATUS_OK)
    {
        SYS_LOGE("[%s:%d %s]\nFataError! hal_ni_camera3d init failed\n",
                __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    rc = hal_ni_camera3d::instance()->open();

    if(rc != HAL_STATUS_OK)
    {
        SYS_LOGE("[%s:%d %s]\nFataError! hal_ni_camera3d open failed\n",
                __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    s_data_width = hal_ni_camera3d::instance()->width();
    s_data_height = hal_ni_camera3d::instance()->height();

    s_data_bytes_siz = sizeof(float) * s_data_width * s_data_height;
    s_p_depth = (float*) malloc(s_data_bytes_siz);
    s_p_color = (char*) malloc(s_data_width * s_data_height);
    s_pcbuf = (unsigned char*) malloc(s_data_width * s_data_height);
    memset(s_pcbuf, 0, s_data_width * s_data_height);

    return 0;
}

static void on_ready_to_publish_depth()
{
    while (1)
    {
        int rc = hal_ni_camera3d::instance()->read_raw(s_p_depth, s_p_color);
        if (rc == HAL_STATUS_OK)
        {
            msg_depth_t mdt;
            mdt.depth = s_p_depth;
            mdt.width = s_data_width;
            mdt.height = s_data_height;
            sys_ros_node_depth::instance()->publish(__CHANNEL_VIEW_DEPTH__,
                                                    &mdt,
                                                    s_data_width * s_data_height);

            sys_ros_node_depth::instance()->publish(__CHANNEL_TO_CRBLLM__,
                                                    &mdt,
                                                    s_data_width * s_data_height);
        }
        else
        {
            SYS_LOGE("[%s:%d %s]\nFataError! hal_ni_camera3d return error.\n",
                        __FILE__, __LINE__, __FUNCTION__);
            exit(1);
        }
        usleep(1000 * 10);
    }
}

void empty_callback() {}

int main(int argc, char **argv)
{
    pthread_t thread;
    int rc = 0;

    /* System init */
    SYS_ROS_GLOBAL_ENV_INIT(argc, argv, (char*)__NODE_NAME__)

    rc = init_hal();
    if(rc != 0)
    {
        return -1;
    }

    sys_ros_node_depth::instance()->add_publisher(__CHANNEL_VIEW_DEPTH__, MAX_MSG_LEN);
    sys_ros_node_depth::instance()->add_publisher(__CHANNEL_TO_CRBLLM__, MAX_MSG_LEN);
    sys_ros_node_depth::instance()->set_on_ready_to_publish(on_ready_to_publish_depth);

    sys_ros_node_depth::instance()->run();

    SYS_LOGI("%s will start\n", __NODE_NAME__);
    sys_ros_node::sys_run();

    return 0;
}
