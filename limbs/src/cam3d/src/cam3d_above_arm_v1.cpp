/* A simple program to show how to set up an X window for OpenGL rendering.
 * X86 compilation: gcc -o -L/usr/X11/lib   main main.c -lGL -lX11
 * X64 compilation: gcc -o -L/usr/X11/lib64 main main.c -lGL -lX11
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hal/hal_ni_camera3d.h"
#include "sys/sys_ros_node_strmsg.h"

using namespace cainiao_robot;

#define __NODE_NAME__ "cam3d_above_arm_v1"
#define __CHANNEL_VIEW__ "viewer_cloud3d_listener"
#define __CHANNEL_CAM3D__ "cam3d_above_arm_v1_listener"
#define MAX_MSG_LEN 1024

static int              s_cam3d_need_send_width = 1;
static int              s_data_width = 0;
static int              s_data_height = 0;
static int              s_data_bytes_siz = 0;
static float            s_data_value_range = 256.f;
static char             s_buf_sync_w_h[32] = {0};
static float           *s_p_depth = 0;
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
    s_pcbuf = (unsigned char*) malloc(s_data_width * s_data_height);
    memset(s_pcbuf, 0, s_data_width * s_data_height);

    return 0;
}

static int on_message_callback(void *msg, u32 len)
{
    SYS_LOGD("[%s:%d %s]\nReceived! %s\n",
                    __FILE__, __LINE__, __FUNCTION__, (c8*)msg);
    if(msg && strncmp((c8*)msg, "recvd", 5) == 0)
    {
        s_cam3d_need_send_width = 0;
        SYS_LOGD("set cam3d_need_send_width = 0;\n");
    }
}

static void on_ready_to_publish()
{
    int rc = 0;
    /* Sync the widht&&height to the listener, channel viewer */
    while (s_cam3d_need_send_width == 1)
    {
        memset(s_buf_sync_w_h, 0, 32);
        sprintf(s_buf_sync_w_h, "width:%d", s_data_width);
        sys_ros_node_strmsg::instance()->publish(__CHANNEL_VIEW__, s_buf_sync_w_h, 32);
        memset(s_buf_sync_w_h, 0, 32);
        sprintf(s_buf_sync_w_h, "height:%d", s_data_height);
        sys_ros_node_strmsg::instance()->publish(__CHANNEL_VIEW__, s_buf_sync_w_h, 32);
        usleep(1000 * 1000);
    }

    while (1)
    {
        rc = hal_ni_camera3d::instance()->read(s_p_depth, s_data_value_range);
        if (rc == HAL_STATUS_OK)
        {
            for(int i = 0; i < s_data_width * s_data_height; i ++)
            {
                int v = (int)s_p_depth[i];
                v = v > 255 ? 255 : v;
                v = v <= 0 ? 10 : v;
                s_pcbuf[i] = (u8)v;
            }
            int rcl = sys_ros_node_strmsg::instance()->publish(__CHANNEL_VIEW__,
                                                                s_pcbuf,
                                                                s_data_bytes_siz);
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

    sys_ros_node_strmsg::instance()->add_publisher(__CHANNEL_VIEW__, MAX_MSG_LEN);
    sys_ros_node_strmsg::instance()->add_subscriber(__CHANNEL_CAM3D__, MAX_MSG_LEN);
    sys_ros_node_strmsg::instance()->set_on_message_callback_func(on_message_callback);
    sys_ros_node_strmsg::instance()->set_on_ready_to_publish(on_ready_to_publish);

    sys_ros_node_strmsg::instance()->run();

    SYS_LOGI("%s will start\n", __NODE_NAME__);
    sys_ros_node::sys_run();

    return 0;
}
