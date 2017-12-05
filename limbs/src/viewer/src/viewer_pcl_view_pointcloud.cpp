#include "hal/hal_gl.h"
#include "sys/sys_ros_node_strmsg.h"
#include "sys/sys_ros_node_pointcloud.h"

using namespace cainiao_robot;

#define __NODE_NAME__ "viewer_pcl_view_pointcloud"
#define __CHANNEL_SELF__ "viewer_pcl_view_pointcloud_listener"
#define __ARGS__ argc, argv
#define MAX_BUF_LEN 1024
static float s_data_value_range = 256.f;

/*
 *      x    y
 *       \   |
          \  |
           \ |
z<------------

 * */
static int render_points(float *points, int w, int h)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin (GL_QUADS);

    for(size_t i = 0; i < w * h; i ++)
    {
        float x = points[i * 3];
        float y = points[i * 3 + 1];
        float z = points[i * 3 + 2];

        float color = y / s_data_value_range;
        glColor3f(color, 1.0 - color, 1.0 - color);

        glVertex3f(x - 2, y, z + 2);
        glVertex3f(x + 2, y, z + 2);
        glVertex3f(x + 2, y, z - 2);
        glVertex3f(x - 2, y, z - 2);

    }

    glEnd();

    hal_gl::instance()->swap_buffer();
}

static void update_hal_graph_config()
{
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, 640.f / 480.f, 0.0, 10.0);

    /* reset modelview matrix to the identity matrix */
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 600.0, 400, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

static int init_hal_graph_env()
{
    int rc = hal_gl::instance()->init();
    if (rc != HAL_STATUS_OK)
    {
        SYS_LOGE("%s FatalError after hal_gl init", __FUNCTION__);
        return -1;
    }
}

static int on_message_callback_pointcloud(void *msg, u32 len)
{
    msg_pointcloud_t *ppt = (msg_pointcloud_t*)msg;

    render_points(ppt->data, ppt->width, ppt->height);
    return 0;
}

static void ready_to_publish()
{
}

int main(int argc, char **argv)
{
    int rc = HAL_STATUS_OK;

    /* System init */
    SYS_ROS_GLOBAL_ENV_INIT(argc, argv, (char*)__NODE_NAME__)

    /* 1. Initialize hal graphic environment */
    rc = init_hal_graph_env();
    if(rc != HAL_STATUS_OK)
    {
        return -1;
    }

    /* 2. First time update graph config */
    update_hal_graph_config();

    /* 3. Open the graph view, it will pop a X-window in linux */
    hal_gl::instance()->open();

    /* 4. Setup ros node */
    sys_ros_node_pointcloud::instance()->add_subscriber(__CHANNEL_SELF__, MAX_BUF_LEN);
    sys_ros_node_pointcloud::instance()->set_on_message_callback_func(on_message_callback_pointcloud);
    sys_ros_node_pointcloud::instance()->set_on_ready_to_publish(ready_to_publish);

    sys_ros_node_pointcloud::instance()->run();

    /* 5. Enter ros running */
    SYS_LOGI("%s will start\n", __NODE_NAME__);
    sys_ros_node::sys_run();

    return 0;
}
