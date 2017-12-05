#include "hal/hal_gl.h"
#include "sys/sys_ros_node_strmsg.h"

using namespace cainiao_robot;

#define __NODE_NAME__ "viewer_cloud3d"
#define __CHANNEL__ "viewer_cloud3d_listener"
#define __CHANNEL_PUB__ "cam3d_above_arm_v1_listener"
#define __ARGS__ argc, argv
#define MAX_BUF_LEN 1024

static int s_width = 0;
static int s_height = 0;
static int s_has_recvd_notified = 0;

/*
 *      x    y
 *       \   |
          \  |
           \ |
z<------------

 * */
static int render_points(unsigned char *depth, int w, int h)
{
    int bw = 4.0f;
    int bh = 4.0f;

    float dist = 100.0f;
    int ds = 1;
    int ws = w / ds;
    int hs = h / ds;
    float range = 256.0f;

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin (GL_QUADS);

    for (int i = 0, i2 = 0; i < hs; i += 1, i2 += ds)
    {
        for (int j = 0, j2 = 0; j < ws; j += 1, j2 += ds)
        {
            float x = (i - ws / 2.0f) * bw;
            float z = (j - hs / 2.0f) * bh;

            float color = (float)depth[i2 * w + j2] / range;
            float y = (float)depth[i2 * w + j2] / range * dist;

            glColor3f(color, 1.0 - color, 1.0 - color);

            glVertex3f(x - 2, y, z + 2);
            glVertex3f(x + 2, y, z + 2);
            glVertex3f(x + 2, y, z - 2);
            glVertex3f(x - 2, y, z - 2);
        }
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
    gluLookAt(400, 800.0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
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

int parse_msg(const char *msg, unsigned int len)
{
    char w[16] = { 0 };
    char h[16] = { 0 };
    if (strncmp(msg, "width:", 6) == 0)
    {
        memcpy(w, msg + 6, len - 6);
        s_width = atoi(w);
    }
    else if (strncmp(msg, "height:", 7) == 0)
    {
        memcpy(h, msg + 7, len - 7);
        s_height = atoi(h);
    }

    if(s_width > 0 && s_height > 0 && s_has_recvd_notified == 0)
    {
        sys_ros_node_strmsg::instance()->publish(__CHANNEL_PUB__, (void*)"recvd", 5);
        s_has_recvd_notified = 1;
    }
    else if(s_has_recvd_notified)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static int on_message_callback(void *msg, u32 len)
{
    /* simulately render points */
    if(parse_msg((c8*)msg, len) == 1 &&
            s_width > 0 && s_width < 4096 && s_height > 0 && s_height < 4096)
    {
        render_points((u8*)msg, s_width, s_height);
    }
}

static void ready_to_publish() { }

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
    sys_ros_node_strmsg::instance()->add_subscriber(__CHANNEL__, MAX_BUF_LEN);
    sys_ros_node_strmsg::instance()->add_publisher(__CHANNEL_PUB__, MAX_BUF_LEN);
    sys_ros_node_strmsg::instance()->set_on_message_callback_func(on_message_callback);
    sys_ros_node_strmsg::instance()->set_on_ready_to_publish(ready_to_publish);

    sys_ros_node_strmsg::instance()->run();

    /* 5. Enter ros running */
    SYS_LOGI("%s will start\n", __NODE_NAME__);
    sys_ros_node::sys_run();

    return 0;
}