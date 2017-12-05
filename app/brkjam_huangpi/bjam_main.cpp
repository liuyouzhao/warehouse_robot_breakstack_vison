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
#include "mm.h"
#include "ipc.h"
#include "bpl_learner_token.h"
#include "bpl_tmplt.h"
#include "bjam_rgb_proc.h"
#include "bjam_ltjudgement.h"
#include "bjam_depth_proc.h"
#include "bjam_state.h"
#include "net/sys_linux_net_broadcast.h"
#include "robotarm/aubo_robot_arm.h"
#include "Rotation.h"
#include "jconf.h"


using namespace vt;
using namespace std;
using namespace cv;
using namespace cainiao_robot;

#define _isnan std::isnan
#define FILE_BOTTOM "../../libs/bottom"
#define FILE_BOTTOM_NEW "../../libs/bottom_new"
#define FILE_INTRINSICS "../../libs/intrinsics.yml"
#define FILE_CAPTURED "../../captured"

/* Camere related varients */
static int              s_data_width = 0;
static int              s_data_height = 0;
static int              s_data_bytes_siz = 0;
static float            *s_p_depth = 0;
static char             *s_p_color = 0;
static pthread_mutex_t   s_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t    s_cond   = PTHREAD_COND_INITIALIZER;

/* Algorithm related varients */
static std::vector< Mat_<Vec3f> >      s_front_worlds;
static std::vector< Mat_<Vec3f> >      s_back_worlds;
static short                           *s_p_short_depth;
static sku_attribute_t                 s_sku;

int box_number = 0;

typedef struct _pick_data_s
{
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} _pick_data_t;

#define __THREAD_SIGNAL_WAIT__ \
        pthread_mutex_lock( &s_mutex ); \
        pthread_cond_wait( &s_cond, &s_mutex ); \
        pthread_mutex_unlock( &s_mutex );

#define __THREAD_SIGNAL_NORIFY__ \
        pthread_mutex_lock( &s_mutex ); \
        pthread_cond_signal( &s_cond ); \
        pthread_mutex_unlock( &s_mutex );

/* Methods in improc.cpp for BPL pre-process */
cv::Mat_<float> clipout_target_image(cv::Mat target);
cv::Mat clipout_target_image(cv::Mat target, int &l, int &r, int &t, int &b);
cv::Mat clipout_target_image_remain(cv::Mat target, int &l, int &r, int &t, int &b);

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
    s_p_depth = (float*) DEBUG_MALLOC(s_data_bytes_siz);
    s_p_color = (char*) DEBUG_MALLOC(s_data_bytes_siz);

    return 0;
}

static int swap_back_front_worlds()
{
    s_front_worlds.clear();
    while(!s_back_worlds.empty())
    {
        s_front_worlds.push_back(s_back_worlds.front());
        s_back_worlds.erase(s_back_worlds.begin());
    }
    return 0;
}

static inline bool ContourSortLeft(RotatedRect r1, RotatedRect r2)
{
    return (r1.center.x < r2.center.x);
}

static int pick_deliver()
{

    _pick_data_t pick;
    double handeye[24];
    loadBinary("../../params/handeye", handeye);
    Mat R, t, tR, tt;

    float temp[24];
    for (int i = 0; i < 24; i++)
    temp[i] = handeye[i];

    R = Mat(3, 3, CV_32F, temp).clone();
    t = Mat(3, 1, CV_32F, temp + 9).clone();
    tR = Mat(3, 3, CV_32F, temp + 12).clone();
    tt = Mat(3, 1, CV_32F, temp + 21).clone();

    Mat ptR2;
    Mat_<double> ptR;
    BDPS->ns()->pickR.convertTo(ptR2, CV_32F);
    Mat Re = R*ptR2*tR.t();
    Re.convertTo(ptR, CV_64F);
    Mat P = (Mat_<float>(3, 1) << BDPS->vpts()[0], BDPS->vpts()[1], BDPS->vpts()[2]);
    cout <<P<<endl;
    {
        Mat* intri = bjam_depth_proc::singleton()->intri();
        Mat R,t;
        intri[2].convertTo(R, CV_32F);
        intri[3].convertTo(t, CV_32F);
        P = R.inv()*P-R.inv()*t;
        cout<<P<<endl;
    }

    Mat newP = (R*P + t) / 1000;
    pick.x = newP.at<float>(0);
    pick.y = newP.at<float>(1);
    pick.z = newP.at<float>(2)+0.005;
    cv::Vec3d angle = rotationMatrixToEulerAngles(ptR);
    angle[2] = 0;
    cv::Vec4d q = eulerAngles2quaternion(angle);
    pick.qx = q[1];
    pick.qy = q[2];
    pick.qz = q[3];
    pick.qw = q[0];

    cout << newP << " q: " << q << endl;

    sys_linux_net_broadcast udp;
    udp.net_broadcast((char*)(&pick), sizeof(_pick_data_t), 26820);

    return 0;
}

static int init_alg_configs()
{
    jconf jc("../../params/sku/now");
    int uuid = (int)jc.get_number("uuid");
    bjam_state_load_sku(s_sku, uuid);
    bjam_depth_proc::singleton()->set_sku_size( s_sku.box_rect_size[s_sku.upper_index].x,
                                                s_sku.box_rect_size[s_sku.upper_index].y);
    s_p_short_depth = (short*) DEBUG_MALLOC(BDPS->screen_height() *
                                            BDPS->screen_width() *
                                            sizeof(short));
    return 0;
}

static void convert_depth_data(short *p_short_depth, float *p_depth, int size)
{
    for(int i = 0; i < size; i ++)
    {
        p_short_depth[i] = (short) p_depth[i];
    }
}

static int alg_box_deliver()
{
    /**
     * First, init time0 for measuring the performance
     */
    static int sample_id = 900;
    int time0 = clock();
    int time1 = 0;

    /**
     * (1) Do 1st step depth and color segmentation
     */
    Mat col( bjam_depth_proc::singleton()->screen_height(),
             bjam_depth_proc::singleton()->screen_width(),
             CV_8UC3, (char*)s_p_color );
    Mat dep( bjam_depth_proc::singleton()->screen_height(),
             bjam_depth_proc::singleton()->screen_width(),
             CV_16S, (char*)s_p_short_depth);

    bjam_depth_proc::singleton()->inform_color_depth_map(col, dep);

    /**
      Segmentation go go go!
      */
    bjam_depth_proc::singleton()->run();

//    if(bjam_depth_proc::singleton()->has_damn_right() != 0)
//    {
//        imshow("only by depth", bjam_depth_proc::singleton()->result_image());
//        pick_deliver();
//        return 0;
//    }

    cv::imshow("dep", dep);
    cv::imshow("col", col);

    while(1)
    {
        unsigned char key = cv::waitKey(100);
        if(key == 0x20)
        {
            char file[128] = {0};
            char file2[128] = {0};
            sprintf(file, "../../libs/col_%d.jpg", sample_id);
            sprintf(file2, "../../libs/dep_%d.dep", sample_id);

            FILE *f = fopen(file2, "w");

            cv::imwrite(file, col);

            fwrite(s_p_short_depth, bjam_depth_proc::singleton()->screen_height() *
                   bjam_depth_proc::singleton()->screen_width() * sizeof(short), 1, f);

            fclose(f);
            printf("save %s %s ok\n", file, file2);
            sample_id ++;
            return 0;
        }
    }


    /**
     * (2) RGB segmentation!
     */
    Point out_pt_add;
    RotatedRect out_rotate_rect;
    int rgb_result = bjam_rgb_proc::run(dep,
                                        col,
                                        bjam_depth_proc::singleton()->result_image(),
                                        bjam_depth_proc::singleton()->world1(),
                                        bjam_depth_proc::singleton()->region(),
                                        out_pt_add,
                                        out_rotate_rect);
    if( rgb_result != 0 )
        return rgb_result;

    /**
     * (3) Rotate image
     */
    cv::Mat matrix;
    float radian = out_rotate_rect.angle * CV_PI / 180.0f;
    cv::Mat distRotated = bjam_rgb_proc::rotate_Img(bjam_rgb_proc::dist(), radian, matrix);

    Point2f vertices[4];
    out_rotate_rect.points(vertices);

    vector<float> x_set;
    vector<float> y_set;

    for( int k = 0; k < 4; k++ )
    {
        Point2f point_area = bjam_rgb_proc::trans_point( vertices[k], matrix );
       //circle( g_colorMat, Point2f(vertices[k].x + rt.x + pt_add.x, vertices[k].y + rt.y + pt_add.y), 1, Scalar(255, 0, 0), 2, 8, 0);

        x_set.push_back(point_area.x);
        y_set.push_back(point_area.y);
    }

    sort(x_set.begin(), x_set.end());
    sort(y_set.begin(), y_set.end());

    Mat dist_processed = Mat::zeros( distRotated.size(), CV_8UC1 );
    for( int i = 0; i < dist_processed.rows; i++ )
    {
        uchar* data = dist_processed.ptr<uchar>(i);
        for( int j = 0; j < dist_processed.cols; j++ )
        {
            if( i > y_set[0] && i < y_set[3]
             && j > x_set[0] && j < x_set[3] )
            {
                data[j] = 255;
            }
        }
    }

    /**
     * (4) Clip image and do bpl score
     *  s1 => verticle
        s2 => horizontal
    */
    int l, r, t, b;
    int type = 0;
    double prob = 0.0;
    Rect rect_pick;
    Rect rt = bjam_depth_proc::singleton()->region();
    float ls = bjam_depth_proc::singleton()->length_scale();

    cv::Mat dist_rotated_cliped = clipout_target_image_remain(distRotated, l, r, t, b);
    bjam_ltjudgement::init_params(ls, dist_rotated_cliped, s_sku);
    bjam_ltjudgement::run(type, 1);

    prob = bjam_ltjudgement::confidence();
    printf("****************type: %d  prob: %f\n", type, prob);

    rect_pick = bjam_ltjudgement::get_first_rect();
    rect_pick.x += l;
    rect_pick.y += t;
    if(type == 2)
    {
        float w = rect_pick.width;
        rect_pick.width = rect_pick.height;
        rect_pick.height = w;
    }

    /**
      * (5) Transfer points back to screen
      */
    Point2f point_set[4] =
    {
        Point2f(rect_pick.x, rect_pick.y),
        Point2f(rect_pick.x + rect_pick.width, rect_pick.y),
        Point2f(rect_pick.x + rect_pick.width, rect_pick.y + rect_pick.height),
        Point2f(rect_pick.x, rect_pick.height + rect_pick.y)
    };

    vector<Point2f> pick_point_set;
    for(int i = 0; i < 4; i++)
    {
        cv::Mat gui = bjam_rgb_proc::gui();
        Point2f pt = bjam_rgb_proc::trans_point_back( point_set[i], matrix );
        pick_point_set.push_back(Point2f(pt.x + out_pt_add.x, pt.y + out_pt_add.y));

        Point pt_trans( pt.x + rt.x + out_pt_add.x, pt.y + rt.y + out_pt_add.y );
        circle( gui, pt_trans, 1, Scalar(255, 0, 0), 2, 8, 0);
    }

    vector<Point> pick_point_in_color;
    for(int i = 0; i < 4; i++)
    {
        Point p;
        p.x = pick_point_set[i].x + 0.5;
        p.y = pick_point_set[i].y + 0.5;
        pick_point_in_color.push_back(p);
        printf("point %d %d \n", p.x,p.y);
    }

    bjam_depth_proc::singleton()->recalculate(pick_point_in_color);

    /**
     * (6) Pick arm go go go!
     *
     */
#ifndef EYEONLY
    pick_deliver();
#endif
    time1 = clock();
    printf("[DEBUG] main alg cost: %d ms\n", (time1 - time0) / 1000);

    return 0;
}

static int alg_box_test(const char *f1, const char* f2, int w, int h)
{
    /**
     * First, init time0 for measuring the performance
     */
    int time0 = clock();
    int time1 = 0;

    unsigned short *p_short_dep = (unsigned short *)DEBUG_MALLOC(sizeof(short) * w * h);
    FILE *f = fopen(f2, "r");
    fread(p_short_dep, sizeof(short) * w * h, 1, f);
    fclose(f);

    Mat col = cv::imread(f1);
    Mat dep( h,
             w,
             CV_16S, (char*)p_short_dep);

    bjam_depth_proc::singleton()->inform_color_depth_map(col, dep);

    /**
      Segmentation go go go!
      */
    bjam_depth_proc::singleton()->run();

//    if(bjam_depth_proc::singleton()->has_damn_right() != 0)
//    {
//        imshow("only by depth", bjam_depth_proc::singleton()->result_image());
//        pick_deliver();
//        return 0;
//    }



    /**
     * (2) RGB segmentation!
     */
    Point out_pt_add;
    RotatedRect out_rotate_rect;
    int rgb_result = bjam_rgb_proc::run(dep,
                                        col,
                                        bjam_depth_proc::singleton()->result_image(),
                                        bjam_depth_proc::singleton()->world1(),
                                        bjam_depth_proc::singleton()->region(),
                                        out_pt_add,
                                        out_rotate_rect);



    if( rgb_result != 0 )
        return rgb_result;

    /**
     * (3) Rotate image
     */
    cv::Mat matrix;
    float radian = out_rotate_rect.angle * CV_PI / 180.0f;
    cv::Mat distRotated = bjam_rgb_proc::rotate_Img(bjam_rgb_proc::dist(), radian, matrix);

    Point2f vertices[4];
    out_rotate_rect.points(vertices);

    vector<float> x_set;
    vector<float> y_set;

    for( int k = 0; k < 4; k++ )
    {
        Point2f point_area = bjam_rgb_proc::trans_point( vertices[k], matrix );
        //circle( g_colorMat, Point2f(vertices[k].x + rt.x + pt_add.x, vertices[k].y + rt.y + pt_add.y), 1, Scalar(255, 0, 0), 2, 8, 0);

        x_set.push_back(point_area.x);
        y_set.push_back(point_area.y);
    }

    sort(x_set.begin(), x_set.end());
    sort(y_set.begin(), y_set.end());

    Mat dist_processed = Mat::zeros( distRotated.size(), CV_8UC1 );
    for( int i = 0; i < dist_processed.rows; i++ )
    {
        uchar* data = dist_processed.ptr<uchar>(i);
        for( int j = 0; j < dist_processed.cols; j++ )
        {
            if( i > y_set[0] && i < y_set[3]
             && j > x_set[0] && j < x_set[3] )
            {
                data[j] = 255;
            }
        }
    }
    /**
     * (4) Clip image and do bpl score
     *  s1 => verticle
        s2 => horizontal
    */
    int l, r, t, b;
    int type = 0;
    double prob = 0.0;
    Rect rect_pick;
    Rect rt = bjam_depth_proc::singleton()->region();
    float ls = bjam_depth_proc::singleton()->length_scale();

    cv::Mat dist_rotated_cliped = clipout_target_image_remain(distRotated, l, r, t, b);

    cv::imshow("dist_rotated_cliped", dist_rotated_cliped);

    bjam_ltjudgement::init_params(ls, dist_rotated_cliped, s_sku);
    bjam_ltjudgement::run(type, 1);

    prob = bjam_ltjudgement::confidence();
    printf("****************type: %d  prob: %f\n", type, prob);

    rect_pick = bjam_ltjudgement::get_first_rect();
    rect_pick.x += l;
    rect_pick.y += t;
    if(type == 2)
    {
        float w = rect_pick.width;
        rect_pick.width = rect_pick.height;
        rect_pick.height = w;
    }

    /**
      * (5) Transfer points back to screen
      */
    Point2f point_set[4] =
    {
        Point2f(rect_pick.x, rect_pick.y),
        Point2f(rect_pick.x + rect_pick.width, rect_pick.y),
        Point2f(rect_pick.x + rect_pick.width, rect_pick.y + rect_pick.height),
        Point2f(rect_pick.x, rect_pick.height + rect_pick.y)
    };

    vector<Point2f> pick_point_set;
    for(int i = 0; i < 4; i++)
    {
        cv::Mat gui = bjam_rgb_proc::gui();
        Point2f pt = bjam_rgb_proc::trans_point_back( point_set[i], matrix );
        pick_point_set.push_back(Point2f(pt.x + out_pt_add.x, pt.y + out_pt_add.y));

        Point pt_trans( pt.x + rt.x + out_pt_add.x, pt.y + rt.y + out_pt_add.y );
        circle( gui, pt_trans, 1, Scalar(255, 0, 0), 2, 8, 0);

        cv::imshow("gui", gui);
    }

    vector<Point> pick_point_in_color;
    for(int i = 0; i < 4; i++)
    {
        Point p;
        p.x = pick_point_set[i].x + 0.5;
        p.y = pick_point_set[i].y + 0.5;
        pick_point_in_color.push_back(p);
        printf("point %d %d \n", p.x,p.y);
    }

    bjam_depth_proc::singleton()->recalculate(pick_point_in_color);

    cv::imshow("dep", dep);


    /**
     * (6) Pick arm go go go!
     *
     */
#ifndef EYEONLY
    pick_deliver();
#endif
    time1 = clock();
    printf("[DEBUG] main alg cost: %d ms\n", (time1 - time0) / 1000);

    return 0;
}

static int loop_run()
{
    int rc = 0;
    Mat_<Vec3f> world;

    while (1)
    {
        rc = hal_ni_camera3d::instance()->read_raw(s_p_depth, s_p_color);

        if (rc == HAL_STATUS_OK)
        {
            swap_back_front_worlds();

            /* convert float array to short array */
            convert_depth_data(s_p_short_depth, s_p_depth, s_data_width * s_data_height);

            /* algorithm go */
            alg_box_deliver();

            /**
              Show all images, for debug
              TODO: here shows all image we need
              */
            if(!bjam_rgb_proc::gui().empty())
                cv::imshow("gui", bjam_rgb_proc::gui());

            __THREAD_SIGNAL_NORIFY__

        }
        else
        {
            printf("[%s:%d %s]\nFataError! hal_ni_camera3d return error.\n",
                        __FILE__, __LINE__, __FUNCTION__);
            exit(1);
        }
        //usleep(1000 * 10);
//        while(1)
//        {
//            char c = cvWaitKey(33);
//            if(c == 27) break;
//        }
    }
    return 0;
}

static int test_run()
{
    static int num = 901;
    for(int i = 0; i < num; i ++)
    {
        char f1[128] = {0};
        char f2[128] = {0};
        sprintf(f1, "../../libs/col_%d.jpg", i);
        sprintf(f2, "../../libs/dep_%d.dep", i);

        alg_box_test(f1, f2, 640, 480);

        while(1)
        {
            char c = waitKey(100);
            if(c == 27) break;
        }
    }
}


#define CAPTURE_IMAGE 0
int main(int argc, char **argv)
{
#if CAPTURE_IMAGE
    init_hal();
    init_alg_configs();

    loop_run();
#else
    init_alg_configs();
    test_run();
#endif
    if(argc > 1)
    {
        return argv == NULL;
    }
    return 0;
}
