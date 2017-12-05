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
#include "bpl_learner_token.h"
#include "bpl_tmplt.h"

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
static float           *s_p_depth = 0;
static char            *s_p_color = 0;
static unsigned char  *s_pcbuf = 0;
static pthread_mutex_t   s_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t    s_cond   = PTHREAD_COND_INITIALIZER;
static int               s_alg_finished = 1;

/* Algorithm related varients */
static int                             s_max_world_cache = 8;
static cv::Mat                         s_box_bottom;
static std::vector< Mat_<Vec3f> >      s_front_worlds;
static std::vector< Mat_<Vec3f> >      s_back_worlds;
static int                             s_world_width = 0;
static int                             s_world_height = 0;
static int                             s_merge_threhold = 10;
static cv::Mat                         s_intri[6];
static cv::Mat                         s_result_show;
static cv::Mat                         s_cliped;
static cv::Mat                         s_cliped_rotated;
static cv::Mat                         s_segmented;
static int                             s_t_counter = 0;
static double                          s_threshold = 0.85f;

/* Algorithm BPL learning token */
bpl_learner_token                      *s_bpl_token = NULL;

cv::Mat binaryImg;
cv::Mat colorMat;
cv::Mat depthMat;
cv::Mat rgbImg;

cv::Mat dis;

int box_number = 0;

struct box{
    float x;
    float y;
    float z;
    float box_width;
    float box_height;
};

vector<box> box_set;

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
cv::Mat fill_spots(cv::Mat cliped);
cv::Mat fill_spots_ext(cv::Mat &cliped);
cv::Mat analysis_bin_map_vertical(cv::Mat bin);
cv::Mat analysis_bin_map_horizon(cv::Mat bin);
cv::Mat corner_detect(cv::Mat src_bin);
cv::Mat edge_detect(cv::Mat src);
cv::Mat rotate_image(cv::Mat src, float angle);


static void gen_token_callback(int *img, int w, int h, double result);


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
    s_pcbuf = (unsigned char*) DEBUG_MALLOC(s_data_width * s_data_height);
    memset(s_pcbuf, 0, s_data_width * s_data_height);

    return 0;
}

static int init_alg_configs()
{
    float bottom[3];

    bool ok = loadBinary(FILE_BOTTOM, bottom);
    if(!ok)
    {
        return -1;
    }
    s_box_bottom = Mat(3, 1, CV_32F, bottom).clone();

    t3d::loadIntri(FILE_INTRINSICS, s_intri);

    s_bpl_token = bpl_learner_token::create();
    s_bpl_token->set_gen_token_callback(gen_token_callback, NULL);
    bpl_tmplt *tmplt = new bpl_tmplt();
    tmplt->load_templates_from_file("");
    prm_set_templates(tmplt->tmplts());
    return 0;
}

static Mat_<Vec3f> depth_data_loading(int width, int height, float *buf, int size)
{
    Mat_<Vec3f> world(height, width);
    Mat_<float> depth(height, width);
    memcpy(depth.data, buf, size * sizeof(float));

    vt::depth2World(depth, world, (float*)s_intri[0].data, (float*)s_intri[4].data);

    if((int)(s_back_worlds.size()) >= s_max_world_cache)
    {
        s_back_worlds.erase(s_back_worlds.begin());
    }
    s_back_worlds.push_back(world);

    s_world_width = width;
    s_world_height = height;

    return world;
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

static cv::Mat bpl_preprocess_resultimg(cv::Mat result)
{
    int i = 0;
    cv::Mat image;
    image = result.clone();

    cv::Mat filled = fill_spots_ext(image);

    cv::Mat vert = analysis_bin_map_vertical(filled);
    cv::Mat hori = analysis_bin_map_horizon(filled);

    printf("\n");
    for( i = 0; i < vert.cols; i ++ )
    {
        printf("%d ", vert.data[i]);
    }
    printf("\n");
    printf("\n");
    for( i = 0; i < hori.rows; i ++ )
    {
        printf("%d ", hori.data[i]);
    }
    printf("\n");

    cv::Mat edge = edge_detect(filled);

    float angle = 0.0f;
    float max_aver = -1;
    float max_angle = 0.0f;
    cv::Mat rotated;
    int counter = 0;

    while(angle <= 360.0f)
    {
        counter = 0;
        rotated = rotate_image(filled, angle);
        cv::Mat _vert = analysis_bin_map_vertical(rotated);
        float aver = 0.0f;
        for( i = 0; i < _vert.cols; i ++ )
        {
            if(_vert.data[i] != 0)
                counter ++;
            aver += _vert.data[i];
        }
        aver = aver / counter;
        if(aver > max_aver) {
            max_angle = angle;
            max_aver = aver;
        }
        angle += 2.0f;
    }
    rotated = rotate_image(filled, max_angle);
    s_cliped = clipout_target_image(rotated);
    return s_cliped;
}

static void gen_token_callback(int *img, int w, int h, double result)
{
    cv::Mat showimg(h, w, CV_8U);
    for(int j = 0; j < w*h; j ++)
    {
        showimg.data[j] = (unsigned char)img[j];
    }
    printf("RESULT: %f\n", result);
    if(result > 0.55)
        s_segmented = showimg.clone();
}

double CalculateContourArea(vector<Point> contour)
{
    return fabs(contourArea(contour));
}

static inline bool ContourSortArea(vector<Point> con1, vector<Point> con2)
{
    return (CalculateContourArea(con1) > CalculateContourArea(con2));
}

/*void detect_lines(  Mat src, int rect_info[4], int thickness, vector<Vec2f> &lines )
{
    Rect roi( rect_information[0] );

    Mat img = src(roi).clone();
    Mat edge;

    Canny(img, edge, 50, 200, 3);
    HoughLines(edge, lines, 1, CV_PI / 180, 150, 0, 0);
}*/

static int alg_box_deliver()
{
    /**
     * First, init time0 for measuring the performance
     */
    int time0 = clock();

    /**
     * (1) Merge all front frames
     */
    Mat_<Vec3f> world(s_world_height, s_world_width);
    multiFrames(s_front_worlds, world, s_merge_threhold);

    /**
     * (2) Get box edge&region, get the border
     */
    vector<Mat> wall;
    //Rect rt = boxEst((float*)world.data, (float*)s_box_bottom.data,
//                    world.cols, world.rows, 280, 20, 620, 440, wall);

    //Rect rt = Rect(220, 29, 270, 190);
    Rect rt = Rect(232, 20, 270, 190);

    if ( rt.width == 0 )
    {
        printf("[Exception] not crisis, boxEst failed once.\n");
        return -1;
    }

    // Mapping color Image and depth Image
    short *p_short_depth = (short*) DEBUG_MALLOC(480 * 640 * sizeof(short));

    for(int i = 0; i < 480 * 640; i ++)
    {
        p_short_depth[i] = (short) s_p_depth[i];
    }

    Mat col( 480, 640, CV_8UC3, (char*)s_p_color );
    Mat dep( 480, 640, CV_16S, (char*)p_short_depth);

    colorMat = col.clone();
    depthMat = dep.clone();

    Mat_<Vec3f> world1(480, 640);   // matched depth Image to rgbImage
    Mat depthNew;
    {
        Mat intri[6];
        Mat L2R;
        t3d::loadAstraParams("../../params/camera_params.ini", L2R, intri);
        vt::mappingDepth2Color(depthMat, depthNew, (float*)L2R.data);
        Mat intri2[2];
        t3d::loadIntri2("../../params/intrinsics.yml", intri2);
        depth2World((Mat_<short>)depthNew, world1, (float*)intri2[0].data, (float*)intri2[1].data);
    }

    for (int j = 0; j < world1.total(); j++)
    {
        if (world1(j)(2) < 10)
            world1(j)(2) = world1(j)(0) = world1(j)(1) = NAN;
    }

    world = Mat(world1, rt).clone();   // change the value here
    setBorder(world);
    removePointsOnPlane(world, s_box_bottom);

    /**
     * (3) Do the main dividing alg
     */
    Mat_<short> depth(world.size(), CV_16S);
    for (int i = 0; i < world.total(); i++)
    {
        float& x = world(i)(2);
        if (_isnan(x))
            depth(i) = -1000;
        else
            depth(i) = x;
    }

    // Using RGB Segmentation
    PlaneFit2 ns(depth, world, wall);
    string result = ns.run(s_result_show);

    binaryImg = s_result_show.clone();

    // Plannar Segmentation
    rgbImg = colorMat.clone();
    Mat rgbImgR = rgbImg(rt);
    dilate(binaryImg, binaryImg, Mat::ones(3, 3, CV_8UC1), Point(-1, -1), 2);

    Mat binaryCopy = binaryImg.clone();
    vector< vector <Point> > cons;
    findContours(binaryCopy, cons, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    Rect area = boundingRect(cons[0]);
    Mat background = Mat::zeros(binaryImg.rows, binaryImg.cols, CV_8UC3);

    for (int i = 0; i < binaryImg.rows; i++)
    {
        uchar* data = binaryImg.ptr<uchar>(i);
        uchar* data_rgb = rgbImgR.ptr<uchar>(i);
        uchar* data_pro = background.ptr<uchar>(i);

        for (int j = 0; j < binaryImg.cols; j++)
        {
            if ( data[j] == 255 )
            {
                data_pro[3 * j] = data_rgb[3 * j];
                data_pro[3 * j + 1] = data_rgb[3 * j + 1];
                data_pro[3 * j + 2] = data_rgb[3 * j + 2];
            }
        }
    }

    Mat plannar = background.clone();

    // Laplacian transform for edge detection
    Mat kernel = ( Mat_<float>(3, 3) <<
        1, 1, 1,
        1, -8, 1,
        1, 1, 1 );

    Mat imgLaplacian;
    Mat sharp = plannar;

    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    plannar.convertTo(sharp, CV_32F);

    Mat imgResult = sharp - imgLaplacian;

    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);

    plannar = imgResult;

    // Create binary image from source image
    Mat bw;
    cvtColor(plannar, bw, CV_BGR2GRAY);

    // eliminate the error pixel
    for (int i = 2; i < bw.rows - 2; i++)
    {
        uchar* g_data_m = bw.ptr<uchar>(i);
        uchar* g_data_p = bw.ptr<uchar>(i - 1);
        uchar* g_data_d = bw.ptr<uchar>(i + 1);

        for (int j = 2; j < bw.cols - 2; j++)
        {
            if (g_data_p[j] == 0 && g_data_d[j] == 0)
            {
                g_data_m[j] = 0;
            }

            else if (g_data_m[j - 1] == 0 && g_data_m[j + 1] == 0)
            {
                g_data_m[j] = 0;
            }

            else if (g_data_p[j - 1] == 0 && g_data_d[j + 1] == 0)
            {
                g_data_m[j] = 0;
            }

            else if (g_data_p[j + 1] == 0 && g_data_d[j - 1] == 0)
            {
                g_data_m[j] = 0;
            }
        }
    }

    //medianBlur(bw, bw, 3);
    threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // Hough Transform
    Mat edge, h;
    Canny( bw(area), edge, 200, 255, 3 );

    vector<Vec2f> lines;
    HoughLines(edge, lines, 1, CV_PI / 180, 90, 0, 0);

    /*for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(bw(area), pt1, pt2, Scalar(0), 1, CV_AA);
    }

    threshold(bw, bw, 240, 255, CV_THRESH_BINARY);*/

    // Distance Transform
    Mat dist;
    distanceTransform(bw, dist, CV_DIST_L2, 0);

    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    normalize(dist, dist, 0, 1, NORM_MINMAX);
    Mat distc(dist.size(), CV_8UC1);

    for (int i = 0; i < dist.rows; i++)
    {
        float* data  = dist.ptr<float>(i);
        uchar* datac = distc.ptr<uchar>(i);

        for (int j = 0; j < dist.cols; j++)
        {
            datac[j] = (int)data[j];
        }
    }

    threshold(dist, dist, 0.1, 255, CV_THRESH_BINARY);

    // Find the box contour
    Mat dist8u = Mat::zeros(dist.size(), CV_8U);
    dist.convertTo(dist8u, CV_8U);

    Mat dist_contour = dist8u.clone();
    dis = dist8u.clone();
    dis = dis(area).clone();

    vector< vector<Point> > box_contours;
    findContours(dist_contour, box_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    Mat markers = Mat::zeros(dist.size(), CV_32SC1);

    for (size_t i = 0; i < box_contours.size(); i++)
        drawContours(markers, box_contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);

    sort( box_contours.begin(), box_contours.end(), ContourSortArea );

    vector< RotatedRect > rRect_set;
    vector< vector<Point2f> > rRect_point_set;

    for (size_t i = 0; i < box_contours.size(); i++)
    {
        if ( fabs(contourArea(box_contours[i]))> 500 )
        {
            drawContours( markers, box_contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1 );

            RotatedRect rRect = minAreaRect(box_contours[i]);
            Point2f vertices[4];

            rRect.points(vertices);
            rRect_set.push_back(rRect);

            vector<Point2f> r_points;
            for (int i = 0; i < 4; i++)
            {
                r_points.push_back(vertices[i]);
            }
            rRect_point_set.push_back(r_points);
        }
    }

    vector < vector<Point2f> > rRect_set_right;
    if (rRect_point_set.size() > 0)
    {
        rRect_set_right.push_back(rRect_point_set[0]);

        if (rRect_set.size() > 1)
        {
            for (size_t i = 0; i < rRect_set.size() - 1; i++)
            {
                for (size_t j = 1; j < rRect_set.size(); j++)
                {
                    Point2f small_center;
                    small_center = rRect_set[j].center;

                    bool measure_dist = false;
                    double distance = pointPolygonTest(rRect_point_set[i], small_center, measure_dist);

                    if (distance < 0)
                    {
                        rRect_set_right.push_back(rRect_point_set[j]);
                    }
                }
            }

        }
    }

    //vector<Vec2f> lines;
    //detect_lines( rgbImgR, lines );

    // show result on RGB Image
    box_number = (int)box_contours.size();

    for (int k = 0; k < rRect_set_right.size(); k++)
    {
        int center_x = 0;
        int center_y = 0;

        for (int i = 0; i < 4; i++)
        {
            line( rgbImg, Point2f(rRect_set_right[k][i].x + rt.x, rRect_set_right[k][i].y + rt.y),
                          Point2f(rRect_set_right[k][(i + 1) % 4].x + rt.x, rRect_set_right[k][(i + 1) % 4].y + rt.y), Scalar(255, 255, 0), 2, 8, 0 );

            center_x += rRect_set_right[k][i].x + rt.x ;
            center_y += rRect_set_right[k][i].y + rt.y ;
        }

        center_x = (int)center_x / 4;
        center_y = (int)center_y / 4;

        circle(rgbImg, Point(center_x, center_y), 1, Scalar(255, 0, 0), 2, 8, 0);

        box b;

        // note the x, y, z information from world Mat
        b.x = world1.at<cv::Vec3f>(center_x, center_y)[0];
        b.y = world1.at<cv::Vec3f>(center_x, center_y)[1];
        b.z = world1.at<cv::Vec3f>(center_x, center_y)[2];

        int ctrl_number = 1;

        while( b.x == NAN || b.y == NAN || b.z == NAN )
        {
            if ( ctrl_number%2 == 1 )
            {
                center_x -= 1;
            }

            else
            {
                center_y -= 1;
            }

            b.x = world1.at<cv::Vec3f>(center_x, center_y)[0];
            b.y = world1.at<cv::Vec3f>(center_x, center_y)[1];
            b.z = world1.at<cv::Vec3f>(center_x, center_y)[2];

            ctrl_number++;
        }

        // store the result
        box_set.push_back( b );
    }

    rectangle(rgbImg, Rect(area.x + rt.x, area.y + rt.y, area.width, area.height), Scalar(0, 0, 255), 2, 8, 0);

    printf("result: %s\n", result.c_str());
    printf("time-cost: %d; ", (int)(clock() - time0));
    //cvWaitKey(33);

#if 0
    /* bpl go */
    cv::Mat cliped = bpl_preprocess_resultimg(s_result_show);
    cv::transpose(cliped, s_cliped_rotated);
    cv::Mat c2;
    cv::flip(s_cliped_rotated, c2, 1);

    s_cliped_rotated = c2.clone();

    s_bpl_token->set_compare_image(cliped.data, cliped.cols, cliped.rows);
    s_bpl_token->learn("prm.0001.dat", 0.6);

    s_bpl_token->set_compare_image(s_cliped_rotated.data, s_cliped_rotated.cols, s_cliped_rotated.rows);
    s_bpl_token->learn("prm.0001.dat", 0.6);
#endif
    DEBUG_FREE(p_short_depth);
    return 0;
}

static void show_preview_image(Mat_<Vec3f> world)
{
    Mat_<short> depth(world.size(), CV_16S);
    for (unsigned int i = 0; i < world.total(); i++)
    {
        float x = world(i)(2);
        x = x > 255.0f ? 255 : x;
        x = x < 0.0f ? 0.0f : x;
        if (_isnan(x))
            depth(i) = -1000;
        else
            depth(i) = x;
    }
    //imshow("show", toGray(depth));
}

static int loop_run()
{
    int rc = 0;
    Mat_<Vec3f> world;

    while (1)
    {
        rc = hal_ni_camera3d::instance()->read_raw(s_p_depth, s_p_color);
        //Mat colorMat( 480, 640, CV_8UC3, (char*)s_p_color );

        if (rc == HAL_STATUS_OK)
        {
            world = depth_data_loading(s_data_width, s_data_height, s_p_depth, s_data_width*s_data_height);

            /* Show what we have captured */
            show_preview_image(world);
            if(s_alg_finished)
            {
                printf("swap\n");
                swap_back_front_worlds();
                s_alg_finished = 0;

                if(!s_result_show.empty())
                {
                    //imshow("result", binaryImg);
                    imshow("color", rgbImg);
                    imshow("depth", toGray(depthMat) );
                    if(!dis.empty())
                        imshow("result", dis);
                    if(!s_segmented.empty())
                        imshow("s_segmented", s_segmented);
                    if(!s_cliped_rotated.empty())
                        imshow("r", s_cliped_rotated);
                }

                __THREAD_SIGNAL_NORIFY__
            }
        }
        else
        {
            printf("[%s:%d %s]\nFataError! hal_ni_camera3d return error.\n",
                        __FILE__, __LINE__, __FUNCTION__);
            exit(1);
        }
        usleep(1000 * 10);

        char c = cvWaitKey(33);
        if(c == 27) break;
        else if(c == 's')
        {
//            saveBinary(FILE_CAPTURED, world.data, world.total() * sizeof(float) * 3);
//            Mat plane = estBottom(world, Rect(160, 175, 290, 175));
//            saveBinary(FILE_BOTTOM_NEW, plane.data, 12);
//            cout << "plane: " << plane;
            //if(!s_result_show.empty())
                //cv::imwrite("../../libs/s_result_0516.jpg", s_result_show);
        }
        else if(c == 't')
        {
            Mat_<Vec3f> world(s_world_height, s_world_width);
            vector<Mat> wall;
            loadBinary(FILE_CAPTURED, world.data);
            Rect rt = boxEst((float*)world.data, (float*)s_box_bottom.data,
                                    world.cols, world.rows, 280, 20, 620, 440, wall);
            cout << rt;
            cout << s_box_bottom;
        }
    }
    return 0;
}

static void *pthread_func_propel_boxes(void *args)
{
    while(1)
    {
        __THREAD_SIGNAL_WAIT__

        /* algorithm go */
        alg_box_deliver();

        /* tell finished */
        s_alg_finished = 1;
    }
    return args;
}

int main(int argc, char **argv)
{
    pthread_t thread;

    init_hal();
    init_alg_configs();
    printf("init hal and alg ok\n");

    pthread_create(&thread, NULL, pthread_func_propel_boxes, NULL);

    //imshow("win", binaryImg);
    loop_run();

    if(argc > 1)
    {
        return argv == NULL;
    }

    return 0;
}
