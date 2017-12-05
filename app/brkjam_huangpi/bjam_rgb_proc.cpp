#include "bjam_rgb_proc.h"
#include "vt_io.h"
#include "vt_visual.h"
#include "vt_pointCloud.h"
#include "vt_depthTrans.h"
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "CameraPara.h"

using namespace cv;
using namespace std;

static int s_box_number = 0;
static vector< rgb_box_t > s_box_set;
cv::Mat bjam_rgb_proc::s_distance_result_image;
cv::Mat bjam_rgb_proc::s_rgb_image;
cv::Mat bjam_rgb_proc::s_color_gui;
extern cv::Mat g_colorMat;

static double CalculateContourArea(vector<Point> contour)
{
    return fabs(contourArea(contour));
}

static inline bool ContourSortArea(vector<Point> con1, vector<Point> con2)
{
    return (CalculateContourArea(con1) > CalculateContourArea(con2));
}

cv::Mat bjam_rgb_proc::rotate_Img( Mat img, float angle, Mat &map_matrix )
{
    float a = sin(angle);
    float b = cos(angle);

    int degree = angle * 180. / CV_PI;

    int width = img.cols;
    int height = img.rows;

    int width_rotate  = int(height * fabs(a) + width * fabs(b));
    int height_rotate = int(width * fabs(a) + height * fabs(b));

    float* map;
    map_matrix = Mat(2, 3, CV_32F);
    map = (float*)map_matrix.data;
    CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);
    CvMat map_matrix2 = map_matrix;

    cv2DRotationMatrix(center, degree, 1.0, &map_matrix2);

    map[2] += (width_rotate - width) / 2;
    map[5] += (height_rotate - height) / 2;

    Mat img_rotate;
    warpAffine(img, img_rotate, map_matrix, Size(width_rotate, height_rotate), 1, 0, 0);

    return img_rotate;
}

cv::Point2f bjam_rgb_proc::trans_point( cv::Point2f p, cv::Mat matrix )
{
    Point2f point_area;

    Mat po(2, 1, CV_32F);
    po.at<float>(0, 0) = p.x ;
    po.at<float>(1, 0) = p.y ;

    float ma[4] = { matrix.at<float>(0, 0), matrix.at<float>(0, 1), matrix.at<float>(1, 0), matrix.at<float>(1, 1) };
    Mat M = Mat(2, 2, CV_32F, ma);

    Mat pm;
    pm = M * po;

    point_area.x = pm.at<float>(0, 0) + matrix.at<float>(0, 2);
    point_area.y = pm.at<float>(1, 0) + matrix.at<float>(1, 2);

    return point_area;
}

cv::Point2f bjam_rgb_proc::trans_point_back( cv::Point2f p, cv::Mat matrix )
{
    Point2f pt;
    pt.x = 0.0f;
    pt.y = 0.0f;

    float ma[4] = { matrix.at<float>(0, 0), matrix.at<float>(0, 1), matrix.at<float>(1, 0), matrix.at<float>(1, 1) };
    Mat M = Mat(2, 2, CV_32F, ma);

    Mat pm;
    Mat pc(2, 1, CV_32F);

    pc.at<float>(0, 0) = p.x - matrix.at<float>(0, 2);
    pc.at<float>(1, 0) = p.y - matrix.at<float>(1, 2);

    pm = M.inv() * pc;
    pt.x = pm.at<float>(0, 0);
    pt.y = pm.at<float>(1, 0);

    return pt;
}

/**
 * @brief bjam_rgb_proc::run
 * @param depth
 * @param color
 * @param binary_image_segmented
 * @param rt
 *
 * TODO: 1. Tidy this function's code.
 *       2. Write more comments.
 */
int bjam_rgb_proc::run( Mat depth,
                        Mat color,
                        Mat binary_image_segmented,
                        Mat world1,
                        Rect rt,
                        Point &px,
                        RotatedRect &rotated_rect_target  )
{
    Mat binaryImg = binary_image_segmented.clone();
    Mat colorMat = color.clone();
    Mat depthMat = depth.clone();
    //Mat rgbImg;

    // Plannar Segmentation
    Mat rgbImg = colorMat.clone();
    Mat rgbImgR = rgbImg(rt);
    dilate(binaryImg, binaryImg, Mat::ones(3, 3, CV_8UC1), Point(-1, -1), 2);

    Mat binaryCopy = binaryImg.clone();
    std::vector< std::vector <Point> > cons;
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

    // Distance Transform
    Mat dist;
    distanceTransform(bw, dist, CV_DIST_L2, 3);

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

    Mat dist_contour = dist8u(area).clone();
    s_distance_result_image = dist8u.clone();
    s_distance_result_image = s_distance_result_image(area).clone();

    vector< vector<Point> > box_contours;
    findContours(dist_contour, box_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(!box_contours.size())
    {
        return -1;
    }

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

    // Get the rotation angle
    int id = 0;

    if( rRect_set.size() > 1 )
    {
        Point2f temp = rRect_set[0].center;

        for( size_t i = 1; i < rRect_set.size(); i++ )
        {
            if( sqrt( rRect_set[i].center.x*rRect_set[i].center.x + rRect_set[i].center.y*rRect_set[i].center.y )
              < sqrt( temp.x*temp.x + temp.y*temp.y ) )
            {
                temp = rRect_set[i].center;
                id = i;
            }
        }
    }

    rotated_rect_target = rRect_set[id];

    vector < vector<Point2f> > rRect_set_right;
    if (rRect_point_set.size() > 0)
    {
		rRect_set_right.push_back( rRect_point_set[0] );

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

   // vector<Vec2f> lines;
    //detect_lines( rgbImgR, lines );

    // show result on RGB Image
    s_box_number = (int)box_contours.size();

    for (int k = 0; k < (int)rRect_set_right.size(); k++)
    {
        int center_x = 0;
        int center_y = 0;

        for (int i = 0; i < 4; i++)
        {
            line( colorMat, Point2f(rRect_set_right[k][i].x + rt.x + area.x, rRect_set_right[k][i].y + rt.y + area.y),
                            Point2f(rRect_set_right[k][(i + 1) % 4].x + rt.x + area.x, rRect_set_right[k][(i + 1) % 4].y + rt.y + area.y),
                            Scalar(255, 255, 0), 2, 8, 0 );

            center_x += rRect_set_right[k][i].x + rt.x;
            center_y += rRect_set_right[k][i].y + rt.y;
        }

        center_x = (int)center_x / 4;
        center_y = (int)center_y / 4;

        circle(rgbImg, Point(center_x, center_y), 1, Scalar(255, 0, 0), 2, 8, 0);

        rgb_box_t b;

        // note the x, y, z information from world Mat
        b.x = world1.at<Vec3f>(center_x, center_y)[0];
        b.y = world1.at<Vec3f>(center_x, center_y)[1];
        b.z = world1.at<Vec3f>(center_x, center_y)[2];

        // store the result
        s_box_set.push_back( b );
    }

    rectangle(colorMat, Rect(area.x + rt.x, area.y + rt.y, area.width, area.height), Scalar(0, 0, 255), 2, 8, 0);

    px.x = area.x;
    px.y = area.y;

//    imshow("color", rgbImg);
//    imshow("inside", colorMat);
    s_color_gui = colorMat.clone();

    s_rgb_image = rgbImg.clone();

    return 0;
}
