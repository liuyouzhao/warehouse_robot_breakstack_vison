#include "opencv2/opencv.hpp"

#define SAMPLES 500
#define T_MAGIN 4

#define POSS 1

#if POSS
#define T_WIDTH 75
#define T_HEIGHT 53
#define NOISE_M 0.5
#define FILE_NAME "t3-n/vec-desc-"
#else
#define T_WIDTH 53
#define T_HEIGHT 75
#define NOISE_M 0.5
#define FILE_NAME "t3-t/vec-desc-ivt-"
#endif

using namespace cv;

static Rect sRegion;
static Mat sColor;
static Mat sRotate;
static int sCurrentId;
static int sCurrentSaveId;

static void invertBinary(Mat &mat)
{
    for(int i = 0; i < mat.rows; i ++)
    {
        for(int j = 0; j < mat.cols; j ++)
        {
            int value = mat.at<unsigned char>(i, j);
            value = value == 0 ? 255 : 0;
            mat.at<unsigned char>(i, j) = (unsigned char)value;
        }
    }
}

static void writeIntArrayToFile(const char *filename, int *arr, int len)
{
    char buffer[len * 4];
    memset(buffer, 0, len * 4);
    for(int i = 0; i < len; i ++)
    {
        sprintf(buffer, "%s%d ", buffer, arr[i]);
    }
    printf("%s\n", buffer);

    FILE *f = fopen(filename, "w");
    if(f == NULL)
    {
        return;
    }
    fwrite(buffer, 1, strlen(buffer), f);
    fclose(f);
}

static void writeFloatArrayToFile(const char *filename, float *arr, int len)
{
    char buffer[len * 6];
    char frag[6] = {0};
    memset(buffer, 0, len * 6);
    for(int i = 0; i < len; i ++)
    {
        memset(frag, 0, 6);
        sprintf(frag, "%.3f ", arr[i]);
        memcpy(buffer + i * 6, frag, 6);
        //sprintf(buffer, "%s%.3f ", buffer, arr[i]);
    }
    printf("%s\n", buffer);

    FILE *f = fopen(filename, "w");
    if(f == NULL)
    {
        return;
    }
    fwrite(buffer, 1, strlen(buffer), f);
    fclose(f);
}


static Point pt1;
static Point pt2;
static int n = 0;
static double degree = 0;

static void onMouseHandler_A(int event, int x, int y, int flag, void* pA)
{
    switch (event)
    {
    case EVENT_LBUTTONUP:
    {
        if(n == 0)
        {
            n ++;
            pt1 = Point(x, y);
        }
        else
        {
            pt2 = Point(x, y);
        }
    } break;
    }
}

static cv::Mat clipout_target_image_remain(cv::Mat target, int &l, int &r, int &t, int &b)
{
    int i = 0;
    int j = 0;
    int w = target.cols;
    int h = target.rows;
    int left = w;
    int right = 0;
    int top = h;
    int bottom = 0;
    for( ; i < h; i ++ )
    {
        for( j = 0; j < w; j ++ )
        {
            unsigned char dat = target.data[i * w + j];
            if(dat == 255)
            {
                left = left > j ? j : left;
                right = right < j ? j : right;
                top = top > i ? i : top;
                bottom = bottom < i ? i : bottom;
            }
        }
    }
    left = left - 2;
    top = top - 2;
    right = right + 2;
    bottom = bottom + 2;
    int tw = right - left;
    int th = bottom - top;


    cv::Mat out = cv::Mat::zeros(th, tw, CV_8U);
    for( i = 0; i < th; i ++ )
    {
        memcpy(out.data + i * tw, target.data + ((i + top) * w) + left, tw);
    }

    l = left;
    r = right;
    t = top;
    b = bottom;
    return out;
}

static void imrotate(Mat& img, Mat& newIm, double angle){
    int len = max(img.cols, img.rows);
    Point2f pt(len/2.,len/2.);
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(img,newIm,r,Size(len,len));
}

void rotateDNN()
{
    char fullpath[128] = {0};
    char fullpathrt[128] = {0};
    char filename[64] = {0};
    char filerotate[64] = {0};
    sCurrentSaveId = 0;
    for( int i = 0; i < SAMPLES; i ++ )
    {
        sCurrentId = i;
        memset(fullpath, 0, 128);
        memset(filename, 0, 64);
        //sprintf(filename, "col_%d.jpg", i);
        sprintf(filename, "dnn/%d.png", i);
        sprintf(filerotate, "dnnrt/%d.png", i);
        sprintf(fullpath, "../../libs/%s", filename);
        sprintf(fullpathrt, "../../libs/%s", filerotate);

        sColor = cv::imread(fullpath);

        if(sColor.cols == 0 || sColor.rows == 0)
        {
            continue;
        }

        imshow(filename, sColor);

        setMouseCallback(filename, onMouseHandler_A, NULL);

        while(1)
        {
            char k = waitKey(100);
            if(k == 27)
                break;
            else if(k == 'a')
            {
                sRotate = sColor.clone();
                imrotate(sColor, sRotate, 1);
                sColor = sRotate.clone();
                imshow(filename, sColor);
            }
            else if(k == 'd')
            {
                degree ++;
                sRotate = sColor.clone();
                imrotate(sColor, sRotate, -1);
                sColor = sRotate.clone();
                imshow(filename, sColor);
            }
            else if(k == 's')
            {
                n = 0;

                Mat newImg = sColor.clone();

//                double tan = 0.0;
//                double degree = 0;
//                if(pt2.x == 0 && pt2.y == 0 && pt1.x == 0 && pt1.y == 0) {
//                    degree = 0;
//                }
//                else {
//                    tan = (double)(pt2.y - pt1.y) / (pt2.x - pt1.x);
//                    degree = atan(tan);
//                }
//                degree = degree / 3.14159 * 360.0;
//                imrotate(sColor, newImg, degree);

                Mat gray(newImg.cols, newImg.rows, CV_8U);
                cv::cvtColor(newImg, gray, CV_RGB2GRAY);
                Mat binary = gray.clone();
                threshold(gray, binary, 160, 255, THRESH_BINARY);
                int l,t,r,b;
                Mat target = clipout_target_image_remain(binary, l, r, t, b);

                cv::imwrite(fullpathrt, target);

                pt2 = Point(0, 0);
                pt1 = Point(0, 0);
            }

        }
        cv::destroyWindow(std::string(filename));

    }
}
