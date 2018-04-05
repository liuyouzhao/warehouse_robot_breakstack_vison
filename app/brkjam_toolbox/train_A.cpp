#include "opencv2/opencv.hpp"

#define SAMPLES 100
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



static int JP = 10;
static int MW = 500 / JP;
static int TH = T_HEIGHT / JP * 2;
static int DT = 2;
static int TM = 0;
static Point LT = Point(0, 0);
static int last_index = -1;
static int STRIP = MW * TH * DT;

static void onMouseHandler_A(int event, int x, int y, int flag, void* pA)
{
    float *A = (float*)pA;
    int dt = 0;

    switch (event)
    {
    case EVENT_LBUTTONUP:
    {
        dt = 1;

        int x1 = x - LT.x;
        int y1 = y - LT.y;
        x1 /= JP;
        y1 /= JP;

        int index = (y1 * MW + x1) * dt;

        if(last_index == -1)
        {
            last_index = index;
            printf("From: %d\n", last_index);
            return;
        }


        A[last_index * STRIP + index] += 1.0;

        printf("[%d]->[%d]\n", last_index, index);

        last_index = index;


    } break;
    case EVENT_RBUTTONUP:
    {
        dt = 2;

        int x1 = x - LT.x;
        int y1 = y - LT.y;
        x1 /= JP;
        y1 /= JP;
        int index = (y1 * MW + x1) * dt;

        if(last_index == -1)
        {
            last_index = index;
            printf("From: %d\n", last_index);
            return;
        }

        printf("[%d]->[%d]\n", last_index, index);
        A[last_index * STRIP + index] += 1.0;
        last_index = index;
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

void train_A()
{
    int len = STRIP * STRIP;
    float *A = (float*) malloc(len * sizeof(float));
    memset(A, 0, len * sizeof(float));

    char fullpath[128] = {0};
    char filename[64] = {0};
    sCurrentSaveId = 0;
    for( int i = 0; i < SAMPLES; i ++ )
    {
        sCurrentId = i;
        memset(fullpath, 0, 128);
        memset(filename, 0, 64);
        //sprintf(filename, "col_%d.jpg", i);
        sprintf(filename, "dnn/%d.png", i);
        sprintf(fullpath, "../../libs/%s", filename);

        sColor = cv::imread(fullpath);

        if(sColor.cols == 0 || sColor.rows == 0)
        {
            continue;
        }

        Mat gray(sColor.cols, sColor.rows, CV_8U);
        cv::cvtColor(sColor, gray, CV_RGB2GRAY);
        Mat binary = gray.clone();
        threshold(gray, binary, 140, 255, THRESH_BINARY);
        int l,t,r,b;
        Mat target = clipout_target_image_remain(binary, l, r, t, b);

        imshow(filename, target);
        setMouseCallback(filename, onMouseHandler_A, A);

        while(1)
        {
            char k = waitKey(100);
            if(k == 27)
                break;
            else if(k == 'C')
            {
                printf("Restart\n");
                last_index = -1;
            }
        }
        cv::destroyWindow(std::string(filename));

    }

    double whole = 0;
    for(int i = 0; i < len; i ++)
    {
        whole += A[i];
    }
    for(int i = 0; i < len; i ++)
    {
        A[i] /= whole;
    }

    writeFloatArrayToFile("../../libs/A.m", A, len);

    free(A);
}
