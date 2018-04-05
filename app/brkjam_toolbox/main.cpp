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

static void projectPixels(Mat binary, int *result)
{
    int sumVert[binary.cols];
    int sumHori[binary.rows];

    memset(sumVert, 0, sizeof(int) * binary.cols);
    memset(sumHori, 0, sizeof(int) * binary.rows);
    for(int i = 0; i < binary.rows; i ++)
    {
        for(int j = 0; j < binary.cols; j ++)
        {
            int v = binary.at<unsigned char>(i, j);
            sumVert[j] += v == 255 ? 1 : 0;
            sumHori[i] += v == 255 ? 1 : 0;
        }
    }
    for(int i = 0; i < binary.cols; i ++)
    {
        if(i <= (binary.cols * NOISE_M) || i >= (binary.cols * (1.0 - NOISE_M)))
            result[i] = sumVert[i];
        else
            result[i] = 0;
    }
    for(int i = 0; i < binary.rows; i ++)
    {
        if(i <= (binary.rows * NOISE_M) || i >= (binary.rows * (1.0 - NOISE_M)))
            result[i + binary.cols] = sumHori[i];
        else
            result[i + binary.cols] = 0;
    }
}

static void handleProjectPixel(int x, int y, int w, int h, Mat color, const char *title, int index)
{
    Rect region = Rect(x, y, w, h);
    Mat gray(color.cols, color.rows, CV_8U);
    Mat binary = gray.clone();
    threshold(gray, binary, 140, 255, THRESH_BINARY_INV);

    Mat croped = binary(region);
    invertBinary(croped);

    int vecWidth = region.width / 4;
    int vecHeight = region.height / 4;
    Mat cropedResized = Mat(vecWidth, vecHeight, CV_8U);
    Size size(vecWidth, vecHeight);
    resize(croped, cropedResized, size);

    int vecDesc[cropedResized.cols + cropedResized.rows];
    memset(vecDesc, 0, cropedResized.cols + cropedResized.rows);
    projectPixels(cropedResized, vecDesc);

    char filename[64] = {0};
    char cmd[128] = {0};
    sprintf(cmd, "mkdir -p ../../libs/%s", title);
    system(cmd);

    sprintf(filename, "../../libs/%s/%d.proj", title, index);
    writeIntArrayToFile(filename, vecDesc, cropedResized.cols + cropedResized.rows);
}

static void onMouseHandler(int event, int x, int y, int flag, void* param)
{
    static int began = 0;
    static std::vector<Point> points;
    switch (event)
    {
    case EVENT_LBUTTONUP:
    {
        sRegion = Rect(x, y, T_WIDTH + T_MAGIN * 2, T_HEIGHT + T_MAGIN * 2);

        Mat gray(sColor.cols, sColor.rows, CV_8U);
        cv::cvtColor(sColor, gray, CV_RGB2GRAY);
        Mat binary = gray.clone();

        threshold(gray, binary, 140, 255, THRESH_BINARY_INV);
        //imshow("bin", binary);

        Mat croped = binary(sRegion);
        invertBinary(croped);

        int vecWidth = sRegion.width / 4;
        int vecHeight = sRegion.height / 4;
        Mat cropedResized = Mat(vecWidth, vecHeight, CV_8U);
        Size size(vecWidth, vecHeight);
        resize(croped, cropedResized, size);
        //imshow("cropedResized", cropedResized);

        int vecDesc[cropedResized.cols + cropedResized.rows];
        memset(vecDesc, 0, cropedResized.cols + cropedResized.rows);
        projectPixels(cropedResized, vecDesc);

        char filename[64] = {0};
        sprintf(filename, "../../libs/%s%d", FILE_NAME, sCurrentSaveId);
        writeIntArrayToFile(filename, vecDesc, cropedResized.cols + cropedResized.rows);
        sCurrentSaveId ++;
    }
        break;
    case EVENT_RBUTTONDOWN:
        began = 1;
        break;
    case EVENT_MOUSEMOVE:
        if(began)
        {
            points.push_back(Point(x, y));
        }
        break;
    case EVENT_RBUTTONUP:
    {
        int len = points.size() * 8;
        char buffer[len];
        memset(buffer, 0, len);
        for(int i = 0; i < points.size(); i ++)
        {
            sprintf(buffer, "%s%d %d ", buffer, points[i].x, points[i].y);
        }
        printf("%s [%d]\n", buffer, points.size());
        FILE *f = fopen("points_track", "w");
        if(f == NULL)
        {
            return;
        }
        fwrite(buffer, 1, strlen(buffer), f);
        fclose(f);
    }
        break;
    }
}

#if 0
int main()
{
    Mat img = Mat(300, 300, CV_8U);
    char *filename = "ok";
    imshow(filename, img);
    setMouseCallback(filename, onMouseHandler, NULL);
    while(1)
    {
        char k = waitKey(100);
        if(k == 27)
            break;
    }
    cv::destroyWindow(std::string(filename));
}
#endif
#if 0
int main(int argc, char *argv[])
{
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

        imshow(filename, sColor);
        //setMouseCallback(filename, onMouseHandler, NULL);

        int h = T_HEIGHT + T_MAGIN * 2;
        int w = T_WIDTH + T_MAGIN * 2;
        for(int y = 0; y < sColor.rows - h; y ++) {
            for(int x = 0; x < sColor.cols - w; x ++) {

                handleProjectPixel(x, y,
                                   w, h,
                                   sColor,
                                   filename,
                                   y * sColor.cols + x);
            }
        }


        while(1)
        {
            char k = waitKey(100);
            if(k == 27)
                break;
        }
        cv::destroyWindow(std::string(filename));
    }

    return 0;
}
#endif
#if 1
void rotateDNN();
void run_hmm();
void train_B();
int svm_B();
int test_B(Mat mat);
int test_folder();
int test_image(std::string file);
void binary_go(const char *folder_src, const char *folder_dst);

int main(int argc, char *argv[])
{
    //train_B();
    //svm_B();

//    Mat testMat = imread("../../libs/dnnrt/9.png.vec/-471000.png");
//    test_B(testMat);
    //test_folder();
    run_hmm();
    //binary_go("../../libs/origin", "../../libs/binary");

    //test_image(std::string("../../libs/binary/col_0.jpg"));

    //rotateDNN();
}
#endif
