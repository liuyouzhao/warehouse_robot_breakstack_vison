/**
 * Samples generator. For svm multi-classifier.
 * [1] We capture and pick out the horizontal rects as positive-samples and then the rests are negative-samples.
 * [2] We capture and pick out the vertical rects as positive-samples and then the rests are negative.
 *
 * First, we need user to click on the image for marking all correct locations as the start points
 * where the positive samples are picked. What's more, we purposely expend the region around every given point so that
 * the positive locations have some spacial flexibility.
 * Then the rests locations are all denote as negative.
 * */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <vector>

using namespace std;
#include "opencv2/opencv.hpp"

#define SAMPLES 100
#define T_MAGIN 1

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

static Mat sColor;
static Mat sShow;

static std::vector<Point> sPositivePoints1;
static std::vector<Point> sPositivePoints2;

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

static int readIntArrayFromFile(const char *filename, int *arr, int *len)
{
    char buffer[1024];
    memset(buffer, 0, 1024);

    FILE *f = fopen(filename, "r");
    if(f == NULL)
    {
        return -1;
    }
    *len = fread(buffer, 1, 1024, f);
    fclose(f);

    int l = 0;
    int index = 0;
    for(int i = 0; i < *len; i ++)
    {
        char c = buffer[i];
        char num[32] = {0};
        if(c == ' ')
        {
            memcpy(num, (char*) buffer + l, i - l);
            arr[index++] = atoi(num);
            l = i + 1;
        }
    }
    *len = index;
    for(int i = 0; i < *len; i ++)
    {
        printf("%d ", arr[i]);
    }
    printf("\n");
    return 0;
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
    if(x + w >= color.cols) {
        w -= (x + w - color.cols);
    }
    if(y + h >= color.rows) {
        h -= (y + h - color.rows);
    }
    Rect region = Rect(x, y, w, h);
    Mat croped = color(region);
    //invertBinary(croped);

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
    sprintf(cmd, "mkdir -p ../../libs/%s.vec", title);
    system(cmd);

    sprintf(filename, "../../libs/%s.vec/%d.proj", title, index);
    writeIntArrayToFile(filename, vecDesc, cropedResized.cols + cropedResized.rows);
}

static void handleSaveRegion(int x, int y, int w, int h, Mat color, const char *title, int index)
{
    if(x + w >= color.cols) {
        w -= (x + w - color.cols);
    }
    if(y + h >= color.rows) {
        h -= (y + h - color.rows);
    }
    Rect region = Rect(x, y, w, h);
    Mat croped = color(region);

    char filename[64] = {0};
    char cmd[128] = {0};
    sprintf(cmd, "mkdir -p ../../libs/%s.vec", title);
    system(cmd);

    sprintf(filename, "../../libs/%s.vec/%d.png", title, index);

    imwrite(filename, croped);
}

static void onMouseHandler(int event, int x, int y, int flag, void* file)
{
    switch (event)
    {
    case EVENT_LBUTTONUP:
    {
        sPositivePoints1.push_back(Point(x, y));

        //sShow = sColor.clone();

        for(int i = 0; i < sPositivePoints1.size(); i ++)
        {
            Point pt = sPositivePoints1[i];
            circle(sShow, pt, 8, Scalar(0, 0, 255), 5, 8, 0);
        }
        imshow("show", sShow);
    } break;
    case EVENT_RBUTTONUP:
    {
        printf("%d %d\n", x, y);
        sPositivePoints2.push_back(Point(x, y));

        //sShow = sColor.clone();

        for(int i = 0; i < sPositivePoints2.size(); i ++)
        {
            Point pt = sPositivePoints2[i];
            circle(sShow, pt, 8, Scalar(255, 0, 0), 5, 8, 0);
        }
        imshow("show", sShow);
    } break;
    }
}
double gaussrand(int NSUM)
{
    double x = 0;
    int i;
    for(i = 0; i < NSUM; i++)
    {
        x += (double)rand() / RAND_MAX;
    }

    x -= NSUM / 2.0;
    x /= sqrt(NSUM / 12.0);

    return x;
}


static int readFileList(char *basePath, int (*filecallback)(const char*, const char*))
{
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            filecallback(basePath, ptr->d_name);
        }
        else if(ptr->d_type == 10)    ///link file
            printf("d_name:%s/%s\n",basePath,ptr->d_name);
        else if(ptr->d_type == 4)    ///dir
        {

        }
    }
    closedir(dir);
    return 1;
}

static void generateSamples(const char *filename)
{
    static int radio = 3;
    int tw = T_WIDTH + T_MAGIN * 2;
    int th = T_HEIGHT + T_MAGIN * 2;
    for(int i = 0; i < sPositivePoints1.size(); i ++)
    {
        for(int j = 0; j < 20; j ++)
        {
            double grand = gaussrand(25);
            int dist = radio * grand;
            int dir = rand() % 360;
            double angle = (double)dir / 360.0 * 3.14159 * 2;
            int cx = sPositivePoints1[i].x;
            int cy = sPositivePoints1[i].y;
            int x = cos(angle) * dist;
            int y = sin(angle) * dist;
            Point pt = Point(cx + x, cy + y);
            pt.x = pt.x < 0 ? 0 : pt.x;
            pt.y = pt.y < 0 ? 0 : pt.y;
            //handleProjectPixel(pt.x, pt.y, tw, th, sColor, filename, i * 100 + j);
            handleSaveRegion(pt.x, pt.y, tw, th, sColor, filename, i * 100 + j);
        }
    }

    for(int i = 0; i < sPositivePoints2.size(); i ++)
    {
        for(int j = 0; j < 20; j ++)
        {
            double grand = gaussrand(25);
            int dist = radio * grand;
            int dir = rand() % 360;
            double angle = (double)dir / 360.0 * 3.14159 * 2;
            int cx = sPositivePoints2[i].x;
            int cy = sPositivePoints2[i].y;
            int x = cos(angle) * dist;
            int y = sin(angle) * dist;
            Point pt = Point(cx + x, cy + y);
            pt.x = pt.x < 0 ? 0 : pt.x;
            pt.y = pt.y < 0 ? 0 : pt.y;
            //handleProjectPixel(pt.x, pt.y, th, tw, sColor, filename, (i * 10000 + j));
            handleSaveRegion(pt.x, pt.y, th, tw, sColor, filename, (i * 10000 + j));
        }

    }

    /* Generate negative samples for all*/
    for(int i = 0; i < 500; i ++)
    {
        int x = rand() % (sColor.cols - tw);
        int y = rand() % (sColor.rows - th);
        int toonear1 = 0;
        int toonear2 = 0;
        for(int j = 0; j < sPositivePoints1.size(); j ++)
        {
            Point pt = sPositivePoints1[j];
            int dist = sqrt((double)(pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y));
            if(dist <= radio * 2)
            {
                toonear1 = 1;
                break;
            }

        }

        for(int j = 0; j < sPositivePoints2.size(); j ++)
        {
            Point pt = sPositivePoints2[j];
            int dist = sqrt((double)(pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y));
            if(dist <= radio * 2)
            {
                toonear2 = 1;
                break;
            }
        }
        if(toonear1 && toonear2)
        {
            i --;
            continue;
        }
        else if(toonear1)
        {
            //handleProjectPixel(x, y, th, tw, sColor, filename, -i * 1000);
            handleSaveRegion(x, y, th, tw, sColor, filename, -i * 1000);
            continue;
        }
        else if(toonear2)
        {
            //handleProjectPixel(x, y, tw, th, sColor, filename, -i);
            handleSaveRegion(x, y, tw, th, sColor, filename, -i);
            continue;
        }
        //handleProjectPixel(x, y, tw, th, sColor, filename, -i);
        //handleProjectPixel(x, y, th, tw, sColor, filename, -i * 1000);

        handleSaveRegion(x, y, tw, th, sColor, filename, -i);
        handleSaveRegion(x, y, th, tw, sColor, filename, -i * 1000);
    }

    sPositivePoints1.clear();
    sPositivePoints2.clear();
}

static Mat trainingData1;
static Mat trainingData2;
static Mat trainingImages1;
static Mat trainingImages2;
static Mat classes1;
static Mat classes2;
static vector<int> trainingLabels1;
static vector<int> trainingLabels2;
static int __TW = 77;
static int __TH = 55;


static int __filecallback(const char *base, const char *name)
{
    char path[128] = {0};
    sprintf(path, "%s/%s", base, name);
    Mat read = imread(std::string(path));

    if(read.cols == 0 || read.rows == 0)
    {
        return 0;
    }

    cvtColor(read, read, CV_RGB2GRAY);

    if(read.cols > read.rows)
    {
        if(read.cols != __TW || read.rows != __TH)
        {
            resize(read, read, Size(__TH, __TW));
        }
        if(name[0] == '-')
        {
            trainingLabels1.push_back(-1);
        }
        else
        {
            trainingLabels1.push_back(1);
        }
        read = read.reshape(1, 1);
        trainingImages1.push_back(read);
    }
    else if(read.cols < read.rows)
    {
        if(read.cols != __TH || read.rows != __TW)
        {
            resize(read, read, Size(__TW, __TH));
        }
        if(name[0] == '-')
        {
            trainingLabels2.push_back(-1);
        }
        else
        {
            trainingLabels2.push_back(1);
        }
        read = read.reshape(1, 1);
        trainingImages2.push_back(read);
    }
    return 0;
}


static vector<float> positives1;
static vector<float> negatives1;
static vector<float> positives2;
static vector<float> negatives2;
float test_B(Mat b, int t = 0)
{
    static int svm1_load = 0;
    static int svm2_load = 0;
    static CvSVM svm1;
    static CvSVM svm2;
    float value = 0.0f;
    if(b.cols > b.rows)
    {
        if(b.cols != __TW || b.rows != __TH)
        {
            resize(b, b, Size(__TH, __TW));
        }
        svm1.load("../../libs/svm-1.xml");
        svm1_load = 1;
        cvtColor(b, b, CV_RGB2GRAY);
        b = b.reshape(1, 1);
        b.convertTo(b, CV_32FC1);
        float v = svm1.predict(b);
        value = svm1.predict(b, true);
        value = v > 0 ? (value > 0 ? value : -value) : (value > 0 ? -value : value);
        if(t == 1)
        {
            positives1.push_back(value);
        }
        else if(t == -1)
        {
            negatives1.push_back(value);
        }
    }
    else if(b.cols < b.rows)
    {
        if(b.cols != __TH || b.rows != __TW)
        {
            resize(b, b, Size(__TW, __TH));
        }
        svm2.load("../../libs/svm-2.xml");
        svm2_load = 1;

        cvtColor(b, b, CV_RGB2GRAY);
        b = b.reshape(1, 1);
        b.convertTo(b, CV_32FC1);
        float v = svm2.predict(b);
        value = v * svm2.predict(b, true);
        value = v > 0 ? (value > 0 ? value : -value) : (value > 0 ? -value : value);
        if(t == 1)
        {
            positives2.push_back(value);
        }
        else if(t == -1)
        {
            negatives2.push_back(value);
        }
    }
    else if(b.cols == 0 || b.rows == 0)
    {
        return -100;
    }

    return value > 0 ? 1.0 : -1.0;
}

float test_B_Dist(Mat b, int t = 0)
{
    static int svm1_load = 0;
    static int svm2_load = 0;
    static CvSVM svm1;
    static CvSVM svm2;
    float value = 0.0f;


    if(svm1_load == 0)
    {
        svm1.load("../../libs/svm-1.xml");
        svm1_load = 1;
    }
    if(svm2_load == 0)
    {
        svm2.load("../../libs/svm-2.xml");
        svm2_load = 1;
    }

    if(b.cols > b.rows)
    {
        if(b.cols != __TW || b.rows != __TH)
        {
            resize(b, b, Size(__TH, __TW));
        }

        cvtColor(b, b, CV_RGB2GRAY);
        b = b.reshape(1, 1);
        b.convertTo(b, CV_32FC1);
        float v = svm1.predict(b);
        value = svm1.predict(b, true);
        value = v > 0 ? (value > 0 ? value : -value) : (value > 0 ? -value : value);
        //printf("%f\n", value);
        if(t == 1)
        {
            positives1.push_back(value);
        }
        else if(t == -1)
        {
            negatives1.push_back(value);
        }
    }
    else if(b.cols < b.rows)
    {
        if(b.cols != __TH || b.rows != __TW)
        {
            resize(b, b, Size(__TW, __TH));
        }

        cvtColor(b, b, CV_RGB2GRAY);
        b = b.reshape(1, 1);
        b.convertTo(b, CV_32FC1);
        float v = svm2.predict(b);
        value = v * svm2.predict(b, true);
        value = v > 0 ? (value > 0 ? value : -value) : (value > 0 ? -value : value);
        //printf("%f\n", value);
        if(t == 1)
        {
            positives2.push_back(value);
        }
        else if(t == -1)
        {
            negatives2.push_back(value);
        }
    }
    else if(b.cols == 0 || b.rows == 0)
    {
        return -100;
    }

    return value;
}

int svm_B()
{
    Mat s1, s2;

    readFileList("../../libs/binary/train/", __filecallback);

    Mat(trainingImages1).copyTo(trainingData1);
    trainingData1.convertTo(trainingData1, CV_32FC1);
    Mat(trainingLabels1).copyTo(classes1);

    Mat(trainingImages2).copyTo(trainingData2);
    trainingData2.convertTo(trainingData2, CV_32FC1);
    Mat(trainingLabels2).copyTo(classes2);

    CvSVMParams SVM_params;
    SVM_params.svm_type = CvSVM::C_SVC;
    SVM_params.kernel_type = CvSVM::LINEAR;
    SVM_params.degree = 0;
    SVM_params.gamma = 1;
    SVM_params.coef0 = 0;
    SVM_params.C = 1;
    SVM_params.nu = 0;
    SVM_params.p = 0;
    SVM_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 1000, FLT_EPSILON);

    CvSVM svm1;
    svm1.train(trainingData1, classes1, Mat(), Mat(), SVM_params);
    svm1.save("../../libs/svm-1.xml");

    CvSVM svm2;
    svm2.train(trainingData2, classes2, Mat(), Mat(), SVM_params);
    svm2.save("../../libs/svm-2.xml");

    while(1)
    {
        char k = waitKey(100);
        if(k == 27)
            break;
    }
    return 0;
}

int train_B()
{
    char fullpath[128];
    char filename[64];
    for( int i = 0; i < SAMPLES; i ++ )
    {
        memset(fullpath, 0, 128);
        memset(filename, 0, 64);
        sprintf(filename, "binary/col_%d.jpg", i);
        sprintf(fullpath, "../../libs/%s", filename);

        sColor = cv::imread(fullpath);
        sShow = sColor.clone();
        if(sColor.cols == 0 || sColor.rows == 0)
        {
            continue;
        }

        imshow("show", sShow);
        setMouseCallback("show", onMouseHandler, filename);

        while(1)
        {
            char k = waitKey(100);
            if(k == 27)
                break;
            if(k == 's')
            {
                generateSamples(filename);
            }
        }
        cv::destroyWindow(std::string(filename));
    }
}

static int __testFolderCallback(const char *base, const char *name)
{
    char filename[128] = {0};
    sprintf(filename, "%s/%s", base, name);
    Mat testMat = imread(filename);
    if(testMat.cols == 0 || testMat.rows == 0)
    {
        return 0;
    }

    static int correct = 0;
    static int whole = 0;

    if(name[0] == '-')
    {
        float t = test_B(testMat, -1);
        if(t == -1.0f)
        {
            correct ++;
        }
    }
    else
    {
        float t = test_B(testMat, 1);
        if(t == 1.0f)
        {
            correct ++;
        }
    }
    whole ++;

    //printf("%d / %d\n", correct, whole);

    return 0;
}


void writeOctaveCommon(const char *filename, string name, vector<float> X);
int test_folder()
{
    positives1.clear();
    negatives1.clear();
    positives2.clear();
    negatives2.clear();
    readFileList("../../libs/binary/train/", __testFolderCallback);

    writeOctaveCommon("../../libs/svm_p_1.mat", "svmp1", positives1);
    writeOctaveCommon("../../libs/svm_n_1.mat", "svmn1", negatives1);

    writeOctaveCommon("../../libs/svm_p_2.mat", "svmp2", positives2);
    writeOctaveCommon("../../libs/svm_n_2.mat", "svmn2", negatives2);

    return 0;
}

int test_image(string file)
{
    int tw = T_WIDTH + T_MAGIN * 2;
    int th = T_HEIGHT + T_MAGIN * 2;
    Mat image = imread(file);
    Mat show = image.clone();
    for(int i = 0; i < image.rows; i ++)
    {
        for(int j = 0; j < image.cols; j ++)
        {
            int x = j;
            int y = i;

            double preHori = 0.0;
            double preVert = 0.0;
            /// horizontal
            if(x + tw < image.cols && y + th < image.rows)
            {
                Mat clip = image(Rect(x, y, tw, th));
                /// predict
                preHori = test_B(clip);
            }
            else
            {
                preHori = -1.0;
            }
            /// vertical
            if(x + th < image.cols && y + tw < image.rows)
            {
                Mat clip = image(Rect(x, y, th, tw));
                /// predict
                preVert = test_B(clip);
            }
            else
            {
                preVert = -1.0;
            }

            if(preHori == 1.0 && preVert == -1.0)
            {
                show.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
            }
            else if(preHori == -1.0 && preVert == 1.0)
            {
                show.at<Vec3b>(y, x) = Vec3b(255, 0, 0);
            }
            else
            {
                show.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
            }
        }
    }

    imshow("show", show);

    imwrite(file + string(".obs.png"), show);
    while(1)
    {
        char k = waitKey(100);
        if(k == 27)
            break;
    }
    return 0;
}
