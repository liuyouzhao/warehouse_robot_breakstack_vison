#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <vector>

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

static int readFileList(char *basePath, int (*filecallback)(const char*, const char*, const char*), const char *dst)
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
            filecallback(basePath, ptr->d_name, dst);
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

static int __testFolderCallback(const char *base, const char *name, const char* dst)
{
    char filename[128] = {0};
    sprintf(filename, "%s/%s", base, name);
    Mat color = imread(filename);
    if(color.cols == 0 || color.rows == 0)
    {
        return 0;
    }

    Mat gray(color.cols, color.rows, CV_8U);

    cvtColor(color, gray, CV_RGB2GRAY);

    Mat binary = gray.clone();
    threshold(gray, binary, 110, 255, THRESH_BINARY_INV);

    char path[128] = {0};
    sprintf(path, "%s/%s", dst, name);

    imwrite(path, binary);

    return 0;
}

void binary_go(const char *folder_src, const char *folder_dst)
{

    readFileList((char*)folder_src, __testFolderCallback, (char*)folder_dst);


}


