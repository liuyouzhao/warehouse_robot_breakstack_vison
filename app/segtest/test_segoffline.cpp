#include "vt_io.h"
#include "vt_visual.h"
#include "vt_pointCloud.h"
#include "opencv2/opencv.hpp"
#include "vt_depthTrans.h"
#include <time.h>
#include <algorithm>
#include "boxIsolate.h"
#include "PlaneFit2.h"

using namespace vt;
using namespace std;
using namespace cv;

#define _isnan std::isnan

int main(int argc, char **argv)
{
    const char* keys =
    {
        "{    p|  path  |../app/dat/list.conf| output path}"
        "{    w|width   |752        |file name  }"
        "{    h|height  |480        |file name  }"
        "{    p|height  |480        |file name  }"
        "{    n|height  |480        |file name  }"
        "{    m|multiframes |1      |file name  }"
    };
    //char* argv1[] = { "main", "-w=752", "-h=480",/* "-p=D:\\workspace\\Cam2Arm\\Save3D\\data\\imglist.txt"*/"-p=D:\\workspace\\Segmentation3D\\SegOnLine\\data\\04124\\imglist.txt" };
    //int argc = sizeof(argv)/sizeof(argv[0]);
    CommandLineParser parser(argc, argv, keys);
    string pathWorld = parser.get<string>("p");
    const float pi = 3.14159265;
    int width = parser.get<int>("w");
    int height = parser.get<int>("h");
    int multiframes = parser.get<int>("m");
    int boxWidth = 520;
    int boxHeight = 360;
    std::vector<std::string> imglist = readList(pathWorld);
    Mat boxbottom;
    float bottom[3];
    loadBinary("bottom", bottom);
    boxbottom = Mat(3, 1, CV_32F, bottom);
    for (int i = 0; i < imglist.size(); i += multiframes)
    {
        clock_t t0 = clock();
        Mat_<Vec3f> world(height, width);
        vector< Mat_<Vec3f> > worlds;
        for (int j = 0; j < multiframes; j++)
        {
            Mat_<Vec3f> world(height, width);
            loadBinary(imglist[i+j].c_str(), world.data);
            worlds.push_back(world);
        }
        multiFrames(worlds, world, 10);
        //continue;
        vector<Mat> wall;
        //if(0)
        {
            Rect rt = boxEst((float*)world.data, (float*)boxbottom.data, world.cols, world.rows, 280, 20, 620, 440, wall);
            if (rt.width == 0)
                continue;
            world = Mat(world, rt).clone();
            setBorder(world);
            removePointsOnPlane(world, boxbottom);
        }

        Mat_<short> depth(world.size(), CV_16S);
        for (int i = 0; i < world.total(); i++)
        {
            float& x = world(i)(2);
            if (_isnan(x))
                depth(i) = -1000;
            else
                depth(i) = x;
        }
        imshow("show", toGray(depth));
        waitKey(0);
        PlaneFit2 ns(depth, world, wall);
        string result = ns.run();
        printf("%s\n", (char*)result.c_str());
        printf("%d; ", (int)(clock() - t0));
        waitKey(0);
    }
    destroyWindow("result");
    destroyWindow("depth");
    //_CrtDumpMemoryLeaks();
}
