#include "vt_io.h"
#include "vt_visual.h"
#include "vt_PointCloud.h"
#include "opencv2\opencv.hpp"
#include "vt_depthTrans.h"
#include <time.h>
#include <algorithm>
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "Rotation.h"

using namespace vt;
using namespace std;
using namespace cv;


int detect(vector<Mat_<short>>& depths, vector<float>& pkpt, vector<float>& ptv)
{    
    const float pi = 3.14159265;
    int width = depths[0].cols;
    int height = depths[0].rows;
    int boxWidth = 520;
    int boxHeight = 360;
    Mat boxbottom;
    float bottom[3];
    loadBinary("bottom", bottom);
    double handeye[24];
    loadBinary("handeye", handeye);
    Mat R, t, tR, tt;
    {
        float temp[24];
        for (int i = 0; i < 24; i++)
            temp[i] = handeye[i];
        R = Mat(3, 3, CV_32F, temp).clone();
        t = Mat(3, 1, CV_32F, temp + 9).clone();
        tR = Mat(3, 3, CV_32F, temp + 12).clone();
        tt = Mat(3, 1, CV_32F, temp + 21).clone();
    }
    Mat intri[2];
    {
        FileStorage fs("intrinsics.yml", FileStorage::READ);
        fs["M1"] >> intri[0];
        fs["D1"] >> intri[1];
        for (int i = 0; i < 2; i++)
        {
            intri[i].convertTo(intri[i], CV_32F);
        }
    }
    Mat_<Vec3f> world(height, width);  
    boxbottom = Mat(3, 1, CV_32F, bottom);
    vector<Mat_<Vec3f>> worlds;       
             
    for (int j = 0; j < depths.size(); j++)
    {
        Mat_<Vec3f> world(height, width);
        depth2World(depths[j], world, (float*)intri[0].data, (float*)intri[1].data);
        worlds.push_back(world);
    }
        multiFrames(worlds, world, 10);
        //continue;
        vector<Mat> wall;
        Rect rt;
        {
            rt = boxEst((float*)world.data, (float*)boxbottom.data, world.cols, world.rows, 280, 20, 620, 440, wall);
            if (rt.width == 0)
                return 0;
            world = Mat(world, rt).clone();
            setBorder(world);
            removePointsOnPlane(world, boxbottom);
        }
        Mat_<short> depth(world.size(), CV_16S);
        for (int i = 0; i < world.total(); i++)
        {
            float& x = world(i)(2);
            if (_isnan(x))
                depth(i) = -10000;
            else
                depth(i) = x;
        }

        PlaneFit2 ns(depth, world, wall);
        vector<float> pkpt, pkv;
        Mat_<double> ptR;
        ns.run(pkpt, ptR, pkv);
        Mat ptR2;
        ptR.convertTo(ptR2, CV_32F);
        Mat Re = R*ptR2*tR.t();
        Mat P = (Mat_<float>(3, 1) << pkpt[0], pkpt[1], pkpt[2]);
        Mat newP = (R*P + t) / 1000;
        newP = (R*P + t - Re*tt) / 1000;
        pkpt[0] = newP.at<float>(0);
        pkpt[1] = newP.at<float>(1);
        pkpt[2] = newP.at<float>(2);
        Re.convertTo(ptR, CV_64F);
        Vec4d q = eulerAngles2quaternion(rotationMatrixToEulerAngles(ptR));
        pkpt[3] = q[1]; pkpt[4] = q[2]; pkpt[5] = q[3]; pkpt[6] = q[0];
        for (int t = 0; t < 4; t++)
        {
            pkv[t*2] += rt.x;
            pkv[2 * t + 1] += rt.y;
        }

   
    //    _CrtDumpMemoryLeaks();
}


int main(int argc, char **argv)
{
    const char* keys =
    {
        "{    p|  path	|| output path}"        
        "{    w|width	|640		|file name  }"
        "{    h|height	|480		|file name  }"
        "{    p|height	|480		|file name  }"
        "{    n|height	|480		|file name  }"
        "{    m|multiframes	|1		|file name  }"
    };    
    //char* argv[] = { "main", "-w=752", "-h=480",/* "-p=D:\\workspace\\Cam2Arm\\Save3D\\data\\imglist.txt"*/"-p=D:\\workspace\\Segmentation3D\\SegOnLine\\data\\04124\\imglist.txt" };
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
        vector<Mat_<Vec3f>> worlds;
        for (int j = 0; j < multiframes; j++)
        {
            Mat_<Vec3f> world(height, width);
            loadBinary(imglist[i+j].c_str(), world.data);
            worlds.push_back(world);
        }       
        multiFrames(worlds, world, 10);
        //continue;
        vector<Mat> wall;
        {            
            Rect rt = boxEst((float*)world.data, (float*)boxbottom.data, world.cols, world.rows,280, 20, 620, 440, wall);            
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

        PlaneFit2 ns(depth, world, wall);
        vector<float> pkpt, pkv;
        Mat_<double> ptR;
       // string result = ns.run(pkpt, pkv);
        ns.run(pkpt, ptR, pkv);
        /*printf(result.c_str());
        printf("%d; ", clock() - t0);*/
        waitKey(0);               
    }
    destroyWindow("result");
    destroyWindow("depth");
//    _CrtDumpMemoryLeaks();
}