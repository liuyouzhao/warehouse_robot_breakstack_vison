#include "PlaneFit2.h"
#include <iostream>
#include <string>
#include <algorithm>
#include "opencv2/core/core.hpp"
#include "vt_pointCloud.h"
#include "time.h"
#include "vt_visual.h"
#include "cam2Arm.h"

using namespace std;
using namespace cv;
using namespace vt;

#ifdef UNIX
struct PlaneDepth
{
    int idx;
    float depth;
};

static bool sort_func_i(const PlaneFit2::SortPair& a, const PlaneFit2::SortPair& b)
{
    return a.value.i > b.value.i;
}

static bool sort_func_f(const PlaneFit2::SortPair& a, const PlaneFit2::SortPair& b)
{
    return a.value.f > b.value.f;
}

static bool sort_func_depth(const PlaneDepth& a, const PlaneDepth& b)
{
    return a.depth < b.depth;
}
#endif

static int mergeTime = 0;
static float ZERO = 1e-5;
const float bin[] =
{ 0.9962, 0.9848 /*, 0.9659,0.9397*/};
PlaneFit2::PlaneFit2(cv::Mat& _depth, cv::Mat& _cloud, vector<Mat>& _wall,
        int _smallRegion, int _gradMax) :
        depth(_depth), cloud(_cloud), total(depth.rows * depth.cols), smallregion(
                _smallRegion), rows(_depth.rows), cols(_depth.cols), nextNeighbor(
                NULL), neighborBucket(NULL), average(NULL), regionIndex(NULL), gradMax(
                _gradMax), wall(_wall), nodes(NULL), rootNodes(NULL)
{
    regionsCount = total;
    binCount = sizeof(bin) / sizeof(bin[0]);

    //init buffer
    neighborBucket = new int[binCount];
    for (int i = 0; i < binCount; i++)
    {
        neighborBucket[i] = -1;
    }
    nextNeighbor = new int[2 * cols * rows];
    memset(nextNeighbor, 0, sizeof(int) * 2 * rows * cols);
    chain = new Node[total];
    for (int i = 0; i < total; i++)
    {
        chain[i].init(i);
    }
    tooSmall = 10;
    maxdistance = 1120;
    pIn = (vec3f*) cloud.data;
}

PlaneFit2::~PlaneFit2()
{
    delete[] nextNeighbor;
    delete[] neighborBucket;
    delete[] chain;

    delete[] average;

    delete[] regionIndex;
    if(nodes)
        delete[] nodes;

    if(rootNodes)
        delete[] rootNodes;
}

//void PlaneFit2::sortPlanes()
//{
//    int count = listc[0];
//    while (count<size-1)
//    {
//        int root = count;
//        planes.push_back(plane());
//        plane& p = planes.back();
//        p.root = root;
//        chain[root].planeIdx = planes.size() - 1;
//        while (chain[root].child != root)
//        {
//            mask[root] = 1;
//            p.pts.push_back(root);
//            root = chain[root].child;
//        }
//        cv::Mat mat(p.pts.size(), 3, CV_32F);
//        vec3f* pmat = (vec3f*)mat.data;
//        for (int i = 0; i < p.pts.size(); i++)
//        {
//            pmat[i] = pIn[p.pts[i]];
//        }
//        cv::Mat  planeMat = mat.inv(cv::DECOMP_SVD)*cv::Mat::ones(p.pts.size(), 1, CV_32F);
//        vec3f* pv = (vec3f*)planeMat.data;
//        p.x = pv->x; p.y = pv->y; p.z = pv->z;
//        p.disToOrigin();
//        count = listc[count];
//    }
//}
void
PlaneFit2::removeRoot(int idx)
{
    while (chain[idx].child != idx)
    {
        pMask[idx] = 0;
        idx = chain[idx].child;
    }
    pMask[idx] = 0;
    regionsCount--;
}

void
PlaneFit2::sortPlanes2()
{
    vector<SortPair> sortlist;
    for (int i = 0; i < total; i++)
    {
        if (pMask[i] == 0 || chain[i].parent != i)
            continue;
        if (chain[i].area < tooSmall)
        {
            removeRoot(i);
            continue;
        }
        int planeIdx = planes.size();
        planes.push_back(plane());
        plane& p = planes.back();
        p.root = i;
        chain[i].planeIdx = planeIdx;
        int idx = i;
        while (chain[idx].child != idx)
        {
            p.pts.push_back(idx);
            p.points.push_back(pIn[idx]);
            idx = chain[idx].child;
        }
        p.pts.push_back(idx);
        p.points.push_back(pIn[idx]);
        sortlist.push_back(SortPair(planeIdx, (int) p.points.size()));

    }
#ifdef UNIX
    sort(sortlist.begin(), sortlist.end(), sort_func_i);
#else
    sort(sortlist.begin(), sortlist.end(),
            [](const SortPair& a, const SortPair& b)
            {   return a.value.i>b.value.i;});
#endif
    int count = sortlist.size();
    for (int i = 0; i < count; i++)
    {
        if (sortlist[i].value.i < smallregion * 4)
            break;
        plane& p = planes[sortlist[i].idx];
        cv::Mat mat(p.points.size(), 3, CV_32F, p.points.data());
        clock_t t0 = clock();
        cv::Mat planeMat = estPlane(mat); //mat.inv(cv::DECOMP_SVD)*cv::Mat::ones(p.pts.size(), 1, CV_32F);
        //printf("%d ms;", clock() - t0);

        //vec3f* pv = (vec3f*)planeMat.data;
        p.set((float*) planeMat.data);
        /* {
         vec3f* ppp = (vec3f*)planeMat.data;
         vec3f pppp = pNormal[p.root].norm() + (*ppp).norm();
         float xxx = pNormal[p.root].align(*ppp);
         printf("%f %f %f %f %f; ", pppp.x, pppp.y, pppp.z, xxx, acos(xxx)/3.14159*180);
         }*/
        /*{
         Mat xxx = (mat*planeMat - Mat::ones(p.pts.size(), 1, CV_32F)) / p.len();
         double minv, maxv;
         minMaxIdx(xxx, &minv, &maxv);
         }*/
        p.disToOrigin();
        //if (p.dis < maxdistance)
        objs.push_back(sortlist[i].idx);
    }

    Mat show = Mat::zeros(depth.rows, depth.cols, CV_8U);
    for (int i = 0; i < objs.size(); i++)
    {
        int idx = planes[objs[i]].root;
        while (chain[idx].child != idx)
        {
            show.data[idx] = 255;
            idx = chain[idx].child;
            if (idx == 95152 || idx == 95151)
                printf("");
        }
    }
}
void
estVertice(Point2f pt[], Point2f center, float angle, int width)
{
    double _angle = angle * CV_PI / 180.;
    float b = (float) cos(_angle) * 0.5f;
    float a = (float) sin(_angle) * 0.5f;

    pt[0].x = center.x - a * width - b * width;
    pt[0].y = center.y + b * width - a * width;
    pt[1].x = center.x + a * width - b * width;
    pt[1].y = center.y - b * width - a * width;
    pt[2].x = 2 * center.x - pt[0].x;
    pt[2].y = 2 * center.y - pt[0].y;
    pt[3].x = 2 * center.x - pt[1].x;
    pt[3].y = 2 * center.y - pt[1].y;
}

void
PlaneFit2::sortPlanes3()
{
    Mat xxx = cloud;
    vector<SortPair> objForSort, objForSort2;
    map<int, Mat> pinvList;
    Mat_<float> pinv;
    for (int i = 0; i < objs.size(); i++)
    {
        int idx = objs[i];
        objForSort.push_back(SortPair(idx, pMean[planes[idx].root].z));
    }
#ifdef UNIX
    sort(objForSort.begin(), objForSort.end(), sort_func_f);
#else
    sort(objForSort.begin(), objForSort.end(),
            [](const SortPair& a, const SortPair& b)
            {   return a.value.f > b.value.f;});
#endif
    RotatedRect rt;
    int current;
    found = false;
    for (current = 0; current < objs.size(); current++)
    {
        planeId = objForSort[current].idx;
        plane& p = planes[planeId];
        int idx = p.root;
        float z0 = 1000 / p.squareroot;

        Mat show = Mat::zeros(cloud.size(), CV_8U);
        {
            for (int i = 0; i < p.pts.size(); i++)
                show.data[p.pts[i]] = 255;
         //   printf("");
        }

        Mat pointMat(p.points.size(), 3, CV_32F, p.points.data());
        vec3f n(p.x, p.y, p.z);
        n = n / n.len();
        if (n.z < 0)
        {
            n.x = -n.x;
            n.y = -n.y;
            n.z = -n.z;
            z0 = -z0;
        }
        float len2 = sqrt(n.x * n.x + n.y * n.y);
        Mat_<float> R =
                (Mat_<float>(3, 3) << -n.y / len2, n.x / len2, 0, -n.x * n.z
                        / len2, -n.y * n.z / len2, (n.x * n.x + n.y * n.y)
                        / len2, n.x, n.y, n.z);
        Mat newPoint = pointMat * R.t();
        /* {
         float zz = 0;
         for (int k = 0; k < newPoint.rows; k++)
         {
         zz += newPoint.at<float>(k, 2);
         }
         zz /= newPoint.rows;
         printf("");

         }*/
        vector<Point> ptOnPlane(newPoint.rows), hull;
        for (int j = 0; j < newPoint.rows; j++)
        {
            ptOnPlane[j].x = newPoint.at<float>(j, 0);
            ptOnPlane[j].y = newPoint.at<float>(j, 1);
        }
        //{
        //    Mat ptss(ptOnPlane.size(), 2, CV_32S, ptOnPlane.data());
        //    double mm[4];
        //    minMaxIdx(ptss.col(0), mm, mm + 1);
        //    minMaxIdx(ptss.col(1), mm + 2, mm + 3);
        //    Mat show = Mat::zeros(mm[3] - mm[2] + 10, mm[1] - mm[0] + 10,CV_8U);
        //    for (int i = 0; i < newPoint.rows; i++)
        //    {
        //       /* if (i==203)
        //            printf("%d,", i);*/
        //        Point pos = ptOnPlane[i] - Point(mm[0] -5, mm[2] -5);
        //        show.at<uchar>(pos)=255;
        //
        //    }
        //    convexHull(ptOnPlane, hull);
        //    rt = minAreaRect(hull);

        //    printf("");

        //}
        convexHull(ptOnPlane, hull);
        rt = minAreaRect(hull);
        Point2f vertices[4];
        estVertice(vertices, rt.center, rt.angle, 80);
        for (int t = 0; t < 4; t++)
        {
            p.vertex.push_back(vertices[t].x);
            p.vertex.push_back(vertices[t].y);
        }
        Mat verticeMat =
                (Mat_<float>(3, 5) << vertices[0].x, vertices[1].x, vertices[2].x, vertices[3].x, rt.center.x, vertices[0].y, vertices[1].y, vertices[2].y, vertices[3].y, rt.center.y, z0, z0, z0, z0, z0);
        pinv = R.t() * verticeMat;
        {
            /*Mat l0 = (pinv.col(0) - pinv.col(1));
             cout << planeId<<"  "<<l0.t() ;
             printf(";    %f %f %f;\n", p.x, p.y, p.z);*/
        }
        /*for (int i = 0; i < verticeMat.cols; i++)
         {
         pinv(2, i) = (1000 - p.x * pinv(0, i) - p.y * pinv(1, i)) / p.z;
         }*/
        Vec3f delta1(n.x * 200, n.y * 200, n.z * 200);
        Vec3f delta2(n.x * 350, n.y * 350, n.z * 350);
        bool collision = false;

        if(wall.size() == 5)
        {
			for (int i = 0; i < 4; i++)
			{
				Mat topPlane = wall[4];
				Vec3f delta;
				if (i == 0)
					delta = delta2;
				else
					delta = delta1;
				float* w = (float*) wall[i].data;
				for (int j = 0; j < 4; j++) {
					if ((pinv(0, j) * w[0] + pinv(1, j) * w[1]
							+ pinv(2, j) * w[2] - 1000)
							* ((pinv(0, j) + delta[0]) * w[0]
									+ (pinv(1, j) + delta[1]) * w[1]
									+ (pinv(2, j) + delta[2]) * w[2] - 1000)
							< 0)
					{
						float t = (1000 - w[0] * pinv(0, j) - w[1] * pinv(1, j)
								- w[2] * pinv(2, j))
								/ (w[0] * p.x + w[1] * p.y + w[2] * p.z);
						Vec3f cross(pinv(0, j) + p.x * t, pinv(1, j) + p.y * t,
								pinv(2, j) + p.z * t);
						bool onTop = cross[0] * topPlane.at<float>(0)
								+ cross[1] * topPlane.at<float>(1)
								+ cross[2] * topPlane.at<float>(2) - 1000 > 0;
						if (onTop)
							continue;
						//float distance2 =
						//printf("%f %f", pinv(0, j)*w[0] + pinv(1, j)*w[1] + pinv(2, j)*w[2] - 1000, (pinv(0, j) + delta[0])*w[0] + (pinv(1, j) + delta[1])*w[1] + (pinv(2, j) + delta[2])*w[2] - 1000);
						collision = true;
						break;
					}
				}
				if (collision)
					break;
			}
        }

        if (collision)
            continue;
        if (rt.size.width > 90 && rt.size.height > 90)
        {
            found = true;
            objForSort2.push_back(
                    SortPair(planeId,
                            (float) 1. * p.pts.size() / rt.size.area()));
            pinvList[planeId] = pinv.clone();

        }
    }
    Mat show = Mat::zeros(cloud.size(), CV_8U);
        {
            for (int i = 0; i < planes[planeId].pts.size(); i++)
                show.data[planes[planeId].pts[i]] = 255;            
        }
        imshow("result", show);
        waitKey(20);

    string result;
    if (found == false)
        resultStr = "EMPTY";
    else
    {
#ifdef UNIX
        sort(objForSort2.begin(), objForSort2.end(), sort_func_f);
#else
        sort(objForSort2.begin(), objForSort2.end(),
                [](const SortPair& a, const SortPair& b)
                {   return a.value.f > b.value.f;});
#endif
        planeId = objForSort2[0].idx;
        Mat pinv = pinvList[planeId];
        Vec3d pNew, angles;
        Mat l0 = (pinv.col(0) - pinv.col(1));
        Mat l1 = (pinv.col(0) - pinv.col(3));
        Vec3d xy, z;
        if (norm(l0) < norm(l1))
        {
            l0 = l1;
        }
        xy = Vec3d(l0.at<float>(0), l0.at<float>(1), l0.at<float>(2));
        z = Vec3d(planes[planeId].x, planes[planeId].y, planes[planeId].z);
        Mat center = (pinv.col(0) + pinv.col(1) + pinv.col(2) + pinv.col(3))
                / 4;
        Vec3d c(center.at<float>(0), center.at<float>(1), center.at<float>(2));
        Vec4d q = getPose(z, xy);
        pickR = getPoseM(z, xy);
        for (int t = 0; t < 3;t++)
            pickPoint.push_back(c(t));
        for (int t = 0; t < 3; t++)
            pickPoint.push_back(q(t+1));
        pickPoint.push_back(q(0));
        pickVertex = planes[planeId].vertex;
        camera2Arm(z, xy, c, Vec3d(158.68, -414.80, 560.92),
                Vec4d(2.13663E-06, 0.166349, 0.986067, 1.54843E-05), pNew,
                angles);
        Vec3d angleds = angles * 180 / CV_PI;
        char str[200];
#ifdef UNIX
        snprintf(str,200, "%f,%f,%f,%f,%f,%f,0,0,0", pNew[0], pNew[1], pNew[2], angleds[0], angleds[1], angleds[2]);
#else
        sprintf_s(str, 200, "%f,%f,%f,%f,%f,%f,0,0,0", pNew[0], pNew[1],
                pNew[2], angleds[0], angleds[1], angleds[2]);
#endif
        resultStr = str;
    }
}

void
PlaneFit2::mergePlanes()
{
    unsigned int index, rindex, neighborIndex, indexv, indexh;
    for (int j = height - 1; j >= 0; j--)
    {
        rindex = j * width;
        for (int i = width - 1; i >= 0; i--)
        {
            index = i + rindex;
            indexv = index + width;
            indexh = index + 1;
            if (mask[index] == 0)
            {
                continue;
            }
            int root1 = getRegionIndex(index);

            if (mask[indexv] > 0 && j < height - 1)
            {
                int root = getRegionIndex(indexv);
                if (root != root1
                        && abs(
                                planes[chain[root].planeIdx].dis
                                        - planes[chain[root1].planeIdx].dis)
                                < 8)
                {
                    int r = mergeRegions(root, root1);
                    compress(r, index);
                    compress(r, indexv);
                }
            }
            root1 = getRegionIndex(index);
            if (mask[indexh] > 0 && i < width - 1)
            {
                int root = getRegionIndex(indexh);
                if (root != root1
                        && abs(
                                planes[chain[root].planeIdx].dis
                                        - planes[chain[root1].planeIdx].dis)
                                < 8)
                {
                    int r = mergeRegions(root, root1);
                    compress(r, index);
                    compress(r, indexh);
                }
            }

        }
    }
#ifdef UNIX
#else
    struct PlaneDepth
    {
        int idx;
        float depth;
    };
#endif

    std::vector<PlaneDepth> rootList;
    int count = listc[0];
    while (count < size - 1)
    {
        int root = count;
        if (chain[root].area < smallregion * 4)
        {
            count = listc[count];
            continue;
        }
        PlaneDepth pd;
        pd.idx = root;
        pd.depth = pMean[root].z;
        rootList.push_back(pd);
        count = listc[count];
    }
#ifdef UNIX
    sort(rootList.begin(), rootList.end(), sort_func_depth);
#else
    sort(rootList.begin(), rootList.end(),
            [](const PlaneDepth& a, const PlaneDepth& b)
            {   return a.depth < b.depth;});
#endif
    cv::Mat_<cv::Vec3f> normMat(height, width, (cv::Vec3f*) pNormal);
    cv::Mat show = cv::Mat::zeros(height, width, CV_8U);
    for (int i = 0; i < rootList.size(); i++)
    {
        int root = rootList[i].idx;
        while (chain[root].child != root)
        {
            show.data[root] = 255;
            root = chain[root].child;
        }
        if (rootList[i].depth > 1090)
            continue;
    }
}

PlaneFit2::plane
PlaneFit2::calculatePlane(int root)
{
    int count = listc[0];
    plane p;
    p.root = root;
    while (chain[root].child != root)
    {
        p.pts.push_back(root);
        root = chain[root].child;
    }
    cv::Mat mat(p.pts.size(), 3, CV_32F);
    vec3f* pmat = (vec3f*) mat.data;
    for (int i = 0; i < p.pts.size(); i++)
    {
        pmat[i] = pIn[p.pts[i]];
    }
    cv::Mat planeMat = estPlane(mat);
    vec3f* pv = (vec3f*) planeMat.data;
    p.x = pv->x;
    p.y = pv->y;
    p.z = pv->z;
    p.disToOrigin();
    return p;
}

bool
PlaneFit2::planeJudge2(int i1, int i2, int disTor)
{
    plane& curPlane = planes[i1];
    float count = 1;
    vec3f point = pIn[i2];
    float sum = abs(
            curPlane.x * point.x + curPlane.y * point.y + curPlane.z * point.z
                    - 1000) / curPlane.squareroot;
    while (chain[i2].child != i2)
    {
        i2 = chain[i2].child;
        point = pIn[i2];
        sum += abs(
                curPlane.x * point.x + curPlane.y * point.y
                        + curPlane.z * point.z - 1000) / curPlane.squareroot;
        count++;
    }
    return sum / count < disTor;
}

bool
PlaneFit2::planeJudge1(int i1, int i2, int disTor)
{
    bool result = true;
    plane& curPlane = planes[i1];
    vec3f point = pIn[i2];
    float dis = abs(
            curPlane.x * point.x + curPlane.y * point.y + curPlane.z * point.z
                    - 1) / curPlane.squareroot;
    if (dis > disTor)
        return false;
    while (chain[i2].child != i2)
    {
        i2 = chain[i2].child;
        point = pIn[i2];
        dis = abs(
                curPlane.x * point.x + curPlane.y * point.y
                        + curPlane.z * point.z - 1) / curPlane.squareroot;
        if (dis > disTor)
        {
            result = false;
            break;
        }
    }

    return result;
}

void
PlaneFit2::expandPlane(int i1, int i2)
{
    plane& curPlane = planes[i1];
    curPlane.points.push_back(pIn[i2]);
    curPlane.pts.push_back(i2);
    while (chain[i2].child != i2)
    {
        i2 = chain[i2].child;
        curPlane.points.push_back(pIn[i2]);
        curPlane.pts.push_back(i2);
    }
}

void
PlaneFit2::mergeByPlane()
{
    clock_t t0 = clock();
    const int distTor = 10;
    int neighbor[4] =
    { -1, 1, -cols, cols };
    Mat compared = Mat::zeros(cloud.size(), CV_8U);
    for (int n = 0; n < objs.size(); n++)
    {
        int planeIdx = objs[n];
        plane& curPlane = planes[planeIdx];
        int root = curPlane.root;
        if (root != getRegionIndex(root))
            continue;
        int pos = 0;
        int posend = curPlane.points.size();
        vector<int> comparedIdx;
        while (pos != posend)
        {
            int Idx = curPlane.pts[pos];
            for (int k = 0; k < 4; k++)
            {
                int neighborIdx = Idx + neighbor[k];
                int neighborRoot = getRegionIndex(neighborIdx);
                if (pMask[neighborIdx] == 0 || neighborRoot == root
                        || compared.data[neighborRoot] == 1)
                    continue;
                compared.data[neighborRoot] = 1;
                comparedIdx.push_back(neighborRoot);
                showRegion(root, neighborRoot);
                if (chain[neighborRoot].area < smallregion
                        || pNormal[neighborRoot].align(pNormal[root]) < 0.9)
                    continue;

                if (planeJudge2(planeIdx, neighborRoot, 5))
                {
                    orderMerge(root, neighborRoot);
                    expandPlane(planeIdx, neighborRoot);
                    posend = curPlane.points.size();
                }
            }
            pos++;
        }
        for (int i = 0; i < comparedIdx.size(); i++)
            compared.data[comparedIdx[i]] = 0;
    }
    vector<int> oldObjs = objs;
    objs.clear();
    for (int i = 0; i < oldObjs.size(); i++)
    {
        int idx = planes[oldObjs[i]].root;
        if (chain[idx].parent != idx)
            continue;
        objs.push_back(oldObjs[i]);
    }

    Mat show = Mat::zeros(depth.rows, depth.cols, CV_8U);
    for (int i = 0; i < objs.size(); i++)
    {
        int idx = planes[objs[i]].root;
        if (chain[idx].parent != idx)
          //  printf("");
        while (chain[idx].child != idx)
        {
            show.data[idx] = 255;
            idx = chain[idx].child;
        }
    }

}

void
PlaneFit2::project(plane& p)
{
    vec3f n = pNormal[p.root];
    n = n / n.len();
    float len2 = sqrt(n.x * n.x + n.y * n.y);

    Mat_<float> R =
            (Mat_<float>(3, 3) << -n.y / len2, n.x / len2, 0, -n.x * n.z / len2, -n.y
                    * n.z / len2, (n.x * n.x + n.y * n.y) / len2, n.x, n.y, n.z);
    for (int i = 0; i < p.points.size(); i++)
    {

    }

}

void PlaneFit2::run(std::vector<float>& position, cv::Mat_<double>& oritation, std::vector<float>& vertexs)
{
    clock_t t0 = clock();
    //initialite mean mat
    Mat meanMat = cloud.clone();
    pMean = (vec3f*)meanMat.data;
    Mat sumMat = cloud.clone();
    pSum = (vec3f*)sumMat.data;
    //calculate normal
    Mat_<Vec3f> normalMat;
    Mat maskMat;
    estimateNorm(cloud, 3, normalMat, maskMat);
    Mat norminv = -normalMat;
    pNormal = (vec3f*)normalMat.data;
    pMask = maskMat.data;
    //calculate gradient magnitude
    Mat gradient;
    {
        Mat dx, dy, ang;
        cv::Sobel(depth, dx, CV_32F, 1, 0);
        cv::Sobel(depth, dy, CV_32F, 0, 1);
        //Laplacian(depth, ang, CV_32F);
        //gradient = abs(ang);
        cartToPolar(dx, dy, gradient, ang);
        pGrad = (float*)gradient.data;
        for (int i = 0; i < total; i++)
        {
            if (pGrad[i] > gradMax)
                pMask[i] = 0;
        }
    }
    for (int i = 0; i < total; i++)
    {
        if (pMask[i] == 0)
            regionsCount--;
    }

    initSeeds();
    segmentation();
    sortPlanes2();
    mergeByPlane();
    sortPlanes3();
    position.clear();
    vertexs.clear();
    if (resultStr == "Empty")
    {
        return;
    }
    else
    {
        position = pickPoint;
        vertexs = pickVertex;
        oritation = pickR;
    }
   
}


string PlaneFit2::run(cv::Mat& result)
{
    clock_t t0 = clock();
    //initialite mean mat
    Mat meanMat = cloud.clone();
    pMean = (vec3f*) meanMat.data;
    Mat sumMat = cloud.clone();
    pSum = (vec3f*) sumMat.data;
    //calculate normal
    Mat_<Vec3f> normalMat;
    Mat maskMat;
    estimateNorm(cloud, 3, normalMat, maskMat);
    Mat norminv = -normalMat;
    pNormal = (vec3f*) normalMat.data;
    pMask = maskMat.data;
    //calculate gradient magnitude
    Mat gradient;
    {
        Mat dx, dy, ang;
        cv::Sobel(depth, dx, CV_32F, 1, 0);
        cv::Sobel(depth, dy, CV_32F, 0, 1);
        //Laplacian(depth, ang, CV_32F);
        //gradient = abs(ang);
        cartToPolar(dx, dy, gradient, ang);
        pGrad = (float*) gradient.data;
        for (int i = 0; i < total; i++)
        {
            if (pGrad[i] > gradMax)
                pMask[i] = 0;
        }
    }
    for (int i = 0; i < total; i++)
    {
        if (pMask[i] == 0)
            regionsCount--;
    }

    initSeeds();
    segmentation();
    sortPlanes2();
    mergeByPlane();
    sortPlanes3();

    Mat show = Mat::zeros(cloud.size(), CV_8U);
    if (found)
    {
        for (int i = 0; i < planes[planeId].pts.size(); i++)
        {
            int j = planes[planeId].pts[i];
            show.data[j] = 255;
        }
    }

    result = show;

    return resultStr;
}

string
PlaneFit2::run()
{
    clock_t t0 = clock();
    //initialite mean mat
    Mat meanMat = cloud.clone();
    pMean = (vec3f*) meanMat.data;
    Mat sumMat = cloud.clone();
    pSum = (vec3f*) sumMat.data;
    //calculate normal
    Mat_<Vec3f> normalMat;
    Mat maskMat;
    estimateNorm(cloud, 3, normalMat, maskMat);
    Mat norminv = -normalMat;
    pNormal = (vec3f*) normalMat.data;
    pMask = maskMat.data;
    //calculate gradient magnitude
    Mat gradient;
    {
        Mat dx, dy, ang;
        cv::Sobel(depth, dx, CV_32F, 1, 0);
        cv::Sobel(depth, dy, CV_32F, 0, 1);
        //Laplacian(depth, ang, CV_32F);
        //gradient = abs(ang);
        cartToPolar(dx, dy, gradient, ang);
        pGrad = (float*) gradient.data;
        for (int i = 0; i < total; i++)
        {
            if (pGrad[i] > gradMax)
                pMask[i] = 0;
        }
    }
    for (int i = 0; i < total; i++)
    {
        if (pMask[i] == 0)
            regionsCount--;
    }

    initSeeds();
    segmentation();
    sortPlanes2();
    mergeByPlane();
    sortPlanes3();

    Mat show = Mat::zeros(cloud.size(), CV_8U);
    if (found)
    {
        for (int i = 0; i < planes[planeId].pts.size(); i++)
        {
            int j = planes[planeId].pts[i];
            show.data[j] = 255;
        }
    }

    /**
     * commented by laogong
     * here imshow will block GUI
     * reason still not be figired out yet
     */
//    imshow("result", show);
//    imshow("depth", toGray(depth));
    /* ended laogong */
    return resultStr;
}

void
PlaneFit2::aveImage(short *out)
{
    for (int i = 0; i < size; i++)
    {
        out[i] = average[getRegionIndex(i)];
    }
}

void
PlaneFit2::showRegion(int i1, int i2)
{
    Mat show = Mat::zeros(rows, cols, CV_8U);
    while (chain[i1].child != i1)
    {
        show.data[i1] = 180;
        i1 = chain[i1].child;
    }
    while (chain[i2].child != i2)
    {
        show.data[i2] = 255;
        i2 = chain[i2].child;
    }

}

void
PlaneFit2::regions(int* r, float* ave)
{
    for (int i = 0; i < size; i++)
    {
        regionIndex[i] = getRegionIndex(i);
    }
    int* index = new int[regionsCount];
    for (int i = 0; i < regionsCount; i++)
    {
        index[i] = -1;
    }
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < regionsCount; j++)
        {
            if (index[j] == -1)
            {
                index[j] = regionIndex[i];
                ave[j] = average[regionIndex[i]];
                r[i] = j;
                break;
            }
            if (index[j] == regionIndex[i])
            {
                r[i] = j;
                break;
            }
        }
    }
    delete[] index;
}

void
PlaneFit2::addNeighborPair(int neighborIndex, int i1, int i2)
{
    float cosTheta = pNormal[i1].align(pNormal[i2]);
    if (cosTheta > bin[0] || abs(pGrad[i1] - pGrad[i2]) < 1)
    {
        int r1 = getRegionIndex(i1);
        int r2 = getRegionIndex(i2);
        if (r1 != r2)
        {
            int r = mergeRegions(r1, r2);
            compress(r, i2);
            compress(r, i1);
        }
    }
    else
    {
        int diff = 1;
        for (; diff < binCount; diff++)
        {
            if (cosTheta > bin[diff])
            {
                break;
            }
        }
        nextNeighbor[neighborIndex] = neighborBucket[diff - 1];
        neighborBucket[diff - 1] = neighborIndex;
    }
}

void
PlaneFit2::addNeighborPair2(int neighborIndex, int i1, int i2)
{
    if (/*(normal[i1].len() <0.1 && normal[i2].len()<0.1) || */abs(
            pGrad[i1] - pGrad[i2]) < 1)
    {
        int r1 = getRegionIndex(i1);
        int r2 = getRegionIndex(i2);
        if (r1 != r2)
        {
            int r = mergeRegions(r1, r2);
            compress(r, i2);
            compress(r, i1);
        }
    }
    else
    {
        float cosTheta = pNormal[i1].align(pNormal[i2]);
        int diff = 1;
        for (; diff < binCount; diff++)
        {
            if (cosTheta > bin[diff])
            {
                break;
            }
        }
        nextNeighbor[neighborIndex] = neighborBucket[diff - 1];
        neighborBucket[diff - 1] = neighborIndex;
    }
}

int
PlaneFit2::getRegionIndex(int i)
{
    int leaf = i;
    int root = chain[i].parent;
    while (root != leaf)
    {
        leaf = root;
        root = chain[leaf].parent;
    }
    return root;
}

inline void
PlaneFit2::compress(int root, int i)
{
    int leaf = chain[i].parent;
    while (leaf != root)
    {
        chain[i].parent = root;
        i = leaf;
        leaf = chain[i].parent;
    }
}

bool
PlaneFit2::normalign(int i1, int i2)
{
    double length1 = pNormal[i1].len();
    double length2 = pNormal[i2].len();

    if (length1 < 0.5 || length2 < 0.5)
        return false;
    /*  if (pNormal[i1]*pNormal[i2]>length1*length2*0.95)
     printf("%f  %f  %d %d %d %d;  ", pNormal[i1] * pNormal[i2], length1*length2, i1 / width, i1%width, i2 / width, i2%width);*/
    return pNormal[i1] * pNormal[i2] > length1 * length2 * 0.97;
}

bool
PlaneFit2::predicate(int i1, int i2)
{
    vec3f sub(pMean[i1].x - pMean[i2].x, pMean[i1].y - pMean[i2].y,
            pMean[i1].z - pMean[i2].z);
    float dis1 = abs(pNormal[i1] * sub / pNormal[i1].len());
    float dis2 = abs(pNormal[i2] * sub / pNormal[i2].len());
    if (dis1 < 5 && dis2 < 5)
        return true;
    else
        return false;

    /* float d2 = average[i1] - average[i2];
     if (abs(d2) < threshold)
     return true;
     d2 *= d2;
     float log1 = log(1.f + area[i1])
     * (g < area[i1] ? g : area[i1]);
     float log2 = log(1.f + area[i2])
     * (g < area[i2] ? g : area[i2]);*/
    return /*d2 < factor * ((log1 + logdelta) / area[i1]
     + ((log2 + logdelta) / area[i2]))  &&*/normalign(i1, i2);
}

void
PlaneFit2::link(int i1, int i2)
{
    int pa = i1;
    int leaf = chain[pa].child;
    while (leaf != pa)
    {
        pa = chain[leaf].child;
        leaf = chain[pa].child;
    }
    chain[leaf].child = i2;
}

int
PlaneFit2::mergeRegions(int i1, int i2)
{
    regionsCount--;
    if (i1 > i2)
    {
        int temp = i1;
        i1 = i2;
        i2 = temp;
    }
    chain[i1].area += chain[i2].area;
    pMean[i1] = (pMean[i1] * chain[i1].area + pMean[i2] * chain[i2].area)
            / (chain[i1].area + chain[i2].area);
    pSum[i1] += pSum[i2];
    chain[i2].parent = i1;
    pNormal[i1] += pNormal[i2];
    link(i1, i2);
    return i1;

}

void
PlaneFit2::orderMerge(int i1, int i2)
{
    regionsCount--;
    chain[i1].area += chain[i2].area;
    pMean[i1] = (pMean[i1] * chain[i1].area + pMean[i2] * chain[i2].area)
            / (chain[i1].area + chain[i2].area);
    pSum[i1] += pSum[i2];
    chain[i2].parent = i1;
    pNormal[i1] += pNormal[i2];
    link(i1, i2);
    while (i2 != chain[i2].child)
    {
        i2 = chain[i2].child;
    }
    compress(i1, i2);
}
void
PlaneFit2::initSeeds()
{
    // Consider C4-connectivity here
    int index, neighborIndex, indexv, indexh;
    int rindex = (rows - 1) * cols;

    for (int j = rows - 1; j >= 0; j--)
    {
        for (int i = cols - 1; i >= 0; i--)
        {
            index = i + rindex;
            indexv = index + cols;
            indexh = index + 1;
            int neighborIndex = index << 1;
            if (pMask[index] == 0)
            {
                continue;
            }
            // vertical
            if (j < rows - 1 && pMask[indexv])
                addNeighborPair(neighborIndex + 1, index, indexv);
            // horizontal
            if (i < cols - 1 && pMask[indexh])
                addNeighborPair(neighborIndex, index, indexh);
        }
        rindex -= cols;
    }
}
void
PlaneFit2::showAll(int area)
{
    Mat show = Mat::zeros(depth.rows, depth.cols, CV_8U);
    for (int i = 0; i < total; i++)
    {
        if (chain[i].parent != i)
            continue;
        if (chain[i].area > area)
        {
            int node = i;
            while (node != chain[node].child)
            {
                show.data[node] = 255;
                node = chain[node].child;
            }
          //  printf("");
        }
    }
}
void
PlaneFit2::segmentation()
{
    for (int i = 0; i < binCount - 1; i++)
    {
        int neighborIndex = neighborBucket[i];
        while (neighborIndex >= 0)
        {
            int i1 = neighborIndex >> 1;
            int i2 = i1 + (0 == (neighborIndex & 1) ? 1 : cols);

            if(i2 < 0 || i1 < 0) {
                printf("ERROR %d %d width: %d neighborIndex: %d\n", i1, i2, cols, neighborIndex);
            }
            int r1 = getRegionIndex(i1);
            int r2 = getRegionIndex(i2);

            if (r1 != r2 /*&& predicate(r1, r2)*/
                    && pNormal[r1].align(pNormal[r2]) > bin[i + 1])
            {
                int root = mergeRegions(r1, r2);
                compress(root, i1);
                compress(root, i2);
            }
            neighborIndex = nextNeighbor[neighborIndex];
        }
        // showAll(200);
    }
}

void
PlaneFit2::mergeSmallSegions()
{
    unsigned int reg1, reg2;
    unsigned int index, rindex;
    for (int i = 0; i < height; i++)
    {
        rindex = i * width;
        for (int j = 1; j < width; j++)
        {
            index = rindex + j;
            reg1 = getRegionIndex(index);
            reg2 = getRegionIndex(index - 1);
            if (reg1
                    != reg2 /*&& (area[reg1] < smallregion || area[reg2] < smallregion)*/)
            {
                int root = mergeRegions(reg1, reg2);
                compress(root, index);
                compress(root, index - 1);
            }
        }
    }
}

