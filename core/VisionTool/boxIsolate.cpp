#include "boxIsolate.h"
#include "opencv2/opencv.hpp"
#include <stdio.h>
using namespace cv;
using namespace std;

/**
 * Added by laogong, for linux&win porting
 * */
#ifdef UNIX
#define _isnan std::isnan
#endif
/* Ended laogong */

float BoxIsolate::findCenterByProjection(float* val, int n, int length)
{
    int len2 = n - length;
    float* sum = new float[len2];
    memset(sum, 0, sizeof(sum[0])*len2);
    for (int i = 0; i < length; i++)
        sum[0] += val[i];
    int pos = 0;
    for (int i = 1; i < len2; i++)
    {
        sum[i] = sum[i - 1] + (val[i + length - 1] - val[i - 1]);
        if (sum[i] > sum[pos])
        {
            pos = i;
        }
    }
    delete[] sum;
    return pos + length / 2.;
}


BoxIsolate::BoxIsolate(cv::Mat& _depth, int _width, int _height, int _top, int _bottom) :
width(_width), height(_height), top(_top), bottom(_bottom), depth(_depth), pi(3.14159265)
{
}

BoxIsolate::BoxIsolate(cv::Mat& _world) :
world(_world), pi(3.14159265)
{

    hp = new float[world.rows];
    vp = new float[world.cols];
}

void BoxIsolate::init(int width, int height, int _top, int _bottom)
{
    boxWidth = width;
    boxHeight = height;
    top = _top;
    bottom = _bottom;
    memset(hp, 0, sizeof(hp[0])*world.rows);
    memset(vp, 0, sizeof(vp[0])*world.cols);
}
void BoxIsolate::run()
{
}
BoxIsolate::~BoxIsolate()
{
    delete[] hp;
    delete[] vp;
}

void BoxIsolate::pointsInRange(std::vector<cv::Point>& candidates)
{
    float* data = (float*)world.data;
    for (int i = 0; i < world.rows; i += 3)
    {
        int rowIdx = world.cols*i;
        for (int j = 0; j < world.cols; j += 3)
        {
            int idx = rowIdx + j;
            if (data[idx] > bottom && data[idx] < top)
            {
                candidates.push_back(Point(j, i));
                hp[i]++;
                vp[j]++;
            }
        }
    }
}


void BoxIsolate::pointsInRange(float* hp, float*vp, std::vector<cv::Point>& candidates)
{
    memset(hp, 0, sizeof(hp[0])*depth.rows);
    memset(vp, 0, sizeof(hp[0])*depth.cols);
    short* data = (short*)depth.data;
    Mat show = Mat::zeros(depth.size(), CV_8U);
    for (int i = 0; i < depth.rows; i += 3)
    {
        int rowIdx = depth.cols*i;
        for (int j = 0; j < depth.cols; j += 3)
        {
            int idx = rowIdx + j;
            if (data[idx] > bottom && data[idx] < top)
            {
                candidates.push_back(Point(j, i));
                show.at<uchar>(idx) = 255;
                hp[i]++;
                vp[j]++;
            }
        }
    }
}
float BoxIsolate::rotationTest(std::vector<cv::Point>& box, int cx, int cy)
{
    Mat boxMat(box.size(), 2, CV_32F);
    float* data = (float*)boxMat.data;
    int j = 0;
    const float w2 = width / 2.;
    const float h2 = height / 2.;
   /* float lx = cx - w2;
    float rx = cx + w2;
    float ty = cy - h2;
    float by = cy + h2;*/
    for (int i = 0; i < box.size(); i++)
    {
        data[j++] = box[i].x - cx;
        data[j++] = box[i].y - cy;

    }
    Mat rotated;
    int l = 20;
    int* inBox = new int[1+l*2];
    int pos = 0;
    int maxInBox = 0;
    for (int i = -l; i <= l; i++)
    {
        inBox[i+l] = 0;
        float theta = 0.5*i*pi / 180;
        if (i == 0)
            rotated = boxMat;
        else
        {
            Mat rMat = (Mat_<float>(2, 2) << cos(theta), sin(theta), -sin(theta), cos(theta));
            rotated = boxMat*rMat;
        }
        float* rdata = (float*)rotated.data;
        for (int j = 0; j < boxMat.rows; j++)
        {
            if (rdata[j] > -w2 && rdata[j]<w2 && rdata[j + 1]>-h2 && rdata[j + 1] < h2)
                inBox[i+l]++;
        }
        if (inBox[i+l] > maxInBox)
        {
            maxInBox = inBox[i+l];
            pos = i;
        }
    }
    delete[] inBox;
    return pos*0.5;
}
void BoxIsolate::filterByRange(float* cx, float* cy, float* theta)
{
    hp = new float[depth.rows];
    vp = new float[depth.cols];
    vector<cv::Point> candidates;
    pointsInRange(hp, vp, candidates);

    *cy = findCenterByProjection(hp, depth.rows, height);
    *cx = findCenterByProjection(vp, depth.cols, width);

    *theta = rotationTest(candidates, *cx, *cy);
    //printf("%f %f \n", cx, cy);

    delete[] hp;
    delete[] vp;
    hp = vp = NULL;
}
namespace vt
{
    float findCenter(float* val, int n, int length)
    {
        int len2 = n - length;
        float* sum = new float[len2];
        memset(sum, 0, sizeof(sum[0])*len2);
        for (int i = 0; i < length; i++)
            sum[0] += val[i];
        int pos = 0;
        for (int i = 1; i < len2; i++)
        {
            sum[i] = sum[i - 1] + (val[i + length - 1] - val[i - 1]);
            if (sum[i] > sum[pos])
            {
                pos = i;
            }
        }
        delete[] sum;
        return pos + length / 2.;
    }

    float findCenterByProjection(float* val, int n, int length)
    {
        int len2 = n - length;
        float* sum = new float[len2];
        memset(sum, 0, sizeof(sum[0])*len2);
        for (int i = 0; i < length; i++)
            sum[0] += val[i];
        int pos = 0;
        for (int i = 1; i < len2; i++)
        {
            sum[i] = sum[i - 1] + (val[i + length - 1] - val[i - 1]);
            if (sum[i] > sum[pos])
            {
                pos = i;
            }
        }
        delete[] sum;
        return pos + length / 2.;
    }

    float rotationEst(float* pts, int length, int cx, int cy, int boxWidth, int boxHeight)
    {
        const float pi = 3.141592654;
        const float w2 = boxWidth / 2.;
        const float h2 = boxHeight / 2.;
        const int total = length * 2;
        float R[4];
        for (int j = 0; j < total;)
        {
            pts[j] -= cx;
            j++;
            pts[j] -= cy;
            j++;
        }
        float* buffer = new float[total];
        float* rotated;
        int l = 20;
        int* inBox = new int[1 + l * 2];
        int pos = 0;
        int maxInBox = 0;
        for (int i = -l; i <= l; i++)
        {
            inBox[i + l] = 0;
            float theta = 0.5*i*pi / 180;
            if (i == 0)
                rotated = pts;
            else
            {
                rotated = buffer;
                R[0] = cos(theta); R[1] = -sin(theta); R[2] = sin(theta); R[3] = cos(theta);
                for (int j = 0; j < total;)
                {
                    rotated[j] = pts[j] * R[0] + pts[j + 1] * R[1];
                    rotated[j + 1] = pts[j] * R[2] + pts[j + 1] * R[3];
                    j += 2;
                }
            }
            for (int j = 0; j < total;)
            {
                if (rotated[j] > -w2 && rotated[j]<w2 && rotated[j + 1]>-h2 && rotated[j + 1] < h2)
                    inBox[i + l]++;
                j += 2;
            }
            if (inBox[i + l] > maxInBox)
            {
                maxInBox = inBox[i + l];
                pos = i;
            }
        }
        delete[] inBox;
        delete[] buffer;
        return pos*0.5*pi / 180;
    }

    Rect boxEst(float* pts, float* plane, int imgWidth, int imgHeight, int height, int error,
        int boxWidth, int boxHeight, vector<Mat>& wall, int borderX, int borderY)
    {
        float p[3], *dis, R[9], *ptsNew, R2[9], cx, cy, angle, len;
        int total, minv, maxv, border[4];
        vector<int> candidates;
        //initiate
        {
            total = imgWidth*imgHeight;
            len = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
            if (plane[2] < 0)
            {
                for (int i = 0; i < 3; i++)
                    p[i] = -plane[i] / len;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                    p[i] = plane[i] / len;
            }
            //len = 1000 / len;
            minv = 1000 - (height + error)*len;
            maxv = 1000 - (height - error)*len ;
            R[6] = p[0]; R[7] = p[1]; R[8] = p[2];
            float xz = -p[0] / p[2];
            R[0] = 1 / sqrt(xz*xz + 1); R[1] = 0; R[2] = xz;
            R[3] = R[2] * R[7]; R[4] = R[8] + R[6] * R[6] / R[8]; R[5] = -R[7];
        }
        //filter the points by height
        {
            Mat show = Mat::zeros(imgHeight, imgWidth, CV_8U);
            int pos = 0;
            float* pt = pts;
            while (pos < total)
            {
                if (!_isnan(pt[0]))
                {
                    float e = pt[0] * plane[0] + pt[1] * plane[1] + pt[2] * plane[2];
                    if (e > minv && e < maxv)
                    {
                        candidates.push_back(pos);
                        show.data[pos] = 255;
                    }
                }
                pt += 3;
                pos++;
            }
            //imshow("dbg", show);
            //waitKey(0);
        }
        //random shuffle
        random_shuffle(candidates.begin(), candidates.end());
        //project
        {
            //Mat show = Mat::zeros(imgHeight, imgWidth, CV_8U);
            int n = candidates.size() / 10;
            if(n < 100)
            {
                printf("Error in n: %d %d\n", n, candidates.size());
                return Rect(0, 0, 0, 0);
            }
            ptsNew = new float[n * 2];
            float* pos = ptsNew;
            int minx, maxx, miny, maxy;
            minx = miny = 10000;
            maxx = maxy = -10000;
            //z0 = 0;
            for (int i = 0; i < n; i++, pos += 2)
            {
                float* old = pts + candidates[i] * 3;
                pos[0] = old[0] * R[0] + old[1] * R[1] + old[2] * R[2];
                pos[1] = old[0] * R[3] + old[1] * R[4] + old[2] * R[5];
                //z0 += old[0] * R[6] + old[1] * R[7] + old[2] * R[8];
                assert(!_isnan(pos[0]));
                if (pos[0] > maxx)
                    maxx = pos[0];
                if (pos[0] < minx)
                    minx = pos[0];
                if (pos[1] > maxy)
                    maxy = pos[1];
                if (pos[1] < miny)
                    miny = pos[1];
                //show.data[candidates[i]] = 255;
            }
            //z0 /= n;
            int rangex = maxx - minx + 3;
            int rangey = maxy - miny + 3;

            float* hp = new float[rangey];
            float* vp = new float[rangex];
            memset(hp, 0, rangey*sizeof(float));
            memset(vp, 0, rangex*sizeof(float));
            for (int i = 0; i < n; i++)
            {
                float* pt = ptsNew + i * 2;
                int x = pt[0] - minx + 1;
                int y = pt[1] - miny + 1;
                hp[y]++;
                vp[x]++;
            }
            if (rangey < boxHeight || rangex < boxWidth)
                return Rect(0, 0, 0, 0);
            cy = findCenterByProjection(hp, rangey, boxHeight) + miny - 1;
            cx = findCenterByProjection(vp, rangex, boxWidth) + minx - 1;
            angle = rotationEst(ptsNew, n, cx, cy, boxWidth, boxHeight);
            delete[] hp;
            delete[] vp;
            delete[] ptsNew;
            float a = cos(angle);
            float b = sin(angle);
            {
                float cx2 = a*cx - b*cy;
                float cy2 = b*cx + a*cy;
                border[0] = cx2 - (boxWidth / 2 - borderX);
                border[1] = cx2 + (boxWidth / 2 - borderX);
                border[2] = cy2 - (boxHeight / 2 - borderY);
                border[3] = cy2 + (boxHeight / 2 - borderY);
            }
            R2[0] = a*R[0] - b*R[3]; R2[1] = a*R[1] - b*R[4]; R2[2] = a*R[2] - b*R[5];
            R2[3] = b*R[0] + a*R[3]; R2[4] = b*R[1] + a*R[4]; R2[5] = b*R[2] + a*R[5];
            //printf("%d", clock() - t0);
        }
        {
            float x1 =  - (boxWidth / 2 - borderX);
            float x2 =  (boxWidth / 2 - borderX);
            float y1 = - (boxHeight / 2 - borderY);
            float y2 = (boxHeight / 2 - borderY);
            float a = cos(angle);
            float b = sin(angle);
            Mat_<float> corners = (Mat_<float>(3, 4) << x1, x2, x2, x1, y1, y1, y2, y2, 0, 0, 0, 0);
            Mat_<float> Rinv = (Mat_<float>(3, 3) <<a,b,0,-b,a,0, 0, 0, 1);
            corners = Rinv*corners;
            for (int i = 0; i < corners.cols; i++)
            {
                corners(0, i) += cx;
                corners(1, i) += cy;
            }
            Rinv = Mat(3, 3, CV_32F, R).inv();
            corners = Rinv*corners;
           /* for (int i = 0; i < corners.cols; i++)
            {
                corners(2, i) = (1000 - plane[0] * corners(0, i) - plane[1] * corners(1, i)) / plane[2];
            }    */
            for (int i = 0; i < corners.cols; i++)
            {
                int m = (i + 1) % corners.cols;
                Mat X = (Mat_<float>(3, 3) << plane[0], plane[1], plane[2],
                    corners(0, i), corners(1, i), corners(2, i),
                    corners(0, m), corners(1, m), corners(2, m));
                Mat one = (Mat_<float>(3, 1) << 0, 1000, 1000);
                Mat planex = X.inv()*one;
                wall.push_back(planex);
            }
            {
                float dis;
                if (plane[2] < 0)
                    dis = -height;
                else
                    dis = height;
                len = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
                float k = 1000 / (dis + 1000 / len);
                Mat botPlane = (Mat_<float> (3,1) << k*plane[0], k*plane[1], k*plane[2]);
                wall.push_back(botPlane);
            }
        }
        {
            //Mat_<Vec3f> world(imgHeight, imgWidth, (Vec3f*)pts);
            int idx = 0;
            int minx, miny, maxx, maxy;
            minx = imgWidth;
            miny = imgHeight;
            maxx = maxy = 0;
            //Mat show = Mat::zeros(imgHeight, imgWidth, CV_8U);
            for (int i = 0; i < imgHeight; i++)
            {
                for (int j = 0; j < imgWidth; j++, idx += 3)
                {
                    if (_isnan(pts[idx]))
                        continue;
                    float x = pts[idx] * R2[0] + pts[idx + 1] * R2[1] + pts[idx + 2] * R2[2];
                    float y = pts[idx] * R2[3] + pts[idx + 1] * R2[4] + pts[idx + 2] * R2[5];
                    if (x > border[0] && x<border[1] && y>border[2] && y < border[3])
                    {
                        if (i > maxy)
                            maxy = i;
                        if (i < miny)
                            miny = i;
                        if (j > maxx)
                            maxx = j;
                        if (j < minx)
                            minx = j;
                    }
                    else
                    {
                        memset(pts + idx, 255, 12);
                    }
                }
            }
            return Rect(minx, miny, maxx - minx, maxy - miny);
        }
    }
}
