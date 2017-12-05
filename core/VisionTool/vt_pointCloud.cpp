#include "vt_pointCloud.h"

using namespace cv;
using namespace std;

#ifdef UNIX
#include <stdio.h>

#define _isnan std::isnan
#endif

namespace vt
{
    cv::Mat estPlane(cv::Mat& pts)
    {
        if (pts.rows < 2)
            return Mat();
        Mat one(pts.rows, 1, CV_32F);
        float* p = (float*)one.data;
        for (int i = 0; i < pts.rows; i++)
            p[i] = 1000;
        return pts.inv(DECOMP_SVD)*one;
    }
    cv::Mat estPlane(vector<cv::Vec3f>& pts)
    {
        int len = pts.size();
        if (len < 2)
            return Mat();
        Mat pointMat(len, 3, CV_32F, pts.data());
        Mat one(len, 1, CV_32F);
        float* p = (float*)one.data;
        for (int i = 0; i < len; i++)
            p[i] = 1000;
        return pointMat.inv(DECOMP_SVD)*one;
    }
    void estimateNorm(cv::Mat_<cv::Vec3f>& mat, int wz, cv::Mat_<cv::Vec3f>& norm, cv::Mat& mask)
    {
        norm = Mat_<Vec3f>::zeros(mat.size());
        Mat marker = Mat::ones(mat.size(), CV_8U);
        mask = Mat::zeros(mat.size(), CV_8U);
        Mat_<float> diff[6];
        float* pdiff[6];
        for (int i = 0; i < 6; i++)
        {
            diff[i] = Mat::zeros(mat.size(), CV_32F);
            pdiff[i] = (float*)diff[i].data;
        }
        int ridx, idx;
        Vec3f* pdata = (Vec3f*)mat.data;
        vector<int> nanList;
        for (int i = 1; i < mat.rows - 1; i++)
        {
            ridx = i*mat.cols;
            for (int j = 1; j < mat.cols - 1; j++)
            {
                idx = ridx + j;
                if (_isnan(pdata[idx][2]))
                {
                    marker.data[idx + 1] = 0;
                    marker.data[idx - 1] = 0;
                    marker.data[idx + mat.cols] = 0;
                    marker.data[idx - mat.cols] = 0;
                    nanList.push_back(idx);
                }
            }
        }
        Mat inMarker, inDiff[6];
        integral(marker, inMarker);
        for (int i = 1; i < mat.rows - 1; i++)
        {
            ridx = i*mat.cols;
            for (int j = 1; j < mat.cols - 1; j++)
            {
                idx = ridx + j;
                if (marker.data[idx] == 1)
                {
                    pdiff[0][idx] = pdata[idx + 1][0] - pdata[idx - 1][0];
                    pdiff[1][idx] = pdata[idx + 1][1] - pdata[idx - 1][1];
                    pdiff[2][idx] = pdata[idx + 1][2] - pdata[idx - 1][2];
                    pdiff[3][idx] = pdata[idx + mat.cols][0] - pdata[idx - mat.cols][0];
                    pdiff[4][idx] = pdata[idx + mat.cols][1] - pdata[idx - mat.cols][1];
                    pdiff[5][idx] = pdata[idx + mat.cols][2] - pdata[idx - mat.cols][2];
                }
                else
                {
                    for (int k = 0; k < 6; k++)
                    {
                        pdiff[k][idx] = 0;
                    }
                }
            }
        }

        for (int k = 0; k < 6; k++)
        {
            integral(diff[k], inDiff[k]);
        }
        float v[6];
        int validThre = (wz + 1)*(wz + 1);
        for (int i = 1 + wz; i < mat.rows - wz - 1; i++)
        {
            for (int j = wz + 1; j < mat.cols - wz - 1; j++)
            {

                int valid = inMarker.at<int>(i - wz, j - wz) + inMarker.at<int>(i + 1 + wz, j + 1 + wz) -
                    inMarker.at<int>(i + 1 + wz, j - wz) - inMarker.at<int>(i - wz, j + 1 + wz);
                if (valid <= validThre)
                {
                    norm.at<Vec3f>(i, j) = Vec3f(0, 0, 0);
                    continue;
                }
                for (int k = 0; k < 6; k++)
                {
                    v[k] = inDiff[k].at<double>(i - wz, j - wz) + inDiff[k].at<double>(i + 1 + wz, j + 1 + wz) -
                        inDiff[k].at<double>(i + 1 + wz, j - wz) - inDiff[k].at<double>(i - wz, j + 1 + wz);
                }
                Vec3f vec(v[1] * v[5] - v[2] * v[4], v[2] * v[3] - v[0] * v[5], v[0] * v[4] - v[1] * v[3]);
                vec = vec / cv::norm(vec);
                assert(!_isnan(vec[0]));
                norm.at<Vec3f>(i, j) = vec;
                mask.at<uchar>(i, j) = 1;
            }
        }
        for (int i = 0; i < nanList.size(); i++)
            mask.data[nanList[i]] = 0;
    }
    bool estimateNorm(cv::Mat_<cv::Vec3f>& mat, cv::Rect rect, int wz, cv::Mat_<cv::Vec3f>& norm)
    {
        int xs, ys, xe, ye;
        xs = rect.x - wz-1;
        ys = rect.y - wz-1;
        xe = rect.br().x + wz + 2;
        ye = rect.br().y + wz + 2;
        if (xs<0|| ys<0|| (mat.cols - xe) <0|| (mat.rows - ye)<0)
            return false;
        norm.create(rect.size());
        Size sz(rect.width + wz * 2, rect.height + wz * 2);
        Mat_<uchar> marker = Mat::ones(sz.height+4, sz.width+4, CV_8U);
        int ridx, idx;
        Vec3f* pdata = (Vec3f*)mat.data;

        for (int i = 1; i < marker.rows-1; i++)
        {
            ridx = (ys-1+i)*mat.cols;
            for (int j = 1; j < marker.cols-1; j++)
            {
                idx = ridx + j+xs-1;
                if (pdata[idx][2] < 100)
                {
                    marker(i,j+1) = 0;
                    marker(i,j-1) = 0;
                    marker(i+1,j) = 0;
                    marker(i-1,j) = 0;
                }
            }
        }
        Mat_<float> diff[6];
        float* pdiff[6];
        for (int i = 0; i < 6; i++)
        {
            diff[i] = Mat::zeros(sz, CV_32F);
            pdiff[i] = (float*)diff[i].data;
        }
        Mat inMarker, inDiff[6];
        integral(marker, inMarker);
        for (int i = 0; i < sz.height; i++)
        {
            ridx = (i+ys+1)*mat.cols;
            for (int j = 0; j < sz.width; j++)
            {
                idx = ridx + j+xs+1;
                if (marker(i+2,j+2) == 1)
                {
                    diff[0](i,j) = pdata[idx + 1][0] - pdata[idx - 1][0];
                    diff[1](i, j) = pdata[idx + 1][1] - pdata[idx - 1][1];
                    diff[2](i, j) = pdata[idx + 1][2] - pdata[idx - 1][2];
                    diff[3](i, j) = pdata[idx + mat.cols][0] - pdata[idx - mat.cols][0];
                    diff[4](i, j) = pdata[idx + mat.cols][1] - pdata[idx - mat.cols][1];
                    diff[5](i, j) = pdata[idx + mat.cols][2] - pdata[idx - mat.cols][2];
                }
                else
                {
                    for (int k = 0; k < 6; k++)
                    {
                        diff[k](i, j) = 0;
                    }
                }
            }
        }
        for (int k = 0; k < 6; k++)
        {
            integral(diff[k], inDiff[k]);
        }
        float v[6];
        int validThre = (wz + 1)*(wz + 1);
        for (int i = 0; i < rect.height; i++)
        {
            for (int j = 0; j < rect.width; j++)
            {

                int valid = inMarker.at<int>(i+ 2, j +2) + inMarker.at<int>(i + 3 + wz*2, j + 3 + wz*2) -
                    inMarker.at<int>(i + 3 + wz*2, j +2) - inMarker.at<int>(i +2, j + 3 + wz*2);
                if (valid <= validThre)
                {
                    norm.at<Vec3f>(i, j) = Vec3f(0, 0, 0);
                    continue;
                }
                for (int k = 0; k < 6; k++)
                {
                    v[k] = inDiff[k].at<double>(i, j) + inDiff[k].at<double>(i + 1 + wz*2, j + 1 + wz*2) -
                        inDiff[k].at<double>(i + 1 + wz*2, j) - inDiff[k].at<double>(i, j + 1 + wz*2);
                }
                Vec3f vec(v[1] * v[5] - v[2] * v[4], v[2] * v[3] - v[0] * v[5], v[0] * v[4] - v[1] * v[3]);
                vec = vec / cv::norm(vec);
                norm.at<Vec3f>(i, j) = vec;
            }
        }
    }
    cv::Mat approxPlane(const cv::Mat& points)
    {
        Mat one = Mat::ones(points.rows, 1, CV_32F);
        return points.inv(DECOMP_SVD)*one;
    }
    inline cv::Mat approxPlane(Vec3f* p, std::vector<int>& index)
    {
        vector<Vec3f> planePts;
        for (int i = 0; i < index.size(); i++)
        {
            planePts.push_back(p[index[i]]);
        }
        return approxPlane(Mat(planePts.size(), 3, CV_32F, (char*)planePts.data()));
    }
    std::vector<int> planeVerify(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat, std::vector<int>& ptOnPlane, float th)
    {
        Vec3f pl;
        for (int i = 0; i < 3; i++)
            pl[i] = plane.at<float>(i);
        pl = pl / cv::norm(pl);
        std::vector<int> list;
        vector<Vec3f> planePts;
        for (int i = 0; i < ptOnPlane.size(); i++)
        {
            Vec3f norm = normMat(ptOnPlane[i]);
           // normMat(ptOnPlane[i]) = Vec3f(255, 255, 255);
            float sum = pl[0] * norm[0] + pl[1] * norm[1] + pl[2] * norm[2];
            if (abs(sum) > th)
            {
                list.push_back(ptOnPlane[i]);
            }
        }
        return list;
    }

    void regular(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat, std::vector<int>& pt)
    {
        Vec3f pl;
        for (int i = 0; i < 3; i++)
            pl[i] = plane.at<float>(i);
        pl = pl / cv::norm(pl);
    }

    inline float distance(float* point, float* plane, float len)
    {
        float sum = -1;
        for (int i = 0; i < 3; i++)
        {
            sum += point[i] * plane[i];
        }
        return sum / len;
    }

    bool Cube::init(cv::Mat* _plane, cv::Mat_<cv::Vec3f>* _world, cv::Mat_<cv::Vec3f>* _normal, std::vector<int>* _index, int _num, float _threshold, bool _fix)
    {
        // need check
        plane = _plane;
        world = _world;
        index = *_index;
        num = _num;
        threshold = _threshold;
        fix = _fix;
        normal = _normal;
        return true;
    }
    void Cube::boards()
    {
        vector<int> remain;
        Vec3f* p = (Vec3f*)world->data;
        vector<Vec3f> planePts, cur;
        for (int k = 0; k < 3; k++)
        {
            for (int j = 0; j < 2; j++)
            {
                planeIdx[k].clear();
                remain.clear();
                cur.clear();
                planePts.clear();
                float len = sqrt((Mat_<float>(plane[k].t()*plane[k]))(0, 0));
                for (int i = 0; i < index.size(); i++)
                {
                    float* curPoint = (float*)(&p[index[i]]);
                    float dis = distance(curPoint, (float*)plane[k].data, len);
                    if (threshold > abs(dis))
                    {
                        planeIdx[k].push_back(index[i]);
                        planePts.push_back(p[index[i]]);
                    }
                    else if (dis < 0)
                    {
                        remain.push_back(index[i]);
                        cur.push_back(p[index[i]]);
                    }
                }
                if (planeIdx[k].size() < num)
                {
                    printf("Can not find board.");
                    return;
                }
                if (fix)
                    break;
                plane[k] = approxPlane(Mat(planePts.size(), 3, CV_32F, (char*)planePts.data()));
            }
            if (!fix)
            {
                std::vector<int> newIdx = planeVerify(plane[k], *normal, planeIdx[k]);
                plane[k] = approxPlane((Vec3f*)world->data, newIdx);
                planeIdx[k] = newIdx;
            }
            index = remain;
        }
        Mat mask = Mat::zeros(world->size(), CV_8U);
        for (int i = 0; i < index.size(); i++)
        {
            mask.at<uchar>(index[i]) = 1;
        }
        Mat inMask;
        remain.clear();
        integral(mask, inMask);
        for (int k = 0; k < index.size(); k++)
        {
            remain.push_back(index[k]);
            int i = index[k] / mask.cols;
            int j = index[k] % mask.cols;
            if (i<10 || j<10 || i>(mask.rows - 11) || j>(mask.cols - 11))
                continue;
            int sum = inMask.at<int>(i - 10, j - 10) + inMask.at<int>(i + 11, j + 11)
                - inMask.at<int>(i - 10, j + 11) - inMask.at<int>(i + 11, j - 10);
            if (sum < 4)
            {
                remain.pop_back();
                mask.data[index[k]] = 0;
            }

        }
        objIdx = remain;
    }
    bool comp(pair<int, float> p0, pair<int, float> p1)
    {
        return p0.second>p1.second;
    }
    vector< pair<int, float> > calmaxdis(Mat plane, Mat& mat, vector<int> points)
    {
        float distance = 0;
        float *planeof = plane.ptr<float>(0);
        Vec3f* pt = (Vec3f*)mat.data;
        float norm = sqrt(planeof[0] * planeof[0] + planeof[1] * planeof[1] + planeof[2] * planeof[2]);
        vector< pair<int, float> >distances;
        for (int i = 0; i < points.size(); i++)
        {
            if (pt[points[i]](2) < 10)
                continue;
            float dis = abs(planeof[0] * pt[points[i]](0) + planeof[1] * pt[points[i]](1) + planeof[2] * pt[points[i]](2) - 1) / norm;
            distances.push_back(pair<int, float>(points[i], dis));
        }
        sort(distances.begin(), distances.end(), comp);
        return distances;
    }
    float maxdis(Mat plane, Mat& mat, vector<int>& points, int k=1)
    {
        float distance = 0;
        float *planeof = plane.ptr<float>(0);
        Vec3f* pt = (Vec3f*)mat.data;
        float norm = sqrt(planeof[0] * planeof[0] + planeof[1] * planeof[1] + planeof[2] * planeof[2]);
        float len = 0;
        vector<float> lens;
        for (int i = 0; i < points.size(); i++)
        {
            if (pt[points[i]](2) < 10)
                continue;
            float dis = abs(planeof[0] * pt[points[i]](0) + planeof[1] * pt[points[i]](1) + planeof[2] * pt[points[i]](2) - 1) / norm;
            lens.push_back(dis);
        }
        sort(lens.begin(), lens.end());
        for (int i = 0; i < k; i++)
            len += lens[lens.size()-i-1];
        return len/k;
    }
    std::vector<int> disVerify(cv::Mat& plane, cv::Mat_<cv::Vec3f>& world, std::vector<int>& points, float th)
    {
        vector<int> list;
        float *planeof = plane.ptr<float>(0);
        Vec3f* pt = (Vec3f*)world.data;
        float norm = sqrt(planeof[0] * planeof[0] + planeof[1] * planeof[1] + planeof[2] * planeof[2]);
        for (int i = 0; i < points.size(); i++)
        {
            float dis = abs(planeof[0] * pt[points[i]](0) + planeof[1] * pt[points[i]](1) + planeof[2] * pt[points[i]](2) - 1) / norm;
            if (dis > th)
                list.push_back(points[i]);
        }
        return list;
    }
    bool compVec(vector<int> p0, vector<int> p1)
    {
        return p0.size()>p1.size();
    }
    float avedis(Mat plane, Mat& mat, vector<int> points, float* maxdis)
    {
        float distance = 0;
        float *planeof = plane.ptr<float>(0);
        Vec3f* pt = mat.ptr<Vec3f>();
        float norm = sqrt(planeof[0] * planeof[0] + planeof[1] * planeof[1] + planeof[2] * planeof[2]);
        *maxdis = 0;
        float len;
        for (int i = 0; i < points.size(); i++)
        {
            len = abs(planeof[0] * pt[points[i]](0) + planeof[1] * pt[points[i]](1) + planeof[2] * pt[points[i]](2) - 1) / norm;
          /* if (len<20)
            mat.at<Vec3f>(points[i]) = Vec3f(255, -1, -1);
            else if (len<25)
                mat.at<Vec3f>(points[i]) = Vec3f(-1, 255, -1);
            else if (len<30)
                mat.at<Vec3f>(points[i]) = Vec3f(-1, -1, 255);*/
             /*else if (len<98)
                mat.at<Vec3f>(points[i]) = Vec3f(-1, 255, 255);
            else if (len<99)
                mat.at<Vec3f>(points[i]) = Vec3f(255, 255, -1);
            else if (len<100)
                mat.at<Vec3f>(points[i]) = Vec3f(255, 255, 255);*/
            distance += len;
            if (len > *maxdis)
                *maxdis = len;
        }
        return distance / points.size();
    }
    std::vector<int> normalCluster(cv::Mat_<cv::Vec3f>& world, cv::Mat_<cv::Vec3f>& normal, std::vector<int> obj, cv::Mat plane, float cos, float dis)
    {
        vector<int> label;
        int labelId = -1;
        Mat mask = Mat::ones(world.size(), CV_32S)*-1;
        int idx, l;
        int* region = new int[8];
        region[0] = -1;
        region[1] = 1;
        region[2] = -mask.cols;
        region[3] = mask.cols;
        region[4] = -mask.cols-1;
        region[5] = -mask.cols+1;
        region[6] = mask.cols-1;
        region[7] = mask.cols+1;
        vector< vector<int> > cluster;
        vector<int> sortlabel;
        for (int i = 0; i < obj.size(); i++)
        {
            idx = obj[i];
            int m = idx / world.cols;
            int n = idx % world.cols;
            if (m<1 || n<1 || m>world.rows - 2 || n>world.cols - 1)
                continue;
            l = -1;
            Vec3f& v0 = normal(idx);
            for (int j = 0; j < 8; j++)
            {
                int& rl = mask.at<int>(idx + region[j]);
                Vec3f& v1 = normal(idx + region[j]);
                if (rl>-1 && l==-1 && v0.dot(v1)>cos)
                {
                    l = rl;
                }
                else if (rl > l && v0.dot(v1)>cos)
                {
                    label[rl] = l;
                }
                else if (rl>-1 && rl < l && v0.dot(v1)>cos)
                {
                    label[l] = rl;
                    l = rl;
                }
            }
            if (l > -1)
                mask.at<int>(idx) = l;
            else
            {
                mask.at<int>(idx) = ++labelId;
                label.push_back(labelId);
            }
        }
        for (int i = 0; i < label.size(); i++)
        {
            int k = label[i];
            while (label[k] != k)
            {
                k = label[k];
            }
            label[i] = k;
            std::vector<int>::iterator it = find(sortlabel.begin(), sortlabel.end(), k);
            if (it == sortlabel.end())
            {
                sortlabel.push_back(k);
            }
        }
        cluster.resize(sortlabel.size());
        for (int i = 0; i < obj.size(); i++)
        {
            idx = obj[i];
            if (mask.at<int>(idx) == -1)
                continue;
            //printf("\n%d", mask.at<int>(idx));
            l = label[mask.at<int>(idx)];
            std::vector<int>::iterator it = find(sortlabel.begin(), sortlabel.end(), l);
            cluster[it - sortlabel.begin()].push_back(idx);
        }
        sort(cluster.begin(), cluster.end(), compVec);
        float maxdis;
        float len = avedis(plane, world, cluster[0], &maxdis);

        for (int i = 1; i < cluster.size(); i++)
        {
            float maxdis2;
            if (abs(avedis(plane, world, cluster[i], &maxdis2) - len) < dis)
                cluster[0].insert(cluster[0].end(), cluster[i].begin(), cluster[i].end());
            printf("%f ", maxdis2);
            if (maxdis2 > maxdis)
                maxdis = maxdis2;
        }
        delete[] region;
        if (maxdis - len < 9)
            return cluster[0];

    }
    void Cube::regularVerify(float* len)
    {

        Mat showVerify2 = Mat::zeros(world->rows, world->cols, CV_8U);
        for (int j = 0; j < objIdx.size(); j++)
        {
            showVerify2.at<uchar>(objIdx[j]) = 90;
        }
        for (int i = 0; i < 3; i++)
        {
            regularIdx[i] = planeVerify(plane[i], *normal, objIdx,0.9);
            len[i] = maxdis(plane[i], *world, objIdx,30<objIdx.size()?30:objIdx.size());
            float len2 = maxdis(plane[i], *world, objIdx, 1);
            for (int j = 0; j <regularIdx[i].size(); j++)
            {
                showVerify2.at<uchar>(regularIdx[i][j]) = 125;
            }
            if (regularIdx[i].size() < 80)
            {
                continue;
            }
            vector<int>list =  normalCluster(*world, *normal, regularIdx[i], plane[i]);
            float maxdis;
            float avelen = avedis(plane[i], *world, list, &maxdis);
            if (len[i] - avelen < 9)
                len[i] = avelen;
            for (int j = 0; j <list.size(); j++)
            {
                showVerify2.at<uchar>(list[j]) = 188;
            }
            printf("");
        }
    }

    void multiFrames(std::vector< cv::Mat_<Vec3f> >& frames, cv::Mat_<Vec3f>& fusion, int th)
    {
        if (frames.size() == 1)
        {
            fusion = frames[0];
            return;
        }
        int m = frames.size();
        Vec3f** p = new Vec3f*[m];
        Vec3f* out = (Vec3f*)fusion.data;
        for (int i = 0; i < m; i++)
        {
            p[i] = (Vec3f*)frames[i].data;
        }
        int rowIdx=0;
        int idx = 0;
        int total = fusion.total();
        for (int i = 0; i < total; i++)
        {
            float sum = 0;
            bool badPoint = false;
            for (int j = 0; j < m; j++)
            {
                float& v = p[j][i].val[2];
                if (_isnan(v))
                {
                    badPoint = true;
                    memset(out+i, 255, 12);
                    break;
                }
                sum += v;
            }
            if (badPoint)
                continue;
            float ave(sum / m);
            int k = 0;
            Vec3f sumNew = 0;
            for (int n = 0; n < m; n++)
            {
                if (abs(p[n][i].val[2] - ave) <th)
                {
                    sumNew += p[n][i];
                    k++;
                }
            }
            if (k > m/2)
               out[i]= sumNew / k;
            else
                memset(out+i, 255, 12);
        }
        delete[] p;
    }

    void removePointsOnPlane(Mat_<Vec3f>& world, Mat plane)
    {
        Vec3f* p = (Vec3f*)world.data;
        float* pp = (float*)plane.data;
        float len = sqrt(pp[0] * pp[0] + pp[1] * pp[1] + pp[2] * pp[2]);
        for (int i = 0; i < world.total(); i++, p++)
        {
            float* fp = (float*)p;
            if (_isnan(*fp))
                continue;
            float dis = abs(fp[0] * pp[0] + fp[1] * pp[1] + fp[2] * pp[2] - 1000) / len;
            if (dis < 10  /*||(dis>355 && dis<370)*/)
            {
                fp[0] = fp[1] = fp[2] = NAN;
            }
        }
    }
    void removePointsUnderPlane(Mat_<Vec3f>& world, Mat plane)
    {
        Vec3f* p = (Vec3f*)world.data;
        float* pp = (float*)plane.data;
        float len = sqrt(pp[0] * pp[0] + pp[1] * pp[1] + pp[2] * pp[2]);
        for (int i = 0; i < world.total(); i++, p++)
        {
            float* fp = (float*)p;
            if (_isnan(*fp))
                continue;
            float dis = (fp[0] * pp[0] + fp[1] * pp[1] + fp[2] * pp[2] - 1000) / len;
            if (dis > -9  /*||(dis>355 && dis<370)*/)
            {
                fp[0] = fp[1] = fp[2] = NAN;
            }
        }
    }
    Mat estBottom(Mat_<Vec3f>& world, Rect rt)
    {
        Range xRange(rt.x, rt.x+rt.width);
        Range yRange(rt.y, rt.y+rt.height);
        vector<Vec3f> points;
        for (int y = yRange.start; y < yRange.end; y++)
        {
            for (int x = xRange.start; x < xRange.end; x++)
            {
                Vec3f& point = world.at<Vec3f>(y, x);
                if (_isnan(point(2)))
                    continue;
                else
                {
                    points.push_back(point);
                }
            }
        }
        return estPlane(points);
    }
    void estBoxBottom(Mat_<Vec3f>& world)
    {
        Range xRange(250, 500);
        Range yRange(150, 300);

        vector<Vec3f> points;
        vector<Point> ids;
        for (int y = yRange.start; y < yRange.end; y++)
        {
            for (int x = xRange.start; x < xRange.end; x++)
            {
                Vec3f& point = world.at<Vec3f>(y, x);
                if (_isnan(point(2)))
                    continue;
                else
                {
                    points.push_back(point);
                    ids.push_back(Point(x, y));
                }
            }
        }
        Mat plane = estPlane(points);
        /*float len = norm(plane);
        Mat pointMat(points.size(), 3, CV_32F, points.data());
        Mat dis = (pointMat*plane - 1000) / len;
        double minv, maxv;
        minMaxIdx(dis, &minv, &maxv);
        Mat show = Mat::zeros(world.size(), CV_8U);
        for (int i = 0; i < dis.rows; i++)
        {
            float error = dis.at<float>(i);

            if (error<5 && error>-5)
            {
                show.at<uchar>(ids[i]) = 155;
            }
            else if (error<10 && error>-10)
            {
                show.at<uchar>(ids[i]) = 200;
            }
            else if (error<15 && error>-15)
            {
                show.at<uchar>(ids[i]) = 255;
            }
            else
            {
                show.at<uchar>(ids[i]) = 100;
            }

        }
*/

        FileStorage fs("box_bottom_plane", FileStorage::WRITE);
        fs << "plane" << plane;
        fs.release();
    }

    void setBorder(cv::Mat_<Vec3f>& world)
    {
        Vec3f* data = (Vec3f*)world.data;
        for (int i = 0; i < 2; i++)
        {
            int rowIdx = i*world.cols;
            for (int j = 0; j < world.cols; j++)
            {
                data[rowIdx + j] = Vec3f(NAN, NAN, NAN);
            }
        }
        for (int i = world.rows - 2; i < world.rows; i++)
        {
            int rowIdx = i*world.cols;
            for (int j = 0; j < world.cols; j++)
            {
                data[rowIdx + j] = Vec3f(NAN, NAN, NAN);
            }
        }
        int rowIdx = 0;
        for (int i = 0; i < world.rows; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                data[rowIdx + j] = Vec3f(NAN, NAN, NAN);
            }
            rowIdx += world.cols;
        }
        rowIdx = 0;
        for (int i = 0; i < world.rows; i++)
        {
            for (int j = world.cols - 2; j < world.cols; j++)
            {
                data[rowIdx + j] = Vec3f(NAN, NAN, NAN);
            }
            rowIdx += world.cols;
        }

    }



}
