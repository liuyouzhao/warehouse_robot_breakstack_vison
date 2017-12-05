#include "PlaneFinder.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

PlaneFinder::PlaneFinder()
{
}


PlaneFinder::~PlaneFinder()
{
}

cv::Mat PlaneFinder::approxPlane(const cv::Mat& points)
{
	Mat one = Mat::ones(points.rows, 1, CV_32F);
	return points.inv(DECOMP_SVD)*one;
}

float PlaneFinder::distance(float* point, float* plane, float len)
{
	float sum = -1;
	for (int i = 0; i < 3; i++)
	{
		sum += point[i] * plane[i];
	}
	return sum/len;
}

void PlaneFinder::ransacPlane(int num, int k, float threshold, int nearby)
{
	bool notSucceed = false;
	Vec3f randPoints[3];
	float* curPoint;
	Mat& points = m_points;
	int iterations = 0;

	while (iterations++ < k)
	{
		for (int i = 0; i < 3; i++)
		{
			randPoints[i] = points.at<Vec3f>(rand() % points.rows);
		}
		Mat plane = approxPlane(Mat(3, 3, CV_32F, (char*)randPoints));

		vector<Vec3f> curPlane;
		vector<Vec3f> remain;
		for (int j = 0; j < 4; j++)
		{
			float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
			curPoint = (float*)points.data;
			curPlane.clear();
			remain.clear();

			for (int i = 0; i < points.rows; i++, curPoint += 3)
			{
				if (threshold>abs(distance(curPoint, (float*)plane.data, len)))
				{
					curPlane.push_back(*(Vec3f*)curPoint);
				}
				else
				{
					remain.push_back(*(Vec3f*)curPoint);
				}
			}
			plane = approxPlane(Mat(curPlane.size(), 3, CV_32F, (char*)curPlane.data()));
		}
		if (curPlane.size() > num)
		{
			if (!m_pointList.empty() && curPlane.size() > m_pointList.front().size())
			{
				m_pointList.push_front(curPlane);
				m_planes.push_front(plane);
			}
			else
			{

				m_pointList.push_back(curPlane);
				m_planes.push_back(plane);
			}
			points = Mat_<Vec3f>(remain, true);
		}
	}
	printf("%d  %d", m_pointList.size(), m_planes.size());

}

bool PlaneFinder::isPlane(Mat& plane, Mat& ptOnPlane, Mat& points, int num, float threshold)
{
	int count;
	vector<Vec3f> ptList;
	vector<Vec3f> remain;
	for (int j = 0; j < 2; j++)
	{
		float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
		float* curPoint = (float*)points.data;
		count = 0;
		ptList.clear();
		remain.clear();
		for (int i = 0; i < points.rows; i++, curPoint += 3)
		{
			if (threshold>abs(distance(curPoint, (float*)plane.data, len)))
			{
				count++;
				ptList.push_back(*(Vec3f*)curPoint);
			}
			else
			{
				remain.push_back(*(Vec3f*)curPoint);
			}
		}
		if (count < num)
			return false;
		plane = approxPlane(Mat(ptList.size(), 3, CV_32F, (char*)ptList.data()));
	}
	points = Mat_<Vec3f>(remain, true);
	ptOnPlane = Mat_<Vec3f>(ptList, true);
	return true;
}

bool PlaneFinder::isPlane(Mat& plane, vector<Vec3f>& ptOnPlane, vector<Vec3f>& points, int num, float threshold)
{
	int count;
	vector<Vec3f> ptList;
	vector<Vec3f> remain;
	for (int j = 0; j < 2; j++)
	{
		float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
		float* curPoint = (float*)points.data();
		count = 0;
		ptList.clear();
		remain.clear();
		for (int i = 0; i < points.size(); i++, curPoint += 3)
		{
			if (threshold>abs(distance(curPoint, (float*)plane.data, len)))
			{
				count++;
				ptList.push_back(*(Vec3f*)curPoint);
			}
			else
			{
				remain.push_back(*(Vec3f*)curPoint);
			}
		}
		if (count < num)
			return false;
		plane = approxPlane(Mat(ptList.size(), 3, CV_32F, (char*)ptList.data()));
	}
	points = remain;
	ptOnPlane = ptList;
	return true;
}
void PlaneFinder::removePoints(cv::Mat& plane, std::vector<cv::Vec3f>& ptOnPlane, cv::Mat& plane2, float threshold)
{
	vector<Vec3f> remain;
	float len = sqrt((Mat_<float>(plane2.t()*plane2))(0, 0));
	float* curPoint = (float*)ptOnPlane.data();
	for (int i = 0; i < ptOnPlane.size(); i++, curPoint += 3)
	{
		float dis = distance(curPoint, (float*)plane2.data, len);
		if (-threshold<(dis))
		{
			continue;
		}
		else
		{
			remain.push_back(*(Vec3f*)curPoint);
		}
	}
	plane = approxPlane(Mat(remain.size(), 3, CV_32F, (char*)remain.data()));
	ptOnPlane = remain;
}
void PlaneFinder::removePoints(cv::Mat_<cv::Vec3f>& mat, cv::Mat& plane, std::vector<int>& ptOnPlane, cv::Mat& plane2, float threshold)
{
	vector<int> remain;
	vector<Vec3f> pts;
	float len = sqrt((Mat_<float>(plane2.t()*plane2))(0, 0));

	for (int i = 0; i < ptOnPlane.size(); i++)
	{
		float* curPoint = ((float*)mat.data) + ptOnPlane[i]*3;
		float dis = distance(curPoint, (float*)plane2.data, len);
		if (-threshold<(dis))
		{
			continue;
		}
		else
		{
			remain.push_back(ptOnPlane[i]);
			pts.push_back(*(Vec3f*)curPoint);
		}
	}
	plane = approxPlane(Mat(pts.size(), 3, CV_32F, (char*)pts.data()));
	ptOnPlane = remain;
}
bool PlaneFinder::setTable(Mat& plane, vector<Vec3f>& ptOnPlane, vector<Vec3f>& points, int num, float threshold)
{
	int count;
	vector<Vec3f> ptList;
	vector<Vec3f> remain;
	for (int j = 0; j < 2; j++)
	{
		float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
		float* curPoint = (float*)points.data();
		count = 0;
		ptList.clear();
		remain.clear();
		for (int i = 0; i < points.size(); i++, curPoint += 3)
		{
			float dis = distance(curPoint, (float*)plane.data, len);
			if (threshold>abs(dis))
			{
				count++;
				ptList.push_back(*(Vec3f*)curPoint);
			}
			else if (dis < 0)
			{
				remain.push_back(*(Vec3f*)curPoint);
			}
		}
		if (count < num)
			return false;
		plane = approxPlane(Mat(ptList.size(), 3, CV_32F, (char*)ptList.data()));
	}
	points = remain;
	ptOnPlane = ptList;
	std::ofstream fs("1.txt");

	/*for (int i = 0; i < ptList.size(); i++)
	{
		fs << ptList[i](0) << ";" << ptList[i](1) << ";" << -ptList[i](2) << endl;
	}*/
	return true;
}

bool PlaneFinder::setTable(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& points, std::vector<cv::Vec3f>& world,
	std::vector<int>& ptOnPlane, std::vector<cv::Vec3f>& ptsPlane, int num, float threshold, bool fix)
{

	int count;
	vector<int> ptList;
	vector<int> remain;

	MatConstIterator_<Vec3f> it = mat.begin();
	Vec3f* p = (Vec3f*)mat.data;
	vector<Vec3f> pts;
	for (int j = 0; j < 3; j++)
	{
		float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
		count = 0;
		ptList.clear();
		remain.clear();
		world.clear();
		pts.clear();
		for (int i = 0; i < points.size(); i++)
		{
			float* curPoint = (float*)(&p[points[i]]);
			float dis = distance(curPoint, (float*)plane.data, len);
			if (threshold>abs(dis))
			{
				count++;
				ptList.push_back(points[i]);
				pts.push_back(p[points[i]]);
			}
			else if (dis < 0)
			{
				remain.push_back(points[i]);
				world.push_back(p[points[i]]);
			}
		}
		if (count < num)
			return false;
		if (fix)
			break;
		plane = approxPlane(Mat(pts.size(), 3, CV_32F, (char*)pts.data()));
	}
	points = remain;
	ptOnPlane = ptList;
	ptsPlane = pts;
	//std::ofstream fs("1.txt");
	//for (int i = 0; i < ptList.size(); i++)
	//{
	//	fs << p[ptList[i]](0) << ";" << p[ptList[i]](1) << ";" << -p[ptList[i]](2) << endl;
	//}
	return true;
}
bool PlaneFinder::setTableTemp(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& points, std::vector<int>& ptOnPlane, int num, float threshold)
{

	int count;
	vector<int> ptList;
	vector<int> remain;
	MatConstIterator_<Vec3f> it = mat.begin();
	Vec3f* p = (Vec3f*)mat.data;
	vector<Vec3f> pts;
	for (int j = 0; j < 2; j++)
	{
		float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
		count = 0;
		ptList.clear();
		remain.clear();
		pts.clear();
		for (int i = 0; i < points.size(); i++)
		{
			float* curPoint = (float*)(&p[points[i]]);
			float dis = distance(curPoint, (float*)plane.data, len);
			if (threshold>abs(dis))
			{
				count++;
				ptList.push_back(points[i]);
				pts.push_back(p[points[i]]);
			}
			else if (dis < 0)
			{
				remain.push_back(points[i]);
			}
		}
		if (count < num)
			return false;
		//plane = approxPlane(Mat(pts.size(), 3, CV_32F, (char*)pts.data()));
	}
	points = remain;
	ptOnPlane = ptList;
	std::ofstream fs("1.txt");
	for (int i = 0; i < ptList.size(); i++)
	{
		fs << p[ptList[i]](0) << ";" << p[ptList[i]](1) << ";" << -p[ptList[i]](2) << endl;
	}
	return true;
}
bool PlaneFinder::comp(pair<int, Mat> i, pair<int, Mat> j)
{
	return (i.first>j.first);
}

void PlaneFinder::ransacPlane2(int num, int k, float threshold, int nearby)
{
	remainpoint = m_cloud;
	bool notSucceed = false;
	Vec3f randPoints[3];
	float* curPoint;
	int iterations = 0;
	vector< pair<int, Mat> > planeList;
	int planeCount = 0;
	while (iterations++ < k)
	{
		for (int i = 0; i < 3; i++)
		{
			randPoints[i] = remainpoint[(rand() % remainpoint.size())];
		}
		Mat plane = approxPlane(Mat(3, 3, CV_32F, (char*)randPoints));
		vector<Vec3f> ptOnPlane;
		if (isPlane(plane, ptOnPlane, remainpoint, num, threshold))
			planeList.push_back(pair<int, Mat>(ptOnPlane.size(), plane));
		if (remainpoint.size() < num)
			break;
	}
	std::sort(planeList.begin(), planeList.end(), comp);
	cout << "max plane point " << planeList[0].first << endl;
	remainpoint = m_cloud;


	////////////////////////cpx///////////////////////
	//if (planeList.size() > 0)
	//{
	//	Mat plane = planeList[0].second;
	//	vector<Vec3f> ptOnPlane;
	//	if (setTable(plane, ptOnPlane, points, num, threshold))
	//	{
	//		m_plList.push_back(pair<Mat, vector<Vec3f>>(plane, ptOnPlane));
	//		m_planes.push_back(plane);
	//		m_pointList.push_back(ptOnPlane);
	//	}
	//}


	if (planeList.size() >0)
	{
		//for (int tempx = 0; tempx <1; tempx++)
		//{
			Mat plane = planeList[0].second;
			vector<Vec3f> ptOnPlane;
			if (setTable(plane, ptOnPlane, remainpoint, num, threshold))
			{
				m_plList.push_back(pair< Mat, vector<Vec3f> >(plane, ptOnPlane));
				std::cout << plane << endl;
				m_planes.push_back(plane);
				m_pointList.push_back(ptOnPlane);
			}
		//}

	}

	//////////////////////////////////////////////////////////////////////

	for (int i = 1; i < 3; i++)
	{
		Mat plane = planeList[i].second;
		vector<Vec3f> ptOnPlane;
		if (isPlane(plane, ptOnPlane, remainpoint, num, threshold))
		{
			m_plList.push_back(pair< Mat, vector<Vec3f> >(plane, ptOnPlane));//3��ƽ��ĳߴ硢ԭʼ����Ϣ
			m_planes.push_back(plane);       //ƽ���a,b,c
			m_pointList.push_back(ptOnPlane);//3��ƽ���У�ÿ��ƽ��ĵ�
		}
	}
}


bool PlaneFinder::setPoints(const cv::Mat& cloud, cv::Rect window, float near, float far)
{
	const Mat_<Vec3f> roi(cloud, window);
	MatConstIterator_<Vec3f> it = roi.begin();

	while (it != roi.end())
	{
		if ((*it)(2) > near && (*it)(2) < far)
			m_cloud.push_back(*it);
		it++;
	}
	m_points = Mat(m_cloud, true);

	if (m_cloud.size() > 2)
		return true;
	else
		return false;
}


void PlaneFinder::project(float* point, float* plane, float len, float* prjPoint)
{
	double sum = -1;
	for (int i = 0; i < 3; i++)
	{
		sum += point[i] * plane[i];
	}
	double k = sum / len;
	for (int i = 0; i < 3; i++)
	{
		prjPoint[i] = point[i] - k*plane[i];
		//cout << prjPoint[i] << " ";
	}

	//sum = 0;
	//for (int i = 0; i < 3; i++)
	//{
	//	sum += prjPoint[i] * plane[i];
	//}
	//cout << sum;

}

void PlaneFinder::planeEva(cv::Mat mat, cv::Mat plane, vector<int> ptList, vector<Vec3f> pts)
{
	Mat points(pts.size(), 3, CV_32F, pts.data());
	Mat one = Mat::ones(pts.size(), 1, CV_32F);
	float len = sqrt((Mat_<float>(plane.t()*plane))(0, 0));
	Mat error = (points*plane - one) / len;
	for (int i = 0; i < ptList.size(); i++)
	{
		mat.at<Vec3b>(ptList[i])(1) = 0;
		float e = error.at<float>(i);
		if (e<2 && e>-2)
			mat.at<Vec3b>(ptList[i])(0) = 255;
		else if (e < 3 && e>0)
			mat.at<Vec3b>(ptList[i])(1) = 255;
		else if (e<0 && e>-3)
			mat.at<Vec3b>(ptList[i])(2) = 255;
		else if (e>=3)
		{
			mat.at<Vec3b>(ptList[i])(2) = 255;
			mat.at<Vec3b>(ptList[i])(1) = 255;
		}
		else if (e<=-3)
		{
			mat.at<Vec3b>(ptList[i])(0) = 255;
			mat.at<Vec3b>(ptList[i])(1) = 255;
		}
	}
	double minV, maxV;
	minMaxIdx(error, &minV, &maxV);
	printf("max value %f, min value %f", maxV, minV);
	Mat show;
	resize(mat, show, mat.size() * 3);
	imshow("planez", mat);
	waitKey(0);
}

bool PlaneFinder::getNorm(cv::Mat_<cv::Vec3f>& mat, int idx, cv::Vec3f& norm)
{
    int width = mat.cols;
    if (idx / width < 1 || idx / width == (mat.rows - 1) || idx%width == 0 || idx%width == (mat.cols - 1))
        return false;
    Vec3f p[4];
    p[0] = mat(idx - width);
    p[1] = mat(idx - 1);
    p[2] = mat(idx + 1);
    p[3] = mat(idx + width);
    for (int i = 0; i < 4; i++)
    {
        if (p[i][2] < 10)
            return false;
    }
    Vec3f v1 = p[3] - p[0];
    Vec3f v2 = p[2] - p[1];
    norm = v1.cross(v2);
    norm = norm / cv::norm(norm);
    return true;
}
std::vector<int> PlaneFinder::planeVerify(cv::Mat& plane, cv::Mat_<cv::Vec3f>& mat, std::vector<int>& ptOnPlane)
{
    Vec3f pl;
    for (int i = 0; i < 3; i++)
        pl[i] = plane.at<float>(i);
    pl = pl / cv::norm(pl);
    std::vector<int> list;
    for (int i = 0; i < ptOnPlane.size(); i++)
    {
        Vec3f norm;
        if (getNorm(mat, ptOnPlane[i], norm))
        {
            float sum = pl[0] * norm[0] + pl[1] * norm[1] + pl[2] * norm[2];
            if (abs(sum) < 0.7)
                list.push_back(ptOnPlane[i]);
            //printf("sum = %f; ", sum);
        }
    }
    return list;
}

bool PlaneFinder::estimateNorm(cv::Mat_<cv::Vec3f>& mat, int wz, cv::Mat_<cv::Vec3f>& norm)
{
    Mat_<uchar> marker = Mat::ones(mat.size(), CV_8U);
    Mat_<float> diff[6];
    float* pdiff[6];
    for (int i = 0; i < 6; i++)
    {
        diff[i] = Mat::zeros(mat.size(), CV_32F);
        pdiff[i] = (float*) diff[i].data;
    }
    int ridx, idx;
    Vec3f* pdata = (Vec3f*) mat.data;
    for (int i = 1; i < mat.rows-1; i++)
    {
        ridx = i*mat.cols;
        for (int j = 1; j < mat.cols-1; j++)
        {
            idx = ridx + j;
            if (pdata[idx][2] < 100)
            {
                marker.data[idx + 1] = 0;
                marker.data[idx - 1] = 0;
                marker.data[idx + mat.cols] = 0;
                marker.data[idx - mat.cols] = 0;
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
    Mat xxx(mat.size(), CV_32F, pdiff[0]);
    for (int k = 0; k < 6; k++)
    {
        integral(diff[k], inDiff[k]);
    }
    float* pIn[6];
    for (int i = 0; i < 6; i++)
    {
        pIn[i] = (float*)inDiff[i].data;
    }
    int ridx2, idx2;
    float v[6];
    int validThre = (wz + 1)*(wz + 1);
    Vec3f* pNorm = (Vec3f*) norm.data;
    for (int i = 1+wz; i < mat.rows - wz-1; i++)
    {
        for (int j = wz+1; j < mat.cols - wz-1; j++)
        {

            int valid = inMarker.at<int>(i - wz, j - wz) + inMarker.at<int>(i + 1 + wz, j + 1 + wz) -
                inMarker.at<int>(i + 1 + wz, j - wz) - inMarker.at<int>(i - wz, j + 1 + wz);
            if (valid < validThre)
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
            norm.at<Vec3f>(i, j) = vec;
        }
    }
    return true;
}


void PlaneFinder::planeVerify2(cv::Mat& plane, cv::Mat_<cv::Vec3f>& normMat)
{
    Vec3f pl;
    for (int i = 0; i < 3; i++)
        pl[i] = plane.at<float>(i);
    pl = pl / cv::norm(pl);
    Mat x = Mat::zeros(normMat.size(), CV_8U);
    for (int i = 0; i < normMat.total(); i++)
    {
        Vec3f norm = normMat(i);
        float sum = pl[0] * norm[0] + pl[1] * norm[1] + pl[2] * norm[2];
        if (abs(sum) > 0.9)
            x.at<uchar>(i) = 255;

    }

}
