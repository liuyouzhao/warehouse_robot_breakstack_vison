#include "Coordinate.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#ifdef UNIX
#include <stdio.h>
#endif

using namespace cv;
using namespace std;
int globalcount = 0;
double inline Coordinate::norm(float* vec, int len)
{
	double sum = 0;
	for (int i = 0; i < len; i++)
		sum += vec[i] * vec[i];
	return sqrt(sum);
}

Coordinate::Coordinate(float* plane)
{
	double gamma[2], alpha[2];
	gamma[0] = plane[0] / norm(plane, 2);
	alpha[0] = plane[2] / norm(plane + 1, 2);

	gamma[1] = sqrt(1 - gamma[0] * gamma[0]);
	alpha[1] = sqrt(1 - alpha[0] * alpha[0]);

	//R = (Mat_<double>(3, 3) << 1, 0, 0, 0, alpha[0], alpha[1], 0, -alpha[1], alpha[0])*Mat_<double>::eye(3, 3)
	//	*(Mat_<double>(3, 3) << gamma[0], gamma[1], 0, -gamma[1], gamma[0], 0, 0, 0, 1);
	//Mat doublePlane = Mat_<double>(3, 1, plane);//.convertTo(doublePlane, CV_64F);
	//Mat out = doublePlane.t()*R;
	//cout << out;
    R = Mat_<float>(3, 3);
    float* v = (float*)R.data;
	//float* v = new float[9];
	v[6] = plane[0] / norm(plane, 3);
	v[7] = plane[1] / norm(plane, 3);
	v[8] = plane[2] / norm(plane, 3);

	v[3] = 0;
	v[4] = -1 / plane[1];
	v[5] = 1 / plane[2];
	double n = norm(v + 3, 3);
	v[4] /= n;
	v[5] /= n;

	v[0] = v[4] * v[8] - v[5] * v[7];
	v[1] = v[5] * v[6] - v[3] * v[8];
	v[2] = v[3] * v[7] - v[4] * v[6];
	//R = Mat_<float>(3, 3, v);
}

//Coordinate::Coordinate(Mat_<float> plane, Mat_<float> plane2, Point3f o)
//{
//	float* v = new float[9];
//	v[6] = plane[0] / norm(plane, 3);
//	v[7] = plane[1] / norm(plane, 3);
//	v[8] = plane[2] / norm(plane, 3);
//
//	v[3] = 0;
//	v[4] = -1 / plane[1];
//	v[5] = 1 / plane[2];
//	double n = norm(v + 3, 3);
//	v[4] /= n;
//	v[5] /= n;
//
//	v[0] = v[4] * v[8] - v[5] * v[7];
//	v[1] = v[5] * v[6] - v[3] * v[8];
//	v[2] = v[3] * v[7] - v[4] * v[6];
//	R = Mat_<float>(3, 3, v);
//}


Coordinate::~Coordinate()
{
}
void Coordinate::set(float* plane)
{
	double gamma[2], alpha[2];
	gamma[0] = plane[0] / norm(plane, 2);
	alpha[0] = plane[2] / norm(plane + 1, 2);

	gamma[1] = sqrt(1 - gamma[0] * gamma[0]);
	alpha[1] = sqrt(1 - alpha[0] * alpha[0]);

	//R = (Mat_<double>(3, 3) << 1, 0, 0, 0, alpha[0], alpha[1], 0, -alpha[1], alpha[0])*Mat_<double>::eye(3, 3)
	//	*(Mat_<double>(3, 3) << gamma[0], gamma[1], 0, -gamma[1], gamma[0], 0, 0, 0, 1);
	//Mat doublePlane = Mat_<double>(3, 1, plane);//.convertTo(doublePlane, CV_64F);
	//Mat out = doublePlane.t()*R;
	//cout << out;
    R = Mat_<float>(3, 3);
    float* v = (float*)R.data;
	v[6] = plane[0] / norm(plane, 3);
	v[7] = plane[1] / norm(plane, 3);
	v[8] = plane[2] / norm(plane, 3);

	v[3] = 0;
	v[4] = -1 / plane[1];
	v[5] = 1 / plane[2];
	double n = norm(v + 3, 3);
	v[4] /= n;
	v[5] /= n;

	v[0] = v[4] * v[8] - v[5] * v[7];
	v[1] = v[5] * v[6] - v[3] * v[8];
	v[2] = v[3] * v[7] - v[4] * v[6];
}
static int c = 0;

Size_<float> Coordinate::project(float* points, int len)
{
	double minX, maxX, minY, maxY, minZ, maxZ;
	Mat_<float> pMat = R*Mat_<float>(len, 3, points).t();
	minMaxLoc(pMat.row(0), &minX, &maxX);
	minMaxLoc(pMat.row(1), &minY, &maxY);
	minMaxLoc(pMat.row(2), &minZ, &maxZ);
//	std::cout << "��" << globalcount << "��X��"<< maxX - minX << endl;
	//std::cout << "��" << globalcount << "��Y��" << maxY - minY << endl;
	Mat_<uchar> image = Mat::zeros(maxY - minY + 40, maxX - minX + 40, CV_8U);
	t = (Mat_<float>(3, 1) << -minX, -minY, 0);
	vector<Point> pointsPj;
	for (int i = 0; i < len; i++)
	{
		Point pt(pMat(0, i) - minX + 20.5, pMat(1, i) - minY + 20.5);
		image(pt) = 255;
		pointsPj.push_back(pt);
	}

	char str[50];
	sprintf(str, "%d", c++%3);
	////std::cout << count;
	imshow(string("project")+str, image);
	//waitKey(0);
	return Size();
}

Size_<float> Coordinate::getSize(Mat image, vector<Point> points)
{
	vector< vector<Point> > contours;
	dilate(image, image, Mat::ones(5, 5, CV_8U) * 255);
	erode(image, image, Mat::ones(5, 5, CV_8U) * 255);
	erode(image, image, Mat::ones(5, 5, CV_8U) * 255);
	dilate(image, image, Mat::ones(5, 5, CV_8U) * 255);
	findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	/*{
		vector<Point> hull;
		convexHull(points, hull);
		contours.push_back(hull);
	}*/
	drawContours(image, contours, -1, Scalar(255), CV_FILLED);
	RotatedRect rRect = minAreaRect(contours[0]);
	return shrink(image, rRect);
}


Size_<float> Coordinate::shrink(cv::Mat image, RotatedRect rRect)
{
	Range wh;
	Point2f vertices[5];
	rRect.points(vertices);
	vertices[4] = vertices[0];
	//Vec4i line;
	float ratio = 0.3;
	int maxShrink = 10;
	double a, b, c;
	Point2f p0 = vertices[3];
	for (int i = 0; i < 4; i++)
	{
		Point2f p1;
		p1 = vertices[i];
		Point2f v = rRect.center - (p1 + p0)*0.5;
		v *= (1 / cv::norm(v));

		for (int j = 0; j < maxShrink; j++)
		{
			int count = 0;
			LineIterator it(image, p0, p1, 8, true);
			for (int k = 0; k < it.count; k++, ++it)
			{
				if ((**it) > 0)
					count++;
			}
			if (count > ratio*it.count)
				break;
			p0 += v;
			p1 += v;
		}
		vertices[i] = p1;
		if (i == 0)
			vertices[3] = p0;
		else
			vertices[i - 1] = p0;
		p0 = p1;
	}

	cout << cv::norm(vertices[0] - vertices[3]) << endl;
	//cout << cv::norm(vertices[1] - vertices[2]) << endl;
	cout << cv::norm(vertices[0] - vertices[1]) << endl;
	//cout << cv::norm(vertices[2] - vertices[3]) << endl;


	for (int i = 0; i < 4; i++)
		line(image, vertices[i], vertices[(i + 1) % 4], Scalar(255));

	   char temp[] = "show";
	   char count[50];
	   sprintf(count, "%s%d", temp, globalcount);
	   //std::cout << count;
	   imshow(count, image);
	   globalcount++;
	return Size_<float>(cv::norm(vertices[0] - vertices[3]), cv::norm(vertices[1] - vertices[0]));

}

float Coordinate::distance(float* plane, std::vector<cv::Vec3f> points)
{
	float len = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	double v=0;
	for (int i = 0; i < points.size(); i++)
	{
		float sum = -1;
		for (int j = 0; j < 3; j++)
		{
			sum += points[i](j)*plane[j];
		}
		v += sum;
	}
	return v / points.size()/len;
}

/*if (vertices[i].x == vertices[i + 1].x)
{
c = vertices[i].x;
float step;
int start = min(vertices[i].y, vertices[i+1].y);
int end = max(vertices[i].y, vertices[i + 1].y);
if (c > rRect.center.x)
step = -1;
else
step = 1;

for (int j = 0; j < maxShrink; j++)
{
int count = 0;
for (int k = start; k < end; k++)
{
if (image.at<uchar>(k, c) >0)
count++;
}
if (count > ratio*(end - start))
{
vertices[i].x = vertices[i + 1].x = c;
break;
}
c += step;
}

}

else if (vertices[i].y == vertices[i + 1].y)
{
c = -vertices[i].y;
float step;
int start = min(vertices[i].x, vertices[i + 1].x);
int end = max(vertices[i].x, vertices[i + 1].x);
if (c > rRect.center.y)
step = -1;
else
step = 1;

for (int j = 0; j < maxShrink; j++)
{
int count = 0;
for (int k = start; k < end; k++)
{
if (image.at<uchar>(c, k) >0)
count++;
}
if (count > ratio*(end - start))
{
vertices[i].y = vertices[i + 1].y = c;
break;
}
c += step;
}
}

else*/
