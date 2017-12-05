#include "vt_depthTrans.h"
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

namespace vt
{
	void astraDepth2Color(cv::Mat &src, cv::Mat &dst, const float *mat)
	{
		float  z;
		unsigned short   u, v, d;
		unsigned short   u_rgb, v_rgb;
		dst = Mat::zeros(src.rows, src.cols, CV_16U);
		float point[3];
		float max = 0;
		for (v = 0; v < src.rows; v++)
		{
			for (u = 0; u < src.cols; u++)
			{
				d = src.at< unsigned short  >(v, u);
				if (d < 1)
					continue;
				point[0] = u*d;
				point[1] = v*d;
				point[2] = d;
				z = mat[8] * point[0] + mat[9] * point[1] + mat[10] * point[2] + mat[11];
				if (abs(z - d)>abs(max))
					max = z - d;

				u_rgb = (mat[0] * point[0] + mat[1] * point[1] + mat[2] * point[2] + mat[3]) / z + 0.5;
				v_rgb = (mat[4] * point[0] + mat[5] * point[1] + mat[6] * point[2] + mat[7]) / z + 0.5;


				if (u_rgb < 0 || u_rgb >= dst.cols || v_rgb < 0 || v_rgb >= dst.rows) continue;
				unsigned short   *val = (unsigned short   *)dst.ptr<uchar>(v_rgb)+u_rgb;
				*val = z;
			}
		}
		printf("%f ", max);
	}

	void mappingDepth2Color(cv::Mat &src, cv::Mat &dst, const float *mat)
	{
		double  z;
		unsigned short u, v, d;
		unsigned short u_rgb, v_rgb;
		cv::Mat newdepth = Mat::zeros(src.rows, src.cols, CV_16U);
		for (v = 0; v < src.rows; v++)
		{
			for (u = 0; u < src.cols; u++)
			{
				d = src.at<unsigned short>(v, u);
				if (d < 1)
					continue;
				//printf("%d,%d,%d\r\n",u,v,d);
				z = (double)d;
				u_rgb = (unsigned short)((mat[0] * (double)u + mat[1] * (double)v + mat[2] + mat[3] / z));
				v_rgb = (unsigned short)((mat[4] * (double)u + mat[5] * (double)v + mat[6] + mat[7] / z));

				if (u_rgb < 0 || u_rgb >= newdepth.cols || v_rgb < 0 || v_rgb >= newdepth.rows) continue;
				unsigned short *val = (unsigned short *)newdepth.ptr<uchar>(v_rgb)+u_rgb;
				*val = d;
			}
		}
		dst = newdepth;
	}

	Mat toColor(cv::Mat &i_depth)
	{
		if (i_depth.type() != CV_16U)
		{
			printf("depth image format error!");
			return Mat();
		}
		float depthHistogram[65536];
		int numberOfPoints = 0;
		Mat depthHist(i_depth.rows, i_depth.cols, CV_8UC3);
		memset(depthHistogram, 0, sizeof(depthHistogram));
		for (int y = 0; y < i_depth.rows; ++y)
		{
			ushort* depthCell = (ushort*)i_depth.ptr<uchar>(y);
			for (int x = 0; x < i_depth.cols; ++x)
			{
				if (*depthCell != 0)
				{
					depthHistogram[*depthCell]++;
					numberOfPoints++;
				}
				depthCell++;
			}
		}

		for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
		{
			depthHistogram[nIndex] += depthHistogram[nIndex - 1];
		}
		for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
		{
			depthHistogram[nIndex] = (numberOfPoints - depthHistogram[nIndex]) / numberOfPoints;
		}
		for (int y = 0; y < i_depth.rows; ++y)
		{
			ushort* depthCell = (ushort*)i_depth.ptr<uchar>(y);
			uchar * showcell = (uchar *)depthHist.ptr<uchar>(y);
			for (int x = 0; x < i_depth.cols; ++x)
			{
				char depthValue = depthHistogram[*depthCell] * 255;
				*showcell++ = 0;
				*showcell++ = depthValue;
				*showcell++ = 0;
				depthCell++;
			}
		}
		return depthHist;
	}

	void framesFusion(std::vector<cv::Mat>& frames, cv::Mat& fusion, int th)
	{
		MatIterator_<unsigned short> it = fusion.begin<unsigned short>();
		for (int i = 0; i < fusion.rows; i++)
		{
			for (int j = 0; j < fusion.cols; j++)
			{
				float sum = 0;
				for (int n = 0; n < frames.size(); n++)
				{
					sum += frames[n].at<unsigned short>(i,j);
				}
				float ave = sum / frames.size();
				int k = 0;
				float sumNew = 0;
				for (int n = 0; n < frames.size(); n++)
				{
					if (abs(frames[n].at<unsigned short>(i, j) - ave) <th)
					{
						sumNew += frames[n].at<unsigned short>(i, j);
						k++;
					}
				}
				int voteThresh = frames.size() - 2;
				if (k > voteThresh)
					*it++ = sumNew / k;
				else
					*it++ = 0;
			}
		}
	}
    void framesAverage(std::vector<cv::Mat>& frames, cv::Mat& fusion)
    {
        int N = fusion.total();
        int n = frames.size();
        short* value = new short[n];
        short** data = new short*[n];
        for (int i = 0; i < n; i++)
        {
            data[i] = (short*)frames[i].data;
        }
        short* dst = (short*)fusion.data;
        int sum,count;
        for (int i = 0; i < N; i++)
        {
            sum = 0; count = 0;
            for (int j = 0; j < n; j++)
            {
                const short& v = data[j][i];
                if (v > 100)
                {
                    value[count] = v;
                    count++;
                }
            }
            if (count > 10)
            {
                sort(value, value + count);
                /*for (int k = 0; k < count - 8; k++)
                    sum += value[k + 4];*/
                dst[i] = value[count/2];
                //dst[i] = sum / (count - 8.)+0.5;
            }
            else
                dst[i] = 0;
        }
        delete[] value;
        delete[] data;
    }
	void irMaxRemove(unsigned short* pIr, int resolution, Mat& depthMat)
	{
		const unsigned short max = 1022;
		int bin;
		int& width = depthMat.cols;
		int& height = depthMat.rows;
		switch (resolution)
		{
		case 0:bin = 3;
			break;
		case 1:bin = 1;
			break;
		}
		int len = bin * 2 + 1;
		int area = len*len;
		for (int i = bin; i < height - bin-8; i++)
		{
			for (int j = bin; j < width - bin; j++)
			{
				if (pIr[i*width + j] == max)
					for (int k = 0; k < area; k++)
						depthMat.at<unsigned short>(i + k / len - bin+8, j + k % len - bin) = 0;
			}
		}

	}

	template void depth2World(Mat_<float> depth, Mat_<Vec3f> worldMat, float* camera, float* k);
	template void depth2World(Mat_<ushort> depth, Mat_<Vec3f> worldMat, float* camera, float* k);

	template <class T>
	void depth2World(Mat_<T> depth, Mat_<Vec3f> worldMat, float* camera, float* k)
	{
		float fx = camera[0];
		float fy = camera[4];
		float u0 = camera[2];
		float v0 = camera[5] + 8;
		float k1 = k[0]; float k2 = k[1]; float k3 = k[4];
		float p1 = k[2]; float p2 = k[3];
        const int width = depth.cols;
		const int height = depth.rows;
		Vec3f* pos = (Vec3f*)worldMat.data;

		for (int i = 0; i < height; i++)
		{
			int rowPos = i*width;
			for (int j = 0; j < width; j++)
			{
				float* p = (float*)(pos + rowPos + j);
				p[2] = depth(i, j);
				if (p[2]>10)
				{
					float u = (j - u0) / fx;
					float v = (i - v0) / fy;
					double r2 = u*u + v*v;
					p[0] = (u /*+ (u*((k3*r2 + k2)*r2 + k1)*r2 + 2 * p1*u*v + p2*(r2 + 2 * u*u))*/)*p[2];
					p[1] = (v /*+ (v*((k3*r2 + k2)*r2 + k1)*r2 + 2 * p2*u*v + p1*(r2 + 2 * v*v))*/)*p[2];
				}
                else
                {
                    p[0] = p[1] = p[2] = NAN;
                }
			}
		}
	}

    void edgeRemove(cv::Mat& depth, int neighbor)
    {
        short* data = (short*)depth.data;
        vector<int> list;
        int width = depth.cols;
        int border = 10;
        int rowidx = border*width;
        for (int i = border; i < depth.rows-border; i++)
        {
            int idx = rowidx + border;
            for (int j = border; j < depth.cols-border; j++,idx++)
            {

                if (data[idx] < 100)
                {
                    if (data[idx + 1]>100)
                    {
                        list.push_back(idx + 1);
                        if (neighbor == 2)
                        {
                            list.push_back(idx + 2);
                            list.push_back(idx + 1 + width);
                            list.push_back(idx + 1 - width);
                        }
                    }
                    if (data[idx - 1] > 100)
                    {
                        list.push_back(idx - 1);
                        if (neighbor == 2)
                        {
                            list.push_back(idx - 2);
                            list.push_back(idx - 1 + width);
                            list.push_back(idx - 1 - width);
                        }
                    }
                    if (data[idx + width] > 100)
                    {
                        list.push_back(idx + width);
                        if (neighbor == 2)
                        {
                            list.push_back(idx + width*2);
                            list.push_back(idx + 1 + width);
                            list.push_back(idx - 1 + width);
                        }
                    }
                    if (data[idx - width] > 100)
                    {
                        list.push_back(idx - width);
                        if (neighbor == 2)
                        {
                            list.push_back(idx - 2*width);
                            list.push_back(idx + 1 - width);
                            list.push_back(idx - 1 - width);
                        }
                    }
                }
            }
            rowidx += width;

        }
        for (int i = 0; i < list.size(); i++)
        {
            data[list[i]] = 0;
        }
    }
    template void depth2World<short>(Mat_<short> depth, Mat_<Vec3f> worldMat, float* camera, float* k);
}
