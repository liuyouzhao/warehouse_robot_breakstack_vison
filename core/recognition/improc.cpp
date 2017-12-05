/*
 * improc.c
 *
 *  Created on: May 2, 2017
 *      Author: hujia
 */
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

cv::Mat clipout_target_image(cv::Mat target)
{
	int i = 0;
	int j = 0;
	int w = target.cols;
	int h = target.rows;
    int left = w;
    int right = 0;
    int top = h;
    int bottom = 0;
	for( ; i < h; i ++ )
	{
		for( j = 0; j < w; j ++ )
		{
            unsigned char dat = target.data[i * w + j];
            if(dat == 255)
			{
                left = left > j ? j : left;
                right = right < j ? j : right;
                top = top > i ? i : top;
                bottom = bottom < i ? i : bottom;
			}
		}
	}
	int tw = right - left;
	int th = bottom - top;


	cv::Mat out = cv::Mat::zeros(th, tw, CV_8U);
	for( i = 0; i < th; i ++ )
	{
		memcpy(out.data + i * tw, target.data + ((i + top) * w) + left, tw);
	}

	return out;
}

cv::Mat clipout_target_image(cv::Mat target, int &l, int &r, int &t, int &b)
{
    int i = 0;
    int j = 0;
    int w = target.cols;
    int h = target.rows;
    int left = w;
    int right = 0;
    int top = h;
    int bottom = 0;
    for( ; i < h; i ++ )
    {
        for( j = 0; j < w; j ++ )
        {
            unsigned char dat = target.data[i * w + j];
            if(dat == 255)
            {
                left = left > j ? j : left;
                right = right < j ? j : right;
                top = top > i ? i : top;
                bottom = bottom < i ? i : bottom;
            }
        }
    }
    int tw = right - left;
    int th = bottom - top;


    cv::Mat out = cv::Mat::zeros(th, tw, CV_8U);
    for( i = 0; i < th; i ++ )
    {
        memcpy(out.data + i * tw, target.data + ((i + top) * w) + left, tw);
    }

    l = left;
    r = right;
    t = top;
    b = bottom;
    return out;
}

cv::Mat clipout_target_image_remain(cv::Mat target, int &l, int &r, int &t, int &b)
{
    int i = 0;
    int j = 0;
    int w = target.cols;
    int h = target.rows;
    int left = w;
    int right = 0;
    int top = h;
    int bottom = 0;
    for( ; i < h; i ++ )
    {
        for( j = 0; j < w; j ++ )
        {
            unsigned char dat = target.data[i * w + j];
            if(dat == 255)
            {
                left = left > j ? j : left;
                right = right < j ? j : right;
                top = top > i ? i : top;
                bottom = bottom < i ? i : bottom;
            }
        }
    }
    left = left - 2;
    top = top - 2;
    right = right + 2;
    bottom = bottom + 2;
    int tw = right - left;
    int th = bottom - top;


    cv::Mat out = cv::Mat::zeros(th, tw, CV_8U);
    for( i = 0; i < th; i ++ )
    {
        memcpy(out.data + i * tw, target.data + ((i + top) * w) + left, tw);
    }

    l = left;
    r = right;
    t = top;
    b = bottom;
    return out;
}

static void clean_queue_set_value(std::vector< cv::Point2i > &queue, cv::Mat &cliped, int v)
{
	int z = 0;
	int w = cliped.cols;
	for( z = 0; z < queue.size(); z ++ )
	{
		int tx = queue[z].x;
		int ty = queue[z].y;
		cliped.data[ty * w + tx] = 0;
	}
	queue.clear();
}

cv::Mat fill_spots_ext(cv::Mat &cliped)
{
	static std::vector< cv::Point2i > queue;

	int i = 0;
	int j = 0;
	int z = 0;
	int x = 0;
	int y = 0;
	int w = cliped.cols;
	int h = cliped.rows;
	int queue_ptr = 0;
	for( i = 0; i < h; i ++ )
	{
		for( j = 0; j < w; j ++ )
		{
			int dat = cliped.data[i * w + j];
			if(dat < 125)
			{
				cliped.data[i * w + j] = 1;
			}
			else if(dat > 125)
			{
				cliped.data[i * w + j] = 255;
			}
		}
	}

	for( i = 0; i < h; i ++ )
	{
		for( j = 0; j < w; j ++ )
		{
			int dat = cliped.data[i * w + j];
			x = j;
			y = i;
			queue_ptr = 0;
			if(dat == 1)
			{
				queue.push_back(cv::Point2i(j, i));
			}
			while(1)
			{
				if(dat == 1 || dat == 2)
				{
					int up = y - 1;
					int down = y + 1;
					int left = x - 1;
					int right = x + 1;
					if(up < 0 || down > h - 1 || left < 0 || right > w - 1)
					{
						clean_queue_set_value(queue, cliped, 0);
						break;
					}

					/* up point value */
					if(cliped.data[up * w + x] == 1) {
						queue.push_back(cv::Point2i(x, up));
						cliped.data[up * w + x] = 2;
					}
					else if(cliped.data[up * w + x] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* down point value */
					if(cliped.data[down * w + x] == 1) {
						queue.push_back(cv::Point2i(x, down));
						cliped.data[down * w + x] = 2;
					}
					else if(cliped.data[down * w + x] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* left point value */
					if(cliped.data[y * w + left] == 1) {
						queue.push_back(cv::Point2i(left, y));
						cliped.data[y * w + left] = 2;
					}
					else if(cliped.data[y * w + left] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* right point value */
					if(cliped.data[y * w + right] == 1) {
						queue.push_back(cv::Point2i(right, y));
						cliped.data[y * w + right] = 2;
					}
					else if(cliped.data[y * w + right] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* left-up value*/
					if(cliped.data[up * w + left] == 1) {
						queue.push_back(cv::Point2i(left, up));
						cliped.data[up * w + left] = 2;
					}
					else if(cliped.data[up * w + left] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* right-up value*/
					if(cliped.data[up * w + right] == 1) {
						queue.push_back(cv::Point2i(right, up));
						cliped.data[up * w + right] = 2;
					}
					else if(cliped.data[up * w + right] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* left-down value */
					if(cliped.data[down * w + left] == 1) {
						queue.push_back(cv::Point2i(left, down));
						cliped.data[down * w + left] = 2;
					}
					else if(cliped.data[down * w + left] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
					/* right-down value */
					if(cliped.data[down * w + right] == 1) {
						queue.push_back(cv::Point2i(right, down));
						cliped.data[down * w + right] = 2;
					}
					else if(cliped.data[down * w + right] == 0) {
						clean_queue_set_value(queue, cliped, 0);
						break;
					}
				}

				if(queue_ptr >= queue.size() || queue.size() == 0)
				{
					break;
				}

				x = queue[queue_ptr].x;
				y = queue[queue_ptr].y;

				dat = cliped.data[y * w + x];

				queue_ptr ++;

			}
			if(queue.size() != 0)
			{
				printf("queue: %d\n", (int) queue.size());
			}
			for( z = 0; z < queue.size(); z ++ )
			{
				int tx = queue[z].x;
				int ty = queue[z].y;
				cliped.data[ty * w + tx] = 255;
			}
			queue.clear();
		}
	}

	for( i = 0; i < h; i ++ )
	{
		for( j = 0; j < w; j ++ )
		{
			int dat = cliped.data[i * w + j];
			if(dat != 0 && dat != 255)
			{
				printf("err: %d\n", dat);
			}
		}
	}

	return cliped;
}

cv::Mat fill_spots(cv::Mat cliped)
{
	std::vector< std::vector< cv::Point > > contours;
	std::vector< cv::Vec4i > hierarchy;
	cv::Mat dst = cv::Mat::zeros(cliped.rows, cliped.cols, CV_8U);
	cv::Mat src = cliped.clone();

	cv::findContours( src, contours, hierarchy,
			CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
		cv::drawContours( dst, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
	}

	return dst;
}

cv::Mat analysis_bin_map_vertical(cv::Mat bin)
{
	int i = 0;

	//vertical histogram
	cv::Mat vertical(1, bin.cols, CV_8U);
	vertical = cv::Scalar::all(0);

	for( i = 0; i < bin.cols; i++)
	{
		vertical.data[i] = cv::countNonZero(bin(cv::Rect(i, 0, 1, bin.rows)));
	}

	return vertical;
}

cv::Mat analysis_bin_map_horizon(cv::Mat bin)
{
	int i = 0;

	//vertical histogram
	cv::Mat horizon(bin.rows, 1, CV_8U);
	horizon = cv::Scalar::all(0);

	for( i = 0; i < bin.rows; i++)
	{
		horizon.data[i] = cv::countNonZero(bin(cv::Rect(0, i, bin.cols, 1)));
	}

	return horizon;
}

cv::Mat corner_detect(cv::Mat src_bin)
{
	/// Detector parameters
	int block_size = 20;
	int aperture_size = 30;
	double k = 0.04;
	int thresh = 200;
	cv::Mat dst = src_bin.clone();
	cv::Mat dst_norm;
	cv::Mat dst_norm_scaled;

	/// Detecting corners
	cv::cornerHarris( src_bin, dst, block_size, aperture_size, k, cv::BORDER_DEFAULT );
	/// Normalizing
	cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_8U, cv::Mat());
	cv::convertScaleAbs(dst_norm, dst_norm_scaled);

	/// Drawing a circle around corners
	for (int j = 0; j < dst_norm.rows; j++) {

		for (int i = 0; i < dst_norm.cols; i++) {

			if ((int) dst_norm.at<float>(j, i) > thresh) {
				cv::circle(dst_norm_scaled, cv::Point(i, j), 5, cv::Scalar(0), 2, 8, 0);
			}
		}
	}
	/// Showing the result

	return dst_norm_scaled;
}

cv::Mat edge_detect(cv::Mat src)
{
	cv::Mat dst, detected_edges;
	int ratio = 3;
	int kernel_size = 3;
	int lowThreshold = 30;

	/// Reduce noise with a kernel 3x3
	cv::blur( src, detected_edges, cv::Size(24, 24) );

	/// Canny detector
	cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

	/// Using Canny's output as a mask, we display our result
	dst = cv::Scalar::all(0);
	src.copyTo( dst, detected_edges);
	return dst;
}


cv::Mat rotate_image(cv::Mat src, float angle)
{
	cv::Mat dst;
	cv::Mat m;
	m = cv::getRotationMatrix2D(cv::Point2f((float)src.cols/2.0f, (float)src.rows/2.0f), angle, 1.0);
	cv::warpAffine(src, dst, m, cv::Size(src.cols, src.rows));
	return dst;
}
