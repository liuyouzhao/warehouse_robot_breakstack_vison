#include "CameraPara.h"
#include <fstream>

namespace t3d
{
	using namespace cv;
	using namespace std;

	void loadAstraParams(std::string path, Mat& L2R, Mat intri[4])
	{
		Mat_<float> left = Mat_<float>::zeros(3, 3);
		Mat_<float> right = Mat_<float>::zeros(3, 3);
		Mat_<float> R = Mat_<float>::zeros(3, 3);
		Mat_<float> t = Mat_<float>::zeros(3, 1);
#ifdef UNIX
		ifstream fs(path.c_str());
#else
		ifstream fs(path);
#endif
		char buffer[100];
		fs.getline(buffer, 100);
		MatIterator_<float> it;
		for (it = left.begin(); it != left.end(); it++)
		{
			fs >> *it;
		}
		Mat left44 = Mat::zeros(4, 4, CV_32F);
		left.copyTo(Mat(left44, Range(0, 3), Range(0, 3)));
		intri[0] = left;
		left44.at<float>(3, 3) = 1;

		fs.getline(buffer, 100);
		fs.getline(buffer, 100);
		for (it = right.begin(); it != right.end(); it++)
		{
			fs >> *it;
		}
		Mat right44 = Mat::zeros(4, 4, CV_32F);
		right.copyTo(Mat(right44, Range(0, 3), Range(0, 3)));
		intri[1] = right;
		right44.at<float>(3, 3) = 1;

		fs.getline(buffer, 100);
		fs.getline(buffer, 100);
		for (it = R.begin(); it != R.end(); it++)
		{
			fs >> *it;
		}
		fs.getline(buffer, 100);
		fs.getline(buffer, 100);
		for (it = t.begin(); it != t.end(); it++)
		{
			fs >> *it;
		}
		Mat R2L_Mat = Mat::zeros(4, 4, CV_32F);
		R.copyTo(Mat(R2L_Mat, Range(0, 3), Range(0, 3)));
		t.copyTo(Mat(R2L_Mat, Range(0, 3), Range(3, 4)));
		intri[2] = R;
		intri[3] = t;
		R2L_Mat.at<float>(3, 3) = 1;
		L2R = right44*R2L_Mat*left44.inv();
	}

    void loadAstraParamsD(std::string path, Mat& L2R, Mat intri[6])
    {
        Mat_<double> left = Mat_<double>::zeros(3, 3);
        Mat_<double> right = Mat_<double>::zeros(3, 3);
        Mat_<double> R = Mat_<double>::zeros(3, 3);
        Mat_<double> t = Mat_<double>::zeros(3, 1);
        intri[4] = Mat_<double>::zeros(1, 5);
        intri[5] = Mat_<double>::zeros(1, 5);
#ifdef UNIX
        ifstream fs(path.c_str());
#else
        ifstream fs(path);
#endif
        char buffer[100];
        fs.getline(buffer, 100);
        MatIterator_<double> it;
        for (it = left.begin(); it != left.end(); it++)
        {
            fs >> *it;
        }
        Mat left44 = Mat::zeros(4, 4, CV_64F);
        left.copyTo(Mat(left44, Range(0, 3), Range(0, 3)));
        intri[0] = left;
        left44.at<double>(3, 3) = 1;

        fs.getline(buffer, 100);
        fs.getline(buffer, 100);
        for (it = right.begin(); it != right.end(); it++)
        {
            fs >> *it;
        }
        Mat right44 = Mat::zeros(4, 4, CV_64F);
        right.copyTo(Mat(right44, Range(0, 3), Range(0, 3)));
        intri[1] = right;
        right44.at<double>(3, 3) = 1;

        fs.getline(buffer, 100);
        fs.getline(buffer, 100);
        for (it = R.begin(); it != R.end(); it++)
        {
            fs >> *it;
        }
        fs.getline(buffer, 100);
        fs.getline(buffer, 100);
        for (it = t.begin(); it != t.end(); it++)
        {
            fs >> *it;
        }
        Mat R2L_Mat = Mat::zeros(4, 4, CV_64F);
        R.copyTo(Mat(R2L_Mat, Range(0, 3), Range(0, 3)));
        t.copyTo(Mat(R2L_Mat, Range(0, 3), Range(3, 4)));
        intri[2] = R;
        intri[3] = t;
        R2L_Mat.at<double>(3, 3) = 1;
        L2R = right44*R2L_Mat*left44.inv();
    }


	bool readStringList(const string& filename, vector<string>& l)
	{
		l.resize(0);
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return false;
		FileNode n = fs.getFirstTopLevelNode();
		if (n.type() != FileNode::SEQ)
			return false;
		FileNodeIterator it = n.begin(), it_end = n.end();
		for (; it != it_end; ++it)
			l.push_back((string)*it);
		return true;
	}

    void loadIntri2(string intriPath, Mat intri[2])
    {
        FileStorage fs(intriPath, FileStorage::READ);
        fs["M1"] >> intri[0];
        fs["D1"] >> intri[1];

        //cout << "win" << intri << endl;
        for (int i = 0; i < 2; i++)
        {
            intri[i].convertTo(intri[i], CV_32F);
        }
    }

	void loadIntri(string intriPath, Mat intri[6])
	{
		FileStorage fs(intriPath, FileStorage::READ);
		fs["M1"] >> intri[0];
		fs["M2"] >> intri[1];
		fs["R"] >> intri[2];
		fs["T"] >> intri[3];
		fs["D1"] >> intri[4];
		fs["D2"] >> intri[5];
		for (int i = 0; i < 6; i++)
		{
			intri[i].convertTo(intri[i], CV_32F);
		}
	}
    void loadIntriD(string intriPath, Mat intri[6])
    {
        FileStorage fs(intriPath, FileStorage::READ);
        fs["M1"] >> intri[0];
        fs["M2"] >> intri[1];
        fs["R"] >> intri[2];
        fs["T"] >> intri[3];
        fs["D1"] >> intri[4];
        fs["D2"] >> intri[5];
    }
	void checkCornerList(vector<Point2f>& corners)
	{
		if (corners.empty())
			return;
		Point2f& p0 = corners.front();
		Point2f& p1 = corners.back();
		if (p0.x > p1.x)
		{
			vector<Point2f> c;
			while (!corners.empty())
			{
				c.push_back(corners.back());
				corners.pop_back();
			}
			corners = c;
		}
	}

}
