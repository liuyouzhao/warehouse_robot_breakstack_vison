#include "DeviceTool.h"
#include <thread>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "DepthTool.h"
#include <iostream>
namespace tools_3d
{
	using namespace openni;
	using namespace std;
	using namespace cv;
	DeviceTool::DeviceTool():
		TIME_INTERNAL(100),
		devices(NULL),
		count(0),
		mode(STOP),
		width(640),
		height(480),
		path("d:\\temp\\obj"),
		imageNum(0)
	{

	}
	DeviceTool::~DeviceTool()
	{
		delete[] devices;
	}
	void DeviceTool::openAllDevices()
	{
		OpenNI::shutdown();
		Status rc = OpenNI::initialize();
		if (rc != STATUS_OK)
		{
			printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
			return;
		}

		Array<DeviceInfo> deviceInfoList;
		OpenNI::enumerateDevices(&deviceInfoList);
		count = deviceInfoList.getSize();
		devices = new Device[count];
		for (int i = 0; i < count; i++)
		{
			rc = devices[i].open(deviceInfoList[i].getUri());
			devices[i].setDepthColorSyncEnabled(true);
			devices[i].setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
			if (rc != STATUS_OK)
			{
				printf("Couldn't open device %d\n%s\n", i, OpenNI::getExtendedError());
			}
		}
	}
	void DeviceTool::run(int current)
	{

	}
	bool saveRaw(const char *rawName, const char *imgBuf, int width, int height, int pixBytes)
	{
		FILE *saveFile;
		size_t writeLen;

		if ((saveFile = fopen(rawName, "wb")) == NULL){
			printf("File could not be opened.\n");
			return 0;
		}
		else {
			writeLen = fwrite(imgBuf, width*pixBytes, height, saveFile);
		}

		fflush(saveFile);
		fclose(saveFile);
		return 1;
	}

	void DeviceTool::setResolution(openni::VideoStream& vStream)
	{
		VideoMode vMode = vStream.getVideoMode();
		vMode.setResolution(width, height);
		vStream.setVideoMode(vMode);
	}

	std::string DeviceTool::num2Str(int i)
	{
		char str[10];
		sprintf(str, "%d", i);
		return string(str);
	}

	void DeviceTool::show(openni::VideoFrameRef(*frameRef)[3])
	{
		switch (mode)
		{
		case DEPTH:
			for (int i = 0; i < count; i++)
			{
				Mat image = Mat(frameRef[i][0].getHeight(), frameRef[i][0].getWidth(), CV_16U, (void*)frameRef[i][0].getData());
				Mat show = toColor(image);
				imshow("depth " + num2Str(i), image);
			}
			break;
		case COLOR:
			for (int i = 0; i < count; i++)
			{
				if (!frameRef[i][1].isValid())
					continue;
				Mat image;
				cvtColor(Mat(frameRef[i][1].getHeight(), frameRef[i][1].getWidth(), CV_8UC3, (void*)frameRef[i][1].getData()), image, CV_RGB2BGR);
				imshow("color " + num2Str(i), image);
			}

			break;
		case DEPTH_COLOR:
			for (int i = 0; i < count; i++)
			{
				Mat image;
				cvtColor(Mat(frameRef[i][1].getHeight(), frameRef[i][1].getWidth(), CV_8UC3, (void*)frameRef[i][1].getData()), image, CV_RGB2BGR);
				Mat depth(frameRef[i][0].getHeight(), frameRef[i][0].getWidth(), CV_16U, (void*)frameRef[i][0].getData());
				imshow("depth " + num2Str(i), toGray(depth));
				imshow("color " + num2Str(i), image);
				//waitKey(100);
			}
			break;
		case IR:
			for (int i = 0; i < count; i++)
			{
				Mat image = Mat(frameRef[i][2].getHeight(), frameRef[i][2].getWidth(), CV_8U, (void*)frameRef[i][2].getData());
				imshow("ir " + num2Str(i), image);
			}
			break;
		}

	}


	void DeviceTool::save(openni::VideoFrameRef(*frameRef)[3])
	{
		switch (mode)
		{
		case DEPTH:
			for (int i = 0; i < count; i++)
			{
				Mat image = Mat(frameRef[i][0].getHeight(), frameRef[i][0].getWidth(), CV_16U, (void*)frameRef[i][0].getData());
				Mat show = toColor(image);
				imshow("depth " + num2Str(i), image);
			}
			break;
		case COLOR:
			for (int i = 0; i < count; i++)
			{
				if (!frameRef[i][1].isValid())
					continue;
				Mat image;
				cvtColor(Mat(frameRef[i][1].getHeight(), frameRef[i][1].getWidth(), CV_8UC3, (void*)frameRef[i][1].getData()), image, CV_RGB2BGR);
				imwrite(path+"\\color " + num2Str(i)+"_"+num2Str(imageNum)+".jpg", image);
			}
			break;
		case DEPTH_COLOR:
			for (int i = 0; i < count; i++)
			{
				Mat image;
				cvtColor(Mat(frameRef[i][1].getHeight(), frameRef[i][1].getWidth(), CV_8UC3, (void*)frameRef[i][1].getData()), image, CV_RGB2BGR);
				imwrite(path + "\\color " + num2Str(i) + "_" + num2Str(imageNum) + ".jpg", image);
				saveRaw((path + "\\depth " + num2Str(i) + "_" + num2Str(imageNum) + ".raw").c_str(),
					(const char *)frameRef[i][0].getData(), width, height, 2);
			}
			break;
		case IR:
			for (int i = 0; i < count; i++)
			{
				Mat image = Mat(frameRef[i][2].getHeight(), frameRef[i][2].getWidth(), CV_8U, (void*)frameRef[i][2].getData());
				imshow("ir " + num2Str(i), image);
			}
			break;
		}
		imageNum++;
	}

	void DeviceTool::setMode(DeviceTool::Mode i_mode, VideoStream(*streams)[3], int* sStart, int* sCount)
	{
		if (i_mode == mode)
			return;
		if (mode == IR || mode == STOP)
		{
			for (int i = 0; i < count; i++)
			{
				streams[i][2].stop();
				streams[i][0].start();
				streams[i][1].start();
			}
		}
		if (i_mode == IR)
		{
			for (int i = 0; i < count; i++)
			{
				streams[i][0].stop();
				streams[i][1].stop();
				streams[i][2].start();
			}
		}

		mode = i_mode;
		switch (mode)
		{
		case DEPTH:
			*sStart = 0;
			*sCount = 1;
			break;
		case COLOR:
			*sStart = 1;
			*sCount = 1;
			break;
		case DEPTH_COLOR:
			*sStart = 0;
			*sCount = 2;
			break;
		case IR:
			*sStart = 2;
			*sCount = 1;
			break;
		}



	}

	void DeviceTool::multiRun()
	{
		int keyHit = -1;
		//Device& device = devices[current];
		//device.setDepthColorSyncEnabled(true);

		VideoStream(*streams)[3] = new VideoStream[count][3];
		VideoStream*(*pstreams)[3] = new VideoStream* [count][3];
		int* changedIndex = new int[count];

		for (int i = 0; i < count; i++)
		{
			streams[i][0].create(devices[i], SENSOR_DEPTH);
			streams[i][1].create(devices[i], SENSOR_COLOR);
			streams[i][2].create(devices[i], SENSOR_IR);
		}
		for (int i = 0; i < count; i++)
			for (int j = 0; j < 3; j++)
			{
				setResolution(streams[i][j]);
				pstreams[i][j] = &streams[i][j];
			}
		VideoFrameRef(*frameRef)[3] = new VideoFrameRef[count][3];
		bool isReady = false;
		int streamStart, streamCount;
		setMode(DEPTH_COLOR, streams, &streamStart, &streamCount);
		while (27 != keyHit)
		{
			//#pragma omp parallel for
			for (int i = 0; i < count; i++)
			{
				OpenNI::waitForAnyStream(pstreams[i] + streamStart, streamCount, changedIndex + i);
				//cout << "camera "<<i<<" channel " << changedIndex[i]<<" is ready"<<endl;
				streams[i][streamStart+changedIndex[i]].readFrame(frameRef[i] + streamStart + changedIndex[i]);
			}
			if (mode == DEPTH_COLOR)
			{
				for (int i = 0; i < count; i++)
				{
					if (frameRef[i][0].isValid() && frameRef[i][1].isValid() &&
						abs(int64_t(frameRef[i][0].getTimestamp() - frameRef[i][1].getTimestamp())) >> 10 < TIME_INTERNAL)
						isReady = true;
					else
					{
						isReady = false;
						break;
					}
				}
				if (isReady)
					keyHit = waitKey(30);
			}
			else
			{
				keyHit = waitKey(30);
				isReady = true;
			}
			if (isReady)
				show(frameRef);

			switch (keyHit)
			{
			case 's':
				save(frameRef);
				break;
			case '0':
				setMode(DEPTH_COLOR, streams, &streamStart, &streamCount);
				break;
			case '1':
				setMode(DEPTH, streams, &streamStart, &streamCount);
				break;
			case '2':
				setMode(COLOR, streams, &streamStart, &streamCount);
				break;
			case '3':
				setMode(IR, streams, &streamStart, &streamCount);
				break;
			default:
				break;
			}
			keyHit = waitKey(30);
			//if (framePair.ready())
			//{
			//	cv::Mat gray = tools_3d::toGray(cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_16U, (void*)depthFrame.getData()));
			//	//cv::imshow("depth", gray);
			//	cv::Mat colorMat(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorFrame.getData());
			//	cv::Mat show(height, width * 2, CV_8UC3);
			//	colorMat.copyTo(Mat(show, Rect(0, 0, width, height)));
			//	cvtColor(gray, Mat(show, Rect(width, 0, width, height)), CV_GRAY2BGR);
			//	cv::imshow("show", show);
			//	cout << "             \r" << ((DepthPixel*)depthFrame.getData())[height / 2 * width + width / 2];
			//
			//	if (saving)
			//	{
			//		framePair.save(path, name, count++);
			//		framePair.clear();
			//	}
			//}
		}



		//rc = OpenNI::waitForAnyStream(&(streams[0]), 2, &changedIndex, SAMPLE_READ_WAIT_TIMEOUT);
		delete[] streams;
		delete[] pstreams;
	}
}
