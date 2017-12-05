#pragma once
#include <OpenNI.h>
#include <vector>

namespace tools_3d
{
	class DeviceTool
	{
	public:
		DeviceTool();
		~DeviceTool();
		void openAllDevices();		
		enum Mode{STOP, DEPTH, IR, COLOR, DEPTH_COLOR};
		void multiRun();
		
		int count;
		openni::Device* devices;
		int width, height;
	private:
		void run(int i);
		DeviceTool::Mode mode;
		inline void setResolution(openni::VideoStream& vStream);
		const int TIME_INTERNAL;
		void setMode(DeviceTool::Mode mode, openni::VideoStream(*streams)[3], int* sStart, int* sCount);
		void show(openni::VideoFrameRef(*frameRef)[3]);
		void save(openni::VideoFrameRef(*frameRef)[3]);
		inline std::string num2Str(int i);
		std::string path;
		int imageNum;
		
	};

	void openAllDevices(openni::Device** devices, int* count);
}