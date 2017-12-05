#pragma once

#include "opencv2/core/core.hpp"
#ifdef CAM2ARM_EXPORTS
#define CAM2ARM_API //__declspec(dllexport)
#else
#define CAM2ARM_API //__declspec(dllimport)
#endif

CAM2ARM_API cv::Mat quaternion2RotationMatrix(cv::Vec4d q);
CAM2ARM_API cv::Vec3d rotationMatrixToEulerAngles(cv::Mat_<double> &R);
CAM2ARM_API cv::Vec4d eulerAngles2quaternion(cv::Vec3d a);
CAM2ARM_API cv::Vec3d quaternion2EulerAngles(cv::Vec4d q);
