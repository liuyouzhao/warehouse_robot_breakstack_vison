#pragma once

#ifdef CAM2ARM_EXPORTS
#define CAM2ARM_API __declspec(dllexport)
#else
#define CAM2ARM_API
#endif

#include "opencv2/core/core.hpp"
/*
z, ƽ�淨����
xy�� ץȡ��һ�ߵķ���
p��  ץȡ������
t,  ץȡ��ʼ��
q��  ץȡ��ʼ����ת����Ԫ���ʾ
pNew�� ���ץȡ����
angles�� ���ץȡ�Ƕ�
*/
CAM2ARM_API void camera2Arm(cv::Vec3d z, cv::Vec3d xy, cv::Vec3d p, cv::Vec3d t,
    cv::Vec4d q, cv::Vec3d& pNew, cv::Vec3d& angles);

CAM2ARM_API cv::Vec4d getPose(cv::Vec3d z, cv::Vec3d xy);
CAM2ARM_API cv::Mat_<double> getPoseM(cv::Vec3d z, cv::Vec3d xy);
