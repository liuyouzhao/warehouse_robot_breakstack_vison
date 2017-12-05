#include "cam2Arm.h"
#include <iostream>
#include "Rotation.h"
using namespace std;
using namespace cv;

inline Vec3d vecProduct(Vec3d a, Vec3d b)
{
    return Vec3d(a(1)*b(2) - a(2)*b(1),
        a(2)*b(0) - a(0)*b(2),
        a(0)*b(1) - a(1)*b(0));
}

/*
z, 平面法向量
xy， 抓取面一边的方向
p，  抓取面中心
t,  抓取初始点
q，  抓取初始点旋转，四元数表示
pNew， 输出抓取中心
angles， 输出抓取角度
*/
void camera2Arm(Vec3d z, Vec3d xy, Vec3d p, Vec3d t, Vec4d q, Vec3d& pNew, Vec3d& angles)
{
    Mat_<double> R = quaternion2RotationMatrix(q);
    if (z[2] > 0)
        z = -z;
    Vec3d pZ = z / norm(z);
    Vec3d pX, pY;
    xy = xy / norm(xy);

    Vec3d x(R(0, 0), R(0, 1), R(0, 2));
    Vec3d y(R(1, 0), R(1, 1), R(1, 2));
    double xyx = (xy.t()*x)(0);
    double xyy = (xy.t()*y)(0);
    if (abs(xyx) > abs(xyy))
    {
        if (xyx < 0)
            xy = -xy;
        pX = xy;
        pY = vecProduct(pZ, pX);
    }
    else
    {
        if (xyy < 0)
            xy = -xy;
        pY = xy;
        pX = vecProduct(pY, pZ);
    }
    Mat Rp = (Mat_<double>(3, 3) <<
        pX[0], pY[0], pZ[0],
        pX[1], pY[1], pZ[1],
        pX[2], pY[2], pZ[2]
        );     
    Mat_<double> newPoint = R.inv()*Mat(p - t);
    pNew = Vec3d(newPoint(0), newPoint(1), newPoint(2));

    Mat_<double> cam2Arm = Rp.inv()*R;    
    angles = -rotationMatrixToEulerAngles(cam2Arm);
}

Vec4d getPose(Vec3d z, Vec3d xy)
{
    if (z[2] > 0)
        z = -z;
    Vec3d pZ = z / norm(z);
    Vec3d pX, pY;
    xy = xy / norm(xy);
    pX = xy;
    pY = vecProduct(pZ, pX);

    Mat_<double> Rp = (Mat_<double>(3, 3) <<
        pX[0], pY[0], pZ[0],
        pX[1], pY[1], pZ[1],
        pX[2], pY[2], pZ[2]
        );
    return eulerAngles2quaternion(rotationMatrixToEulerAngles(Rp));
}

Mat_<double> getPoseM(Vec3d z, Vec3d xy)
{
    if (z[2] > 0)
        z = -z;
    Vec3d pZ = z / norm(z);
    Vec3d pX, pY;
    xy = xy / norm(xy);
    pX = xy;
    pY = vecProduct(pZ, pX);

    return (Mat_<double>(3, 3) <<
        pX[0], pY[0], pZ[0],
        pX[1], pY[1], pZ[1],
        pX[2], pY[2], pZ[2]
        );
}