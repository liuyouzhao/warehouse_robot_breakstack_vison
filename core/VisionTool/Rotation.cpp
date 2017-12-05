#include "Rotation.h"


using namespace std;
using namespace cv;

Mat quaternion2RotationMatrix(Vec4d q)
{
    double& w = q[0];
    double& x = q[1];
    double& y = q[2];
    double& z = q[3];
    return (Mat_<double>(3, 3) <<
        1 - 2 * y*y - 2 * z*z, 2 * x*y - 2 * z*w, 2 * x*z + 2 * y*w,
        2 * x*y + 2 * z*w, 1 - 2 * x*x - 2 * z*z, 2 * y*z - 2 * x*w,
        2 * x*z - 2 * y*w, 2 * y*z + 2 * x*w, 1 - 2 * x*x - 2 * y*y
        );
}

#include <stdio.h>
Vec3d rotationMatrixToEulerAngles(Mat_<double> &R)
{
    // Checks if a matrix is a valid rotation matrix.
#ifdef UNIX
    Mat shouldBeIdentity = R.t() * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
    double normv = norm(I, shouldBeIdentity);
    bool ass = normv * 1000 - (1e-6 * 1000);
    printf("normv %f %d \n", normv, ass);
    assert(ass);
#else
    assert([](Mat R)->bool{
        Mat shouldBeIdentity = R.t() * R;
        Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
        return  norm(I, shouldBeIdentity) < 1e-4;
    }(R));
#endif

    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return Vec3d(x, y, z);
}

Vec4d eulerAngles2quaternion(Vec3d a)
{
    Vec4d q;
    double f = a[0] / 2; //fai
    double t = a[1] / 2; //thit
    double p = a[2] / 2; //psai
    q[0] = cos(f)*cos(t)*cos(p) + sin(f)*sin(t)*sin(p);
    q[1] = sin(f)*cos(t)*cos(p) - cos(f)*sin(t)*sin(p);
    q[2] = cos(f)*sin(t)*cos(p) + sin(f)*cos(t)*sin(p);
    q[3] = cos(f)*cos(t)*sin(p) - sin(f)*sin(t)*cos(p);
    return q;
}

Vec3d quaternion2EulerAngles(Vec4d q)
{
    Vec3d a;
    double& w = q[0];
    double& x = q[1];
    double& y = q[2];
    double& z = q[3];

    a[0] = atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y));
    a[1] = asin(2 * (w*y - z*x));
    a[2] = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
    return a;
}
