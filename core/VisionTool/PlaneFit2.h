#pragma once
#include <math.h>
#include "opencv2/core/core.hpp"
#include <list>
class PlaneFit2
{
public:
    float planeDistance;
    void filterByRect(cv::RotatedRect rr);
    
    struct vec3f
    {
        float x, y, z;
        vec3f(){}
        vec3f(float a, float b, float c):x(a), y(b), z(c){}
        inline vec3f& operator +=(const vec3f& vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            return *this;
        }
        inline float operator *(const vec3f& vec)
        {
            return x*vec.x + y*vec.y + z*vec.z;
        }
        inline vec3f operator +(const vec3f& vec)
        {
            return vec3f(x+vec.x,  y+vec.y, z+vec.z);
        }
        inline vec3f operator -(const vec3f& vec)
        {
            return vec3f(x - vec.x, y - vec.y, z - vec.z);
        }
        inline vec3f operator *(const float a)
        {
            return vec3f(x*a, y*a, z*a);
        }
        inline vec3f operator /(const float a)
        {
            return vec3f(x/a, y/a, z/a);
        }
        inline float len()
        {
            return sqrt(x*x + y*y + z*z);
        }
        inline vec3f& norm()
        {
            float length = len();
            x /= length;
            y /= length;
            z /= length;
            return *this;
        }
        inline float align(vec3f& vec)
        {
            float len1 = len();
            float len2 = vec.len();
           /* if (len1 < 0.5 || len2 < 0.5)
                return 0.;
            else*/
               return (*this)*vec/len1/len2;
        }
    };
    struct plane
    {
        float x, y, z, dis, squareroot;
        std::vector<int> pts;
        std::vector<vec3f> points;
        std::vector<float> vertex;
        int root;
        inline float len()
        {
            return sqrt(x*x + y*y + z*z);
        }
        inline float disToOrigin()
        {
            dis = 1000 / squareroot;
            return dis;
        }
        inline void set(float v[3])
        {
            x = v[0]; y = v[1]; z = v[2];
            squareroot = len();
        }
    };
    void sortPlanes3();
    void sortPlanes4();
    cv::Mat camera;
    void solvePtOnPlane(cv::Point p, plane planex, cv::Point3f& out);
    float thetaMaxLength;
    float calRotate(cv::Mat curve);
    void findRightAngle(cv::Mat curve);
    bool damnRight;
    cv::Size boxSize;
    cv::Size boxPixelSize;
    float fxy;
    PlaneFit2(cv::Mat& depth, cv::Mat& cloud, std::vector<cv::Mat>& wall, int smallRegion = 50, int gradMax = 50);
    PlaneFit2(cv::Mat& color, cv::Mat& cloud, cv::Size box, cv::Mat camera, int smallRegion = 50, int gradMax = 50);
    void reCalculate(std::vector<cv::Point> vertices);
	~PlaneFit2();
    bool found;
	std::string run();
    void run(std::vector<float>& position, cv::Mat_<double>& oritation, std::vector<float>& vertexs);
    cv::Mat_<double> pickR;
    cv::Mat _binary;

	std::string run(cv::Mat& result);
	void segmentation();
	void mergeSmallSegions();
    inline void addNeighborPair(int neighborIndex, int i1, int i2);
    inline void addNeighborPair2(int neighborIndex, int i1, int i2);
    bool predicate(int i1, int i2);
    inline bool normalign(int i1, int i2);
    inline int getRegionIndex(int i);
    inline void compress(int root, int i);
    inline int mergeRegions(int i1, int i2);
    inline void orderMerge(int i1, int i2);
    void aveImage(short *out);
    void aveImage(float *out);
    void regions(int* r, float* ave);
    void mergePlanes();
    void initSeeds();
    int regionsCount;
    float* average;
    vec3f* pMean, *pSum;
    int* regionIndex;
    void project(plane& p);
    int planeId;
    inline bool planeJudge1(int i1, int i2, int disTor);
    inline bool planeJudge2(int i1, int i2, int disTor);
    //void sortPlanes();
    void sortPlanes2();
    void findOutline();
    plane calculatePlane(int root);
    void showRegion(int i1, int i2);
    cv::Mat_<short> depth;
    cv::Mat_<cv::Vec3f> cloud;
    cv::Mat color;
    cv::Mat_<float> gradMag;
    cv::Mat maskMat;
    int total,rows,cols;
    void showAll(int area = 1);
    std::vector<cv::Mat> wall;
    std::string resultStr;
    std::vector<float> pickPoint, pickVertex;
    std::vector<cv::Point> verticesInColor;
private:
	int width;
	int height;
	unsigned int size;

    const unsigned int smallregion;
    void expandPlane(int i1, int i2);

    int binCount;
    vec3f* pIn, *pNormal;
    float* pGrad;
    unsigned char* pMask;
    int gradMax;

    struct Node
    {
        bool good;
        int parent, child;
        int area, planeIdx;
        Node() :area(1){}
        void init(int _idx){ parent = child = _idx;}

    };
    void link(int l1, int l2);
    vec3f* normVec;

    int* nextNeighbor;
    int* neighborBucket;
    int *listp, *listc;
    std::vector<plane> planes;
    std::vector<int> objs;
    unsigned char* mask;
    int tooSmall;
    void removeRoot(int idx);
    void mergeByPlane();
public:
    Node* rootNodes;
    Node* nodes;
    Node* chain;
    Node badNode;
    int maxdistance;
    struct SortPair{
        int idx;
        union{
            int i;
            float f;
        } value;
        SortPair(int i, int j) :idx(i){ value.i = j; }
        SortPair(int i, float j) :idx(i){ value.f = j; }
    };

};
