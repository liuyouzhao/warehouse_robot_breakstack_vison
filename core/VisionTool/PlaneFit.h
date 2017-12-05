#pragma once
#include <math.h>
#include "opencv2/core/core.hpp"

class PlaneFit
{
public:
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
        inline vec3f operator *(const int a)
        {
            return vec3f(x*a, y*a, z*a);
        }
        inline vec3f operator /(const int a)
        {
            return vec3f(x/a, y/a, z/a);
        }
        inline float len()
        {
            return sqrt(x*x + y*y + z*z);
        }
        inline void norm()
        {
            float length = len();
            x /= length;
            y /= length;
            z /= length;
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
        int root;
        inline float len()
        {
            squareroot = sqrt(x*x + y*y + z*z);
            return sqrt(x*x + y*y + z*z);
        }
        inline float disToOrigin()
        {
            dis = 1. / len();
            return dis;
        }
    };



	PlaneFit(unsigned int width, unsigned int height, float *in, float* normal, float* grad, int smallRegion = 50, int gradMax = 20);
	~PlaneFit();
	void run();
	void segmentation();
	void mergeSmallSegions();
    inline void addNeighborPair(int neighborIndex, int i1, int i2);
    inline void addNeighborPair2(int neighborIndex, int i1, int i2);
    bool predicate(int i1, int i2);
    inline bool normalign(int i1, int i2);
    inline int getRegionIndex(int i);
    inline void compress(int root, int i);
    inline int mergeRegions(int i1, int i2);
    inline int orderMerge(int i1, int i2);
    void aveImage(short *out);
    void aveImage(float *out);
    void regions(int* r, float* ave);
    void mergePlanes();
    int regionsCount;
    float* average;
    vec3f* mean;
    int* regionIndex;
    int* area;
    void sortPlanes();
    void sortPlanes2();
    void findOutline();
    plane calculatePlane(int root);
    void showRegion(int i1, int i2);

private:
	int width;
	int height;
	unsigned int size;

    const unsigned int smallregion;


    int binCount;
    vec3f* in, *normal;
    float* grad;
    int gradMax;

    struct Node
    {
        int idx;
        bool small, good, root;
        int parent, child;
        int area, planeIdx;

        Node* p, *pp;
        Node() :p(0), area(1), good(true), root(true){}
        void init(int _idx){ parent = child = idx = _idx;}
        void link(Node* n){
            Node* l = this;
            while (l->p != 0){ l = l->p; }
            l->p = n;
        }
    };
    void link(int l1, int l2);
    vec3f* normVec;

    int* nextNeighbor;
    int* neighborBucket;
    int *listp, *listc;
    std::vector<plane> planes;
    unsigned char* mask;

public:
    Node* rootNodes;
    Node* nodes;
    Node* chain;
    Node badNode;

};
