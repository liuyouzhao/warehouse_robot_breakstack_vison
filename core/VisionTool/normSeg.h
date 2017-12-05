#pragma once
#include <math.h>

class NormSeg
{
public:
	NormSeg(double Q, unsigned int width, unsigned int height, short *in, float* normal, int smallRegion = 50, int max = 256, double threshold=5);
	~NormSeg();
	void run();
	void segmentation();
	void mergeSmallSegions();
    inline void addNeighborPair(int neighborIndex, short* pixel, int i1, int i2);
    bool predicate(int i1, int i2);
    inline bool normalign(int i1, int i2);
    inline int getRegionIndex(int i);
    inline void compress(int root, int i);
    inline int mergeRegions(int i1, int i2);
    void aveImage(short *out); 
    void aveImage(float *out);
    void regions(int* r, float* ave);
    int regionsCount;
    float* average;
    int* regionIndex;
    int* area; 
    
private:
	unsigned int width;
	unsigned int height;
	unsigned int size;	
    const unsigned short maxValue;
    const unsigned int smallregion;
	short *in;
	short *out;   
    float* normal;
    struct vec3f
    {
        float x, y, z;
        inline vec3f& operator +=(const vec3f& vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            return *this;
        }
        inline float operator *(const vec3f& vec)
        {           
            return x*vec.x+y*vec.y+z*vec.z;
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
    };

    struct Node
    {
        int idx;
        Node* p, *pp;
        Node() :p(0){}
        void init(int _idx){ idx = _idx; }
        void link(Node* n){ 
            Node* l = this;
            while (l->p != 0){ l = l->p; }
            l->p = n;
        }
    };

    vec3f* normVec;
	
	double logdelta;	
	double g;
	double Q;
    double factor;
    double threshold;
    int* nextNeighbor;
    int* neighborBucket;  
    
public:
    Node* rootNodes;
    Node* nodes;
    
};