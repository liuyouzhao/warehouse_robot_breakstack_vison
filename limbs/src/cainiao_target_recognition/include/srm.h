#pragma once

template<class T>
class SRM
{
public:
	SRM(double Q, unsigned int width, unsigned int height, T *in, float* normal, int smallRegion = 50, int max = 256, double threshold=5);
	~SRM();
	void run();
	void segmentation();
	void mergeSmallSegions();
    inline void addNeighborPair(int neighborIndex, T* pixel, int i1, int i2);
    bool predicate(int i1, int i2);
    inline bool normalign(int i1, int i2);
    inline int getRegionIndex(int i);
    inline void compress(int root, int i);
    inline int mergeRegions(int i1, int i2);
    void aveImage(T *out); 
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
	T *in;
	T *out;   
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
    };
    vec3f* normVec;
	
	double logdelta;	
	double g;
	double Q;
    double factor;
    double threshold;
    int* nextNeighbor;
    int* neighborBucket;  
    
};