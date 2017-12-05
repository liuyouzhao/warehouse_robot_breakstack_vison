#pragma once

template<class T>
class SRM0
{
public:
	SRM0(double Q, unsigned int width, unsigned int height, T *in, int smallRegion = 50, int max = 256, double threshold=5);
	~SRM0();
	void run();
	void segmentation();
	void mergeSmallSegions();
    inline void addNeighborPair(int neighborIndex, T* pixel, int i1, int i2);
    bool predicate(int i1, int i2);
    inline int getRegionIndex(int i);
    inline void compress(int root, int i);
    inline int mergeRegions(int i1, int i2);
    void aveImage(T *out);    
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
	
	double logdelta;	
	double g;
	double Q;
    double factor;
    double threshold;
    int* nextNeighbor;
    int* neighborBucket;       
};