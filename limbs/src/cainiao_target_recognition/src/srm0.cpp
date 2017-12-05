#include "srm0.h"
#include <math.h>
#ifdef UNIX
#include <string.h>
#include <stdio.h>
#else
#include <string>
#endif

template<class T>
SRM0<T>::SRM0(double _Q, unsigned int _width, unsigned int _height, T *_in, int _smallRegion, int _max, double _threshold):
width(_width), height(_height), Q(_Q), size(_width*_height), in(_in), regionsCount(_width*_height),
smallregion(_smallRegion), maxValue(_max), threshold(_threshold),
nextNeighbor(NULL), neighborBucket(NULL), average(NULL), area(NULL), regionIndex(NULL)
{
}

template<class T>
SRM0<T>::~SRM0()
{
    delete[] nextNeighbor;
    delete[] neighborBucket;
    delete[] average;
    delete[] area;
    delete[] regionIndex;
}

template<class T>
void SRM0<T>::run()
{
    //from paper: Statistical Region Merging, Richard Nock and Frank Nielsen
    logdelta = 2.0 * log(6.0 * size);
    g = 256.0;
    factor = g * g / 2 / Q;

    //init buffer
    neighborBucket = new int[maxValue];
    for (int i = 0; i < maxValue; i++)
    {
        neighborBucket[i] = -1;
    }
    nextNeighbor = new int[2 * width*height];
    memset(nextNeighbor, 0, sizeof(int) * 2 * width*height);

    average = new float[size];
    area = new int[size];
    regionIndex = new int[size];
    for (int i = 0; i < size; i++)
    {
        average[i] = in[i];
        area[i] = 1;
        regionIndex[i] = i;
    }
    segmentation();
    //mergeSmallSegions();
    int r0 = getRegionIndex(0);
    int r4 = getRegionIndex(4);

}
template<class T>
void SRM0<T>::aveImage(T *out)
{
    for (int i = 0; i < size; i++)
    {
        out[i] = average[getRegionIndex(i)];
    }
}
template<class T>
void SRM0<T>::regions(int* r, float* ave)
{
    for (int i = 0; i < size; i++)
    {
        regionIndex[i] = getRegionIndex(i);
    }
    int* index = new int[regionsCount];
    for (int i = 0; i < regionsCount; i++)
    {
        index[i] = -1;
    }
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < regionsCount; j++)
        {
            if (index[j] == -1)
            {
                index[j] = regionIndex[i];
                ave[j] = average[regionIndex[i]];
                r[i] = j;
                break;
            }
            if (index[j] == regionIndex[i])
            {
                r[i] = j;
                break;
            }
        }
    }
    delete[] index;
}

template <class T>
void SRM0<T>::addNeighborPair(int neighborIndex, T* pixel, int i1, int i2)
{
    int difference = pixel[i1]>pixel[i2] ? pixel[i1] - pixel[i2] : pixel[i2] - pixel[i1];
    nextNeighbor[neighborIndex] = neighborBucket[difference];
    neighborBucket[difference] = neighborIndex;
}

template <class T>
int SRM0<T>::getRegionIndex(int i)
{
    int leaf = i;
    int root = regionIndex[i];
    while (root != leaf)
    {
        leaf = root;
        root = regionIndex[leaf];
    }
    return root;
}

template <class T>
inline void SRM0<T>::compress(int root, int i)
{
    int leaf = regionIndex[i];
    while (leaf != root)
    {
        regionIndex[i] = root;
        i = leaf;
        leaf = regionIndex[i];
    }
}

template <class T>
bool SRM0<T>::predicate(int i1, int i2)
{
    float d2 = average[i1] - average[i2];
    d2 *= d2;
    if (area[i1] > 10)
        printf("");
    float log1 = log(1.f + area[i1])
        * (g < area[i1] ? g : area[i1]);
    float log2 = log(1.f + area[i2])
        * (g < area[i2] ? g : area[i2]);
    float ccc = factor * ((log1 + logdelta) / area[i1]
        + ((log2 + logdelta) / area[i2]));
    bool result = d2 < ccc;
    if (!result)
        printf("");
    return result;
}

template <class T>
int SRM0<T>::mergeRegions(int i1, int i2)
{
    regionsCount--;
    int mergedCount = area[i1] + area[i2];
    float mergedAverage = (average[i1] * area[i1]
        + average[i2] * area[i2]) / mergedCount;

    // merge larger index into smaller index
    if (i1 > i2)
    {
        average[i2] = mergedAverage;
        area[i2] = mergedCount;
        regionIndex[i1] = i2;
        return i2;
    }
    else
    {
        average[i1] = mergedAverage;
        area[i1] = mergedCount;
        regionIndex[i2] = i1;
        return i1;
    }
}

template<class T>
void SRM0<T>::segmentation()
{
    // Consider C4-connectivity here
    unsigned int index, rindex, neighborIndex;
    for (int j = height - 1; j >= 0; j--)
    {
        rindex = j*width;
        for (int i = width - 1; i >= 0; i--)
        {
            index = i + rindex;
            int neighborIndex = index << 1;

            // vertical
            if (j < height - 1)
                addNeighborPair(neighborIndex + 1, in, index, index + width);

            // horizontal
            if (i < width - 1)
                addNeighborPair(neighborIndex, in, index, index + 1);
        }
    }
    for (int i = 0; i < 4; i++)
    {
        int neighborIndex = neighborBucket[i];
        while (neighborIndex >= 0)
        {
            int i1 = neighborIndex >> 1;
            int i2 = i1 + (0 == (neighborIndex & 1) ? 1 : width);

            int r1 = getRegionIndex(i1);
            int r2 = getRegionIndex(i2);

            if (r1 != r2 /*&& predicate(r1, r2)*/)
            {
                int root = mergeRegions(r1, r2);
                compress(root, i1);
                compress(root, i2);
            }
            neighborIndex = nextNeighbor[neighborIndex];
        }
    }
}
template<class T>
void SRM0<T>::mergeSmallSegions()
{
    unsigned int reg1, reg2;
    unsigned int index, rindex;
    for (int i = 0; i < height; i++)
    {
        rindex = i*width;
        for (int j = 1; j < width; j++)
        {
	        index = rindex + j;
            reg1 = getRegionIndex(index);
            reg2 = getRegionIndex(index - 1);
            if (reg1 != reg2 && (area[reg1] < smallregion || area[reg2] < smallregion))
            {
                int root = mergeRegions(reg1, reg2);
                compress(root, index);
                compress(root, index - 1);
            }
        }
    }
}

template class SRM0<unsigned char>;
template class SRM0<unsigned short int>;
template class SRM0<short int>;


