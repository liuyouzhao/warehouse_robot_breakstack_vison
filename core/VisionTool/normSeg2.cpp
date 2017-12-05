#include "normSeg2.h"

#include <string>
#include <algorithm>
#include "opencv2/core/core.hpp"

static int mergeTime = 0;
const float bin[] = {0.9962,    0.9848,    0.9659,    0.9397 };
NormSeg2::NormSeg2(unsigned int _width, unsigned int _height, float *_in, float* _normal, float* _grad, int _smallRegion, int _gradMax) :
width(_width), height(_height), size(_width*_height), in((vec3f*)_in), regionsCount(_width*_height), smallregion(_smallRegion),
nextNeighbor(NULL), neighborBucket(NULL), average(NULL), area(NULL), regionIndex(NULL), normal((vec3f*)_normal), grad(_grad), gradMax(_gradMax)
{
    binCount = sizeof(bin) / sizeof(bin[0]);
}


NormSeg2::~NormSeg2()
{
    delete[] nextNeighbor;
    delete[] neighborBucket;
    delete[] average;
    delete[] area;
    delete[] regionIndex;
    delete[] nodes;
    delete[] rootNodes;
}

void NormSeg2::sortPlanes()
{
    int count = listc[0];
    while (count<size-1)
    {

        int root = count;
        planes.push_back(plane());
        plane& p = planes.back();
        p.root = root;
        chain[root].planeIdx = planes.size() - 1;
        while (chain[root].child != root)
        {
            mask[root] = 1;
            p.pts.push_back(root);
            root = chain[root].child;
        }
        cv::Mat mat(p.pts.size(), 3, CV_32F);
        vec3f* pmat = (vec3f*)mat.data;
        for (int i = 0; i < p.pts.size(); i++)
        {
            pmat[i] = in[p.pts[i]];
        }
        cv::Mat  planeMat = mat.inv(cv::DECOMP_SVD)*cv::Mat::ones(p.pts.size(), 1, CV_32F);
        vec3f* pv = (vec3f*)planeMat.data;
        p.x = pv->x; p.y = pv->y; p.z = pv->z;
        p.disToOrigin();
        count = listc[count];

    }
}
void NormSeg2::mergePlanes()
{
    //cv::Mat maskShow = cv::Mat(height, width, CV_8U, mask);
    //maskShow = maskShow * 255;

    unsigned int index, rindex, neighborIndex, indexv, indexh;
    for (int j = height - 1; j >= 0; j--)
    {
        rindex = j*width;
        for (int i = width - 1; i >= 0; i--)
        {
            index = i + rindex;
            indexv = index + width;
            indexh = index + 1;
            if (mask[index] == 0)
            {
                continue;
            }
            int root1 = getRegionIndex(index);

            if (mask[indexv] > 0 && j < height - 1)
            {
                int root = getRegionIndex(indexv);
                if (root != root1 && abs(planes[chain[root].planeIdx].dis - planes[chain[root1].planeIdx].dis) < 8)
                {
                    int r = mergeRegions(root, root1);
                    compress(r, index);
                    compress(r, indexv);
                }
            }
            root1 = getRegionIndex(index);
            if (mask[indexh] > 0 && i < width - 1)
            {
                int root = getRegionIndex(indexh);
                if (root != root1 && abs(planes[chain[root].planeIdx].dis - planes[chain[root1].planeIdx].dis) < 8)
                {
                    int r = mergeRegions(root, root1);
                    compress(r, index);
                    compress(r, indexh);
                }
            }

        }
    }
    struct PlaneDepth { int idx; float depth; } ;
    std::vector<PlaneDepth> rootList;
    int count = listc[0];
    while (count<size-1)
    {
        int root = count;
        if (chain[root].area < smallregion*4)
        {
            count = listc[count];
            continue;
        }
        PlaneDepth pd;
        pd.idx = root;
        pd.depth = mean[root].z;
        rootList.push_back(pd);
        count = listc[count];
    }
    sort(rootList.begin(), rootList.end(), [](const PlaneDepth& a, const PlaneDepth& b){return a.depth < b.depth; });
    cv::Mat_<cv::Vec3f> normMat(height, width, (cv::Vec3f*)normal);
    cv::Mat show = cv::Mat::zeros(height, width, CV_8U);
    for (int i = 0; i < rootList.size(); i++)
    {
        int root = rootList[i].idx;
        while (chain[root].child != root)
        {
            show.data[root] = 255;
            root = chain[root].child;
        }
        if (rootList[i].depth > 1090)
            continue;
    }
}
void NormSeg2::run()
{
    //init buffer
    neighborBucket = new int[binCount];
    for (int i = 0; i < binCount; i++)
    {
        neighborBucket[i] = -1;
    }
    nextNeighbor = new int[2 * width * height];
    memset(nextNeighbor, 0, sizeof(int) * 2 * width * height);

    area = new int[size];
    regionIndex = new int[size];
    nodes = new Node[size];
    rootNodes = new Node[size];
    chain = new Node[size];
    listp = new int[size];
    listc = new int[size];
    mean = new vec3f[size];
    mask = new uchar[size];
    memset(mask, 0, size);
    for (int i = 0; i < size; i++)
    {
        area[i] = 1;
        regionIndex[i] = i;
        nodes[i].init(i);
        rootNodes[i].init(i);
        chain[i].init(i);
        rootNodes[i].p = rootNodes + i + 1;
        if (i < size - 1)
            rootNodes[i + 1].pp = rootNodes + i;

        listp[i] = i - 1;
        listc[i] = i + 1;
        mean[i] = in[i];
    }
    segmentation();
    if (chain[0].area < smallregion)
    {
        chain[0].small = true;
        regionsCount--;
        listp[0] = -2;
    }
    int k = listc[0];
    while (k<size-1)
    {
        if (k == 81598)
            printf("");
        if (chain[k].area < smallregion)
        {
            chain[k].small = true;
            regionsCount--;
            listc[listp[k]] = listc[k];
            listp[listc[k]] = listp[k];
        }
        k = listc[k];
    }
    sortPlanes();
    mergePlanes();
    int count = 0;
    cv::Mat_<cv::Vec3f> mat(height, width, (cv::Vec3f*)normal);
    cv::Mat av(height, width, CV_32F, average);
    cv::Mat show = cv::Mat::zeros(height, width, CV_8U);
    int timehit = 0;
    while (count<size-1)
    {
        count = listc[count];
        int root = count;

        while (chain[root].child != root)
        {
            show.data[root] = 255;
            root = chain[root].child;
        }
        vec3f& normTemp = normal[chain[count].idx];
        vec3f& meanTemp = mean[chain[count].idx];
        timehit++;

    }
    //mergeSmallSegions();
}

void NormSeg2::aveImage(short *out)
{
    for (int i = 0; i < size; i++)
    {
        out[i] = average[getRegionIndex(i)];
    }
}


void NormSeg2::regions(int* r, float* ave)
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


void NormSeg2::addNeighborPair(int neighborIndex, int i1, int i2)
{
    float cosTheta = normal[i1].align(normal[i2]);
    if (cosTheta > bin[0] || abs(grad[i1]-grad[i2])<1)
    {
        int r1 = getRegionIndex(i1);
        int r2 = getRegionIndex(i2);
        if (r1 != r2)
        {
            int r = mergeRegions(r1, r2);
            compress(r, i2);
            compress(r, i1);
        }
    }
    else
    {
        int diff = 1;
        for (; diff < binCount; diff++)
        {
            if (cosTheta > bin[diff])
            {
                break;
            }
        }
        nextNeighbor[neighborIndex] = neighborBucket[diff-1];
        neighborBucket[diff-1] = neighborIndex;
    }
}

void NormSeg2::addNeighborPair2(int neighborIndex, int i1, int i2)
{
    if ((normal[i1].len() <0.1 && normal[i2].len()<0.1) || abs(grad[i1] - grad[i2])<1)
    {
        int r1 = getRegionIndex(i1);
        int r2 = getRegionIndex(i2);
        if (r1 != r2)
        {
            int r = mergeRegions(r1, r2);
            compress(r, i2);
            compress(r, i1);
        }
    }
    else
    {
        float cosTheta = normal[i1].align(normal[i2]);
        int diff = 1;
        for (; diff < binCount; diff++)
        {
            if (cosTheta > bin[diff])
            {
                break;
            }
        }
        nextNeighbor[neighborIndex] = neighborBucket[diff - 1];
        neighborBucket[diff - 1] = neighborIndex;
    }
}


int NormSeg2::getRegionIndex(int i)
{
    int leaf = i;
    int root = chain[i].parent;
    while (root != leaf)
    {
        leaf = root;
        root = chain[leaf].parent;
    }
    return root;
}


inline void NormSeg2::compress(int root, int i)
{
    int leaf = chain[i].parent;
    while (leaf != root)
    {
        chain[i].parent = root;
        i = leaf;
        leaf = chain[i].parent;
    }
}


bool NormSeg2::normalign(int i1, int i2)
{
    double length1 = normal[i1].len();
    double length2 = normal[i2].len();

    if (length1<0.5 || length2<0.5)
        return false;
  /*  if (normal[i1]*normal[i2]>length1*length2*0.95)
        printf("%f  %f  %d %d %d %d;  ", normal[i1] * normal[i2], length1*length2, i1 / width, i1%width, i2 / width, i2%width);*/
    return normal[i1] * normal[i2]>length1*length2*0.97;
}


bool NormSeg2::predicate(int i1, int i2)
{
    vec3f sub(mean[i1].x - mean[i2].x, mean[i1].y - mean[i2].y, mean[i1].z - mean[i2].z);
    float dis1 = abs(normal[i1] * sub / normal[i1].len());
    float dis2 = abs(normal[i2] * sub / normal[i2].len());
    if (/*dis1 < 5 && dis2<5*/true)
        return true;
    else
        return false;

   /* float d2 = average[i1] - average[i2];
    if (abs(d2) < threshold)
        return true;
    d2 *= d2;
    float log1 = log(1.f + area[i1])
        * (g < area[i1] ? g : area[i1]);
    float log2 = log(1.f + area[i2])
        * (g < area[i2] ? g : area[i2]);*/
    return /*d2 < factor * ((log1 + logdelta) / area[i1]
        + ((log2 + logdelta) / area[i2]))  &&*/ normalign(i1, i2);
}

void NormSeg2::link(int i1, int i2)
{
    int pa = i1;
    int leaf = chain[pa].child;
    while (leaf != pa)
    {
        pa = chain[leaf].child;
        leaf = chain[pa].child;
    }
    chain[leaf].child = i2;
}

int NormSeg2::mergeRegions(int i1, int i2)
{
    regionsCount--;
   /* int mergedCount = area[i1] + area[i2];
    float mergedAverage = (average[i1] * area[i1]
        + average[i2] * area[i2]) / mergedCount;*/

    // merge larger index into smaller index
    if (i1 > i2)
    {
        int temp = i1;
        i1 = i2;
        i2 = temp;
    }
    chain[i1].area += chain[i2].area;
    mean[i1] = (mean[i1] * chain[i1].area + mean[i2] * chain[i2].area) / (chain[i1].area + chain[i2].area);
    chain[i2].parent = i1;
    normal[i1] += normal[i2];
    link(i1, i2);
    listc[listp[i2]] = listc[i2];
    listp[listc[i2]] = listp[i2];
    return i1;

}


void NormSeg2::segmentation()
{
    // Consider C4-connectivity here
    unsigned int index, rindex, neighborIndex, indexv, indexh;
    for (int j = height - 1; j >= 0; j--)
    {
        rindex = j*width;
        for (int i = width - 1; i >= 0; i--)
        {
            index = i + rindex;
            indexv = index + width;
            indexh = index + 1;
            int neighborIndex = index << 1;
            if (abs(normal[index].z) < 0.00001)
                continue;

                // vertical
                if (j < height - 1 && abs(normal[indexv].z)>0.00001)
                    addNeighborPair2(neighborIndex + 1, index, indexv);

                // horizontal
                if (i < width - 1 && abs(normal[indexh].z)>0.00001)
                    addNeighborPair2(neighborIndex, index, indexh);

        }
    }
    cv::Mat_<cv::Vec3f> mat(height, width, (cv::Vec3f*)normal);
    int count = listc[0];
    int hit = 0;
    cv::Mat show = cv::Mat::zeros(height, width, CV_8U);
    while (count<size-1)
    {

        int root = count;
        while (chain[root].child != root)
        {

            show.data[root] = 255;
            root = chain[root].child;
        }
        count = listc[count];
    }
    for (int i = 0; i < binCount; i++)
    {
        int neighborIndex = neighborBucket[i];
        while (neighborIndex >= 0)
        {
            int i1 = neighborIndex >> 1;
            int i2 = i1 + (0 == (neighborIndex & 1) ? 1 : width);

            int r1 = getRegionIndex(i1);
            int r2 = getRegionIndex(i2);

            if (r1 != r2 && predicate(r1, r2) && normal[r1].align(normal[r2])>bin[1])
            {
                int root = mergeRegions(r1, r2);
                compress(root, i1);
                compress(root, i2);
            }
            neighborIndex = nextNeighbor[neighborIndex];
        }
        show = cv::Mat::zeros(height, width, CV_8U);
        count = 0;
        while (count<size)
        {
            count = listc[count];
            int root = count;
            if (chain[root].area < 200)
                continue;
            while (chain[root].child != root)
            {
                show.data[root] = 255;
                root = chain[root].child;
            }

        }
    }
}

void NormSeg2::mergeSmallSegions()
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


