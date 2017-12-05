#include "houghWu.h"
#include "opencv2/core/internal.hpp"
#include "opencv2/core/core_c.h"

using namespace cv;

typedef struct CvLinePolar2
{
    float rho;
    float angle;
}
CvLinePolar2;

/*=====================================================================================*/

#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])
static CV_IMPLEMENT_QSORT_EX(icvHoughSortDescent32s, int, hough_cmp_gt, const int*)
static void
icvHoughLinesStandard2(const CvMat* img, float rho, float theta,
int threshold, CvSeq *lines, int linesMax, std::vector<int>& score)
{
    cv::AutoBuffer<int> _accum, _sort_buf;
    cv::AutoBuffer<float> _tabSin, _tabCos;

    const uchar* image;
    int step, width, height;
    int numangle, numrho;
    int total = 0;
    int i, j;
    float irho = 1 / rho;
    double scale;

    CV_Assert(CV_IS_MAT(img) && CV_MAT_TYPE(img->type) == CV_8UC1);

    image = img->data.ptr;
    step = img->step;
    width = img->cols;
    height = img->rows;

    numangle = cvRound(CV_PI / theta);
    numrho = cvRound(((width + height) * 2 + 1) / rho);

    _accum.allocate((numangle + 2) * (numrho + 2));
    _sort_buf.allocate(numangle * numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);
    int *accum = _accum, *sort_buf = _sort_buf;
    float *tabSin = _tabSin, *tabCos = _tabCos;

    memset(accum, 0, sizeof(accum[0]) * (numangle + 2) * (numrho + 2));

    float ang = 0;
    for (int n = 0; n < numangle; ang += theta, n++)
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }

    // stage 1. fill accumulator
    for (i = 0; i < height; i++)
        for (j = 0; j < width; j++)
        {
            if (image[i * step + j] != 0)
                for (int n = 0; n < numangle; n++)
                {
                    int r = cvRound(j * tabCos[n] + i * tabSin[n]);
                    r += (numrho - 1) / 2;
                    accum[(n + 1) * (numrho + 2) + r + 1]++;
                }
        }

    // stage 2. find local maximums
    for (int r = 0; r < numrho; r++)
        for (int n = 0; n < numangle; n++)
        {
            int base = (n + 1) * (numrho + 2) + r + 1;
            if (accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                sort_buf[total++] = base;
        }

    // stage 3. sort the detected lines by accumulator value
    icvHoughSortDescent32s(sort_buf, total, accum);

    // stage 4. store the first min(total,linesMax) lines to the output buffer
    linesMax = MIN(linesMax, total);
    scale = 1. / (numrho + 2);
    for (i = 0; i < linesMax; i++)
    {
        CvLinePolar2 line;
        int idx = sort_buf[i];
        score.push_back(accum[idx]);
        int n = cvFloor(idx*scale) - 1;
        int r = idx - (n + 1)*(numrho + 2) - 1;
        line.rho = (r - (numrho - 1)*0.5f) * rho;
        line.angle = n * theta;
        cvSeqPush(lines, &line);
    }
}

const int STORAGE_SIZE = 1 << 12;

static void seqToMat(const CvSeq* seq, OutputArray _arr)
{
    if (seq && seq->total > 0)
    {
        _arr.create(1, seq->total, seq->flags, -1, true);
        Mat arr = _arr.getMat();
        cvCvtSeqToArray(seq, arr.data);
    }
    else
        _arr.release();
}


/* Wrapper function for standard hough transform */
CV_IMPL CvSeq*
cvHoughLinesWu2(CvArr* src_image, void* lineStorage, 
double rho, double theta, int threshold, std::vector<int>& score)

{
    CvSeq* result = 0;

    CvMat stub, *img = (CvMat*)src_image;
    CvMat* mat = 0;
    CvSeq* lines = 0;
    CvSeq lines_header;
    CvSeqBlock lines_block;
    int lineType, elemSize;
    int linesMax = INT_MAX;
    int iparam1, iparam2;

    img = cvGetMat(img, &stub);

    if (!CV_IS_MASK_ARR(img))
        CV_Error(CV_StsBadArg, "The source image must be 8-bit, single-channel");

    if (!lineStorage)
        CV_Error(CV_StsNullPtr, "NULL destination");

    if (rho <= 0 || theta <= 0 || threshold <= 0)
        CV_Error(CV_StsOutOfRange, "rho, theta and threshold must be positive");

    lineType = CV_32FC2;
    elemSize = sizeof(float) * 2;

    if (CV_IS_STORAGE(lineStorage))
    {
        lines = cvCreateSeq(lineType, sizeof(CvSeq), elemSize, (CvMemStorage*)lineStorage);
    }
    else if (CV_IS_MAT(lineStorage))
    {
        mat = (CvMat*)lineStorage;

        if (!CV_IS_MAT_CONT(mat->type) || (mat->rows != 1 && mat->cols != 1))
            CV_Error(CV_StsBadArg,
            "The destination matrix should be continuous and have a single row or a single column");

        if (CV_MAT_TYPE(mat->type) != lineType)
            CV_Error(CV_StsBadArg,
            "The destination matrix data type is inappropriate, see the manual");

        lines = cvMakeSeqHeaderForArray(lineType, sizeof(CvSeq), elemSize, mat->data.ptr,
            mat->rows + mat->cols - 1, &lines_header, &lines_block);
        linesMax = lines->total;
        cvClearSeq(lines);
    }
    else
        CV_Error(CV_StsBadArg, "Destination is not CvMemStorage* nor CvMat*");

        icvHoughLinesStandard2(img, (float)rho,
            (float)theta, threshold, lines, linesMax,score);


    if (mat)
    {
        if (mat->cols > mat->rows)
            mat->cols = lines->total;
        else
            mat->rows = lines->total;
    }
    else
        result = lines;

    return result;
}

void HoughLinesWu(InputArray _image, OutputArray _lines, std::vector<int>& score,
    double rho, double theta, int threshold)
{
    Ptr<CvMemStorage> storage = cvCreateMemStorage(STORAGE_SIZE);
    Mat image = _image.getMat();
    CvMat c_image = image;
    CvSeq* seq = cvHoughLinesWu2(&c_image, storage, 
                        rho, theta, threshold,score);
    seqToMat(seq, _lines);
}
