#include "opencv2/opencv.hpp"
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

#define SAMPLES 100
#define T_MAGIN 1

#define POSS 1

#if POSS
#define T_WIDTH 75
#define T_HEIGHT 53
#define NOISE_M 0.5
#define FILE_NAME "t3-n/vec-desc-"
#else
#define T_WIDTH 53
#define T_HEIGHT 75
#define NOISE_M 0.5
#define FILE_NAME "t3-t/vec-desc-ivt-"
#endif
using namespace cv;

float test_B_Dist(Mat b, int t = 0);

static double SIGMOID_A[] = {
    //-3.450447486214617,
    //0.1338399426888076

    /* DNN */
    //-3.523751335789116,
    //0.2544172679008934

    /* BIN */
    -3.28699062555766,
    -0.004186793723420901
};

static Mat sColor;

//static double svmMul(int *arr, int len)
//{
//    double rtarr[len + 1];
//    memset(rtarr, 0, sizeof(double) * (len + 1));
//    for(int i = 0; i < len; i ++)
//    {
//        rtarr[i] = (double)(arr[i]);
//    }
//    rtarr[len] = 1.0;

//    double result = 0.0;
//    for(int i = 0; i < len + 1; i ++)
//    {
//        result += rtarr[i] * SVM_A[i];
//    }
//    return result;
//}

static double sigmoidMul(double *arr, int len)
{
    double rtarr[len + 1];
    memset(rtarr, 0, sizeof(double) * (len + 1));
    for(int i = 0; i < len; i ++)
    {
        rtarr[i] = (double)(arr[i]);
    }
    rtarr[len] = 1.0;

    double result = 0.0;
    for(int i = 0; i < len + 1; i ++)
    {
        result += rtarr[i] * SIGMOID_A[i];
    }
    return result;
}

static void invertBinary(Mat &mat)
{
    for(int i = 0; i < mat.rows; i ++)
    {
        for(int j = 0; j < mat.cols; j ++)
        {
            int value = mat.at<unsigned char>(i, j);
            value = value == 0 ? 255 : 0;
            mat.at<unsigned char>(i, j) = (unsigned char)value;
        }
    }
}

static int readIntArrayFromFile(const char *filename, int *arr, int *len)
{
    char buffer[1024];
    memset(buffer, 0, 1024);

    FILE *f = fopen(filename, "r");
    if(f == NULL)
    {
        return -1;
    }
    *len = fread(buffer, 1, 1024, f);
    fclose(f);

    int l = 0;
    int index = 0;
    for(int i = 0; i < *len; i ++)
    {
        char c = buffer[i];
        char num[32] = {0};
        if(c == ' ')
        {
            memcpy(num, (char*) buffer + l, i - l);
            arr[index++] = atoi(num);
            l = i + 1;
        }
    }
    *len = index;
    for(int i = 0; i < *len; i ++)
    {
        printf("%d ", arr[i]);
    }
    printf("\n");
    return 0;
}

static void writeIntArrayToFile(const char *filename, int *arr, int len)
{
    char buffer[len * 4];
    memset(buffer, 0, len * 4);
    for(int i = 0; i < len; i ++)
    {
        sprintf(buffer, "%s%d ", buffer, arr[i]);
    }
    printf("%s\n", buffer);

    FILE *f = fopen(filename, "w");
    if(f == NULL)
    {
        return;
    }
    fwrite(buffer, 1, strlen(buffer), f);
    fclose(f);
}

static void projectPixels(Mat binary, int *result)
{
    int sumVert[binary.cols];
    int sumHori[binary.rows];

    memset(sumVert, 0, sizeof(int) * binary.cols);
    memset(sumHori, 0, sizeof(int) * binary.rows);
    for(int i = 0; i < binary.rows; i ++)
    {
        for(int j = 0; j < binary.cols; j ++)
        {
            int v = binary.at<unsigned char>(i, j);
            sumVert[j] += v == 255 ? 1 : 0;
            sumHori[i] += v == 255 ? 1 : 0;
        }
    }
    for(int i = 0; i < binary.cols; i ++)
    {
        if(i <= (binary.cols * NOISE_M) || i >= (binary.cols * (1.0 - NOISE_M)))
            result[i] = sumVert[i];
        else
            result[i] = 0;
    }
    for(int i = 0; i < binary.rows; i ++)
    {
        if(i <= (binary.rows * NOISE_M) || i >= (binary.rows * (1.0 - NOISE_M)))
            result[i + binary.cols] = sumHori[i];
        else
            result[i + binary.cols] = 0;
    }
}

static void handleProjectPixel(int x, int y, int w, int h, Mat color, const char *title, int index)
{
    Rect region = Rect(x, y, w, h);
    Mat croped = color(region);
    //invertBinary(croped);

    int vecWidth = region.width / 4;
    int vecHeight = region.height / 4;
    Mat cropedResized = Mat(vecWidth, vecHeight, CV_8U);
    Size size(vecWidth, vecHeight);
    resize(croped, cropedResized, size);

    int vecDesc[cropedResized.cols + cropedResized.rows];
    memset(vecDesc, 0, cropedResized.cols + cropedResized.rows);

    projectPixels(cropedResized, vecDesc);

    char filename[64] = {0};
    char cmd[128] = {0};
    sprintf(cmd, "mkdir -p ../../libs/%s", title);
    system(cmd);

    sprintf(filename, "../../libs/%s/%d.proj", title, index);
    writeIntArrayToFile(filename, vecDesc, cropedResized.cols + cropedResized.rows);
}

static void all_observation(Mat src, const char *name)
{
    int tw = T_WIDTH + T_MAGIN * 2;
    int th = T_HEIGHT + T_MAGIN * 2;
    for(int i = 0; i < src.rows - th; i ++)
    {
        for(int j = 0; j < src.cols - tw; j ++)
        {
            handleProjectPixel(j, i, tw, th, src, name, i * src.cols + j);
        }
    }

    for(int i = 0; i < src.rows - tw; i ++)
    {
        for(int j = 0; j < src.cols - th; j ++)
        {
            handleProjectPixel(j, i, th, tw, src, name, -(i * src.cols + j));
        }
    }
}

typedef struct hmm_state_s
{
    float probability;
    Rect rect;
    int level;
    struct hmm_state_s *father;
} hmm_state_t;

typedef struct hmm_state_map_node_s
{
    Point point;
    float probability;
    hmm_state_t *p_state;
} hmm_state_map_node_t;

float predictB(Mat b1);
#define MIN_RATE 0.5
#define MIN_PREDICT 0.4
#define HMM_TOLERATE_REGION 10
#define MAX_LEVEL 1
static vector< vector<hmm_state_t*> > hmm_tree;
static vector< hmm_state_t* > hmm_queue;
static vector< hmm_state_t* > hmm_map1;
static vector< hmm_state_t* > hmm_map2;
//static vector< hmm_state_map_node_t > hmm_map_state;
static Rect hmm_start_region = Rect(0, 0, HMM_TOLERATE_REGION, HMM_TOLERATE_REGION);

static bool is_cross_parent(hmm_state_t stt, hmm_state_t kid)
{
    Rect r = stt.rect;
    Rect k = kid.rect;

    bool cross = false;
    if(r.x + r.width < k.x || r.y + r.height < k.y ||
       k.x + k.width < r.x || k.y + k.height < r.y)
    {
        cross = false;
    }
    else
    {
        return true;
    }

    hmm_state_t *ptr = stt.father;
    while(ptr)
    {
        r = ptr->rect;
        if(r.x + r.width < k.x || r.y + r.height < k.y ||
           k.x + k.width < r.x || k.y + k.height < r.y)
        {
            cross = false;
        }
        else
        {
            return true;
        }
        ptr = ptr->father;
    }
    return cross;
}

static bool is_outof_image(Mat image, Rect rect)
{
    if(rect.x >= 0 && rect.x + rect.width < image.cols &&
       rect.y >= 0 && rect.y + rect.height < image.rows)
    {
        return false;
    }
    return true;
}

static void hmm_try_push_node(hmm_state_t *stt, hmm_state_t **child, Mat src, int clz)
{
    hmm_state_t *node = *child;
    Rect nr = node->rect;
    if(!is_cross_parent(*stt, *node) && !is_outof_image(src, nr))
    {
        Mat b = src(node->rect);
        float rate = predictB(b);

        if(MIN_PREDICT < rate)
        {
            float cond_rate = stt->probability * rate;

            if(cond_rate > pow(MIN_RATE, (double)(stt->level) + 1))
            {
                node->probability = cond_rate;
                node->father = stt;
                node->level = stt->level + 1;



                int mindex = node->rect.y * src.cols + node->rect.x;

                if(clz == 1)
                {
                    if(hmm_map1[mindex] != NULL && hmm_map1[mindex]->rect == node->rect &&
                       hmm_map1[mindex]->level == node->level)
                    {
                        if(hmm_map1[mindex]->probability < node->probability)
                        {
                            hmm_map1[mindex]->probability = node->probability;
                            hmm_map1[mindex]->father = node->father;
                            hmm_map1[mindex]->level = node->level;
                        }
                    }
                    else if(hmm_map1[mindex] == NULL)
                    {
                        hmm_map1[mindex] = node;
                        hmm_tree[node->level].push_back(node);
                        hmm_queue.push_back(node);
                        printf("[%d] %f\n", node->level, node->probability);
                        return;
                    }
                    else
                    {
                        hmm_tree[node->level].push_back(node);
                        hmm_queue.push_back(node);
                        printf("[%d] %f\n", node->level, node->probability);
                        return;
                    }
                }
                else if(clz == -1)
                {
                    if(hmm_map2[mindex] != NULL && hmm_map2[mindex]->rect == node->rect &&
                       hmm_map2[mindex]->level == node->level)
                    {
                        if(hmm_map2[mindex]->probability < node->probability)
                        {
                            hmm_map2[mindex]->probability = node->probability;
                            hmm_map2[mindex]->father = node->father;
                            hmm_map2[mindex]->level = node->level;
                        }
                    }
                    else if(hmm_map2[mindex] == NULL)
                    {
                        hmm_map2[mindex] = node;
                        hmm_tree[node->level].push_back(node);
                        hmm_queue.push_back(node);
                        printf("[%d] %f\n", node->level, node->probability);
                        return;
                    }
                    else
                    {
                        hmm_tree[node->level].push_back(node);
                        hmm_queue.push_back(node);
                        printf("[%d] %f\n", node->level, node->probability);
                        return;
                    }
                }
            }
        }


    }
    delete *child;
    *child = NULL;
}

/**
 * @brief hmm_expand_queue_nodes
 * @param src
 * @param stt
 * @param jp
 * @param thick
 * @param epsl
 *
 * for new_stt:
 * 1           2
 * -------------
 * |           |
 * |           |
 * |           |
 * -------------
 * 3           4
 */
static void hmm_expand_queue_nodes(Mat src, hmm_state_t *stt, int jp, int thick)
{
    Rect r = stt->rect;
    int rl = r.x - thick;
    int rt = r.y - thick;
    int rr = r.x + r.width + thick;
    int rb = r.y + r.height + thick;

    int tw1 = T_WIDTH + T_MAGIN * 2;
    int th1 = T_HEIGHT + T_MAGIN * 2;

    int tw2 = th1;
    int th2 = tw1;

    for(int i = rt; i < rb; i += jp)
    {
        for(int j = rl; j < rr; j += jp)
        {
            if(j < r.x + r.width && j > r.x &&
                i < r.y + r.height && i > r.y)
            {
                continue;
            }
            /* Class 1*/
            hmm_state_t *new_stt1_1 = new hmm_state_t();
            hmm_state_t *new_stt2_1 = new hmm_state_t();
            hmm_state_t *new_stt3_1 = new hmm_state_t();
            hmm_state_t *new_stt4_1 = new hmm_state_t();

            /* Class 2*/
            hmm_state_t *new_stt1_2 = new hmm_state_t();
            hmm_state_t *new_stt2_2 = new hmm_state_t();
            hmm_state_t *new_stt3_2 = new hmm_state_t();
            hmm_state_t *new_stt4_2 = new hmm_state_t();

            new_stt1_1->rect = Rect(j, i, tw1, th1);
            new_stt1_2->rect = Rect(j, i, tw2, th2);

            new_stt2_1->rect = Rect(j - tw1, i, tw1, th1);
            new_stt2_2->rect = Rect(j - tw2, i, tw2, th2);

            new_stt3_1->rect = Rect(j, i - th1, tw1, th1);
            new_stt3_2->rect = Rect(j, i - th2, tw2, th2);

            new_stt4_1->rect = Rect(j - tw1, i - th1, tw1, th1);
            new_stt4_2->rect = Rect(j - tw2, i - th2, tw2, th2);

            hmm_try_push_node(stt, &new_stt1_1, src, 1);
            hmm_try_push_node(stt, &new_stt1_2, src, -1);
            hmm_try_push_node(stt, &new_stt2_1, src, 1);
            hmm_try_push_node(stt, &new_stt2_2, src, -1);
            hmm_try_push_node(stt, &new_stt3_1, src, 1);
            hmm_try_push_node(stt, &new_stt3_2, src, -1);
            hmm_try_push_node(stt, &new_stt4_1, src, 1);
            hmm_try_push_node(stt, &new_stt4_2, src, -1);
        }
    }
}

static bool hmm_best_sort(hmm_state_t *left, hmm_state_t *right)
{
    return left->probability > right->probability;
}

static void hmm_go(Mat image, Point start, int max_levels)
{
    /* t = 0, multi-nodes in layer 1 */
    int sl = start.x - (hmm_start_region.width >> 1);
    int st = start.y - (hmm_start_region.height >> 1);
    int sr = start.x + (hmm_start_region.width >> 1);
    int sb = start.y + (hmm_start_region.height >> 1);

    int tw = T_WIDTH + T_MAGIN * 2;
    int th = T_HEIGHT + T_MAGIN * 2;

    Mat src = image.clone();

    /*(0) Init hmm_map_state */
    for(int i = 0; i < image.rows; i ++)
    {
        for(int j = 0; j < image.cols; j ++)
        {
            hmm_map1.push_back(NULL);
            hmm_map2.push_back(NULL);
        }
    }


    /*(1) Init tree */
    for(int i = 0; i < max_levels; i ++)
    {
        vector<hmm_state_t*> vec;
        hmm_tree.push_back(vec);
    }

    for(int i = st; i < sb; i ++)
    {
        for(int j = sl; j < sr; j ++)
        {
            hmm_state_t *f1 = new hmm_state_t();
            hmm_state_t *f2 = new hmm_state_t();
            f1->rect = Rect(j, i, tw, th);
            f2->rect = Rect(j, i, th, tw);
            f1->father = NULL;
            f2->father = NULL;

            Mat b1 = src(f1->rect);
            Mat b2 = src(f2->rect);

            f1->probability = 1.0 * predictB(b1);
            f2->probability = 1.0 * predictB(b2);

            if(f1->probability > MIN_RATE)
            {
                hmm_queue.push_back(f1);
                hmm_tree[0].push_back(f1);
            }
            if(f2->probability > MIN_RATE)
            {
                hmm_queue.push_back(f2);
                hmm_tree[0].push_back(f2);
            }
        }
    }


    int iter = 0;
    while(iter < hmm_queue.size())
    {
        if(hmm_queue[iter]->level <= MAX_LEVEL)
        {
            hmm_expand_queue_nodes(src, hmm_queue[iter], 2, 10);
        }
        iter ++;
    }

    int last_lever = -1;
    for(int i = 0; i < hmm_tree.size(); i ++)
    {
        if(hmm_tree[i].size() == 0)
        {
            break;
        }
        last_lever ++;
    }
    std::sort(hmm_tree[last_lever].begin(), hmm_tree[last_lever].end(), hmm_best_sort);

    int bestn = hmm_tree[last_lever].size() < 5 ? hmm_tree[last_lever].size() : 5;
    for(int i = 0; i < bestn; i ++)
    {
        hmm_state_t *node = hmm_tree[last_lever][i];
        Mat show = image.clone();
        while(1)
        {
            printf("[%f]<-", node->probability);

            cv::rectangle(show, node->rect, Scalar(0, 0, 255), 2);

            node = node->father;
            if(node == NULL)
            {
                break;
            }
        }
        char ni[16] = {0};
        sprintf(ni, "%d", i);
        imshow(string("show") + string(ni), show);
        printf("\n");
    }
}

float predictB(Mat b1)
{
    float p1 = test_B_Dist(b1);

    double pp1[2] = {
        p1, 1.0
    };

    double reg1 = sigmoidMul(pp1, 1);

    double prob1 = 1.0 / (1.0 + exp(reg1));
    //printf("predict: %f\n", prob1);

    return prob1;
}


void predictBB(Mat b1, Mat b2, float &pb1, float &pb2)
{
    float p1 = test_B_Dist(b1);
    float p2 = test_B_Dist(b2);

    double pp1[2] = {
        p1, 1.0
    };
    double pp2[2] = {
        p2, 1.0
    };

    double reg1 = sigmoidMul(pp1, 1);
    double reg2 = sigmoidMul(pp2, 1);

    double prob1 = 1.0 / (1.0 + exp(reg1));
    double prob2 = 1.0 / (1.0 + exp(reg2));
    printf("1-predict: %f\n", prob1);
    printf("2-predict: %f\n", prob2);

    pb1 = prob1;
    pb2 = prob2;
}

static void onMouseHandler(int event, int x, int y, int flag, void* file)
{
    switch (event)
    {
    case EVENT_LBUTTONUP:
    {
#if 0
        int tw = T_WIDTH + T_MAGIN * 2;
        int th = T_HEIGHT + T_MAGIN * 2;

        Mat b1 = sColor(Rect(x, y, tw, th));
        Mat b2 = sColor(Rect(x, y, th, tw));

        imshow("b1", b1);
        imshow("b2", b2);

        float f1;
        float f2;
        predictBB(b1, b2, f1, f2);

        printf("%f \n %f", f1, f2);
#endif
        hmm_go(sColor, Point(x, y), 10);

    } break;
    case EVENT_RBUTTONUP:
    {
        printf("%d %d\n", x, y);
        int tw = T_WIDTH + T_MAGIN * 2;
        int th = T_HEIGHT + T_MAGIN * 2;

        Mat b1 = sColor(Rect(x, y, tw, th));
        Mat b2 = sColor(Rect(x, y, th, tw));

        imshow("b1", b1);
        imshow("b2", b2);

        float f1;
        float f2;
        predictBB(b1, b2, f1, f2);

        printf("%f \n %f", f1, f2);
    } break;
    }
}

int run_hmm()
{
    char fullpath[128];
    char filename[64];
    for( int i = 0; i < SAMPLES; i ++ )
    {
        memset(fullpath, 0, 128);
        memset(filename, 0, 64);

        /* For dnn */
#if USE_DNN
        sprintf(filename, "dnn/%d.png", i);
        sprintf(fullpath, "../../libs/%s", filename);
#else
        sprintf(filename, "binary/col_%d.jpg", i);
        sprintf(fullpath, "../../libs/%s", filename);
#endif
        /* For bin */


        sColor = cv::imread(fullpath);

        if(sColor.cols == 0 || sColor.rows == 0)
        {
            continue;
        }

        imshow(filename, sColor);
        setMouseCallback(filename, onMouseHandler, filename);

        while(1)
        {
            char k = waitKey(100);
            if(k == 27)
                break;
        }
        cv::destroyWindow(std::string(filename));

    }
}
