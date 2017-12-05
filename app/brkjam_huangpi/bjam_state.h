#ifndef BJAM_STATE_H
#define BJAM_STATE_H

#include "opencv2/opencv.hpp"

typedef struct sku_attribute_s
{
    cv::Point2f box_rect_size[3];
    int upper_index;
    char train_1[64];
    char train_2[64];
} sku_attribute_t;

extern sku_attribute_t g_default_sku;

#define BJAM_JUDGEMENT_MAX_TOKENS 16

#define BJAM_WBMIN_RATE 3.0

#define BJAM_EMPTY_COUNT 3

#define BJAM_T 8

#define BJAM_MAGIN 20
#define BJAM_MAGIN_STEP 2

#define BJAM_SCORE_ENHANCE 10
#define BJAM_FIRST_STEP 2
#define BJAM_FIRST_NOISE_NUM 3

#define BJAM_SIZE_FIXED 8

#define BJAM_MIN_SCORE 1
#define BJAM_GIVEUP 0

#define BJAM_EDGE_PARAM 0.3

#define BJAM_FIRST_SCAN_THICK 4

int bjam_state_load_sku(sku_attribute_t &sku, unsigned long uuid);

#endif // BJAM_STATE_H
