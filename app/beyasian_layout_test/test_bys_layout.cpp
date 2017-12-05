#include "vt_io.h"
#include "vt_visual.h"
#include "vt_pointCloud.h"
#include "opencv2/opencv.hpp"
#include "vt_depthTrans.h"
#include <time.h>
#include <algorithm>
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "CameraPara.h"
#include "hal/hal_ni_camera3d.h"
#include "sys/sys_types.h"
#include <pthread.h>

#include "bpl_learner_prm.h"
#include "bpl_learner_token.h"
#include "bpl_tmplt.h"
#include "bpl_voting.h"
#include "mm.h"
#include "jconf.h"

using namespace vt;
using namespace std;
using namespace cv;
using namespace cainiao_robot;

#define _isnan std::isnan
#define PI 3.1416

/* Camere related varients */
static int              s_cam3d_need_send_width = 1;
static int              s_data_width = 0;
static int              s_data_height = 0;
static int              s_data_bytes_siz = 0;
static float            s_data_value_range = 256.f;
static char             s_buf_sync_w_h[32] = {0};
static float           *s_p_depth = 0;
static unsigned char  *s_pcbuf = 0;
static pthread_mutex_t   s_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t    s_cond   = PTHREAD_COND_INITIALIZER;
static int               s_alg_finished = 1;

/* Algorithm related varients */
static int                             s_max_world_cache = 8;
static cv::Mat                         s_box_bottom;
static std::vector< Mat_<Vec3f> >      s_front_worlds;
static std::vector< Mat_<Vec3f> >      s_back_worlds;
static int                             s_world_width = 0;
static int                             s_world_height = 0;
static int                             s_merge_threhold = 10;
static cv::Mat                         s_intri[6];
static cv::Mat                         s_result_show;

#define __THREAD_SIGNAL_WAIT__ \
        pthread_mutex_lock( &s_mutex ); \
        pthread_cond_wait( &s_cond, &s_mutex ); \
        pthread_mutex_unlock( &s_mutex );

#define __THREAD_SIGNAL_NORIFY__ \
        pthread_mutex_lock( &s_mutex ); \
        pthread_cond_signal( &s_cond ); \
        pthread_mutex_unlock( &s_mutex );

cv::Mat_<float> clipout_target_image(cv::Mat target);
cv::Mat fill_spots(cv::Mat cliped);
cv::Mat fill_spots_ext(cv::Mat &cliped);
cv::Mat analysis_bin_map_vertical(cv::Mat bin);
cv::Mat analysis_bin_map_horizon(cv::Mat bin);
cv::Mat corner_detect(cv::Mat src_bin);
cv::Mat edge_detect(cv::Mat src);
cv::Mat rotate_image(cv::Mat src, float angle);
cv::Mat cliped;
static int token_count = 0;
static int need_to_save = 0;

#define TOKEN_ID1 "train_0051"
#define TOKEN_ID2 "train_0052"
#define TOKEN_WIDTH_HEIGHT 142, 204

void on_bpl_img_callback(int *img, int w, int h, double result)
{
    if(need_to_save == 0)
        return;
    char name[256] = {0};
    cv::Mat small;
    cv::Mat showimg(h, w, CV_8U);
    sprintf(name, "../../params/bpl/" TOKEN_ID1 "/images/token_%04d.jpg", token_count);

    for(int j = 0; j < w*h; j ++)
    {
        showimg.data[j] = (unsigned char)img[j];
    }

    while(0)
    {
        cv::imshow("s", showimg);
        int c = cv::waitKey(33);
        if(c == 27)
            break;
    }

    cv::resize(showimg, small, cv::Size(w/10, h/10));
    cv::imwrite(name, small);
}

void on_bpl_token_callback(token_t *token)
{
    int i = 0;
    int jid = -1;
    char id[128] = {0};
    need_to_save = 1;
    jconf jc("../../params/bpl/" TOKEN_ID1 "/tokens", "token", token_count);
    jc.add_root_array("tokens");

    face_entity_t *first = token->entities[0];

    for( ; i < token->entities.size(); i ++ )
    {
        jid = -1;
        face_entity_t *entity = token->entities[i];

        if(entity->state.y < first->state.y)
        {
            need_to_save = 0;
            return;
        }


        jc.add_array_item("tokens");
        memset(id, 0, 128);
        sprintf(id, "%p", entity);
        jc.set_array_item_string("tokens", i, "id", id);
        memset(id, 0, 128);
        sprintf(id, "%p", entity->parent);
        jc.set_array_item_string("tokens", i, "parent", id);

        if(entity->parent != NULL)
        {
            jid = face_belong_junct(entity, entity->parent);
        }

        jc.set_array_item_double("tokens", i, "pjid", jid);
        jc.set_array_item_double("tokens", i, "x", entity->state.x);
        jc.set_array_item_double("tokens", i, "y", entity->state.y);
        jc.set_array_item_double("tokens", i, "w", entity->state.width);
        jc.set_array_item_double("tokens", i, "h", entity->state.height);
    }

    jc.syncf();

    token_count ++;
    need_to_save = 1;
}

void on_bpl_img_callback2(int *img, int w, int h, double result)
{
    if(need_to_save == 0)
        return;
    char name[256] = {0};
    cv::Mat small;
    cv::Mat showimg(h, w, CV_8U);
    sprintf(name, "../../params/bpl/" TOKEN_ID2 "/images/token_%04d.jpg", token_count);

    for(int j = 0; j < w*h; j ++)
    {
        showimg.data[j] = (unsigned char)img[j];
    }

    while(0)
    {
        cv::imshow("s", showimg);
        int c = cv::waitKey(33);
        if(c == 27)
            break;
    }

    cv::resize(showimg, small, cv::Size(w/10, h/10));
    cv::imwrite(name, small);
}

void on_bpl_token_callback2(token_t *token)
{
    int i = 0;
    int jid = -1;
    char id[128] = {0};
    need_to_save = 1;
    jconf jc("../../params/bpl/" TOKEN_ID2 "/tokens", "token", token_count);
    jc.add_root_array("tokens");

    face_entity_t *first = token->entities[0];

    for( ; i < token->entities.size(); i ++ )
    {
        jid = -1;
        face_entity_t *entity = token->entities[i];

        if(entity->state.y < first->state.y)
        {
            need_to_save = 0;
            return;
        }


        jc.add_array_item("tokens");
        memset(id, 0, 128);
        sprintf(id, "%p", entity);
        jc.set_array_item_string("tokens", i, "id", id);
        memset(id, 0, 128);
        sprintf(id, "%p", entity->parent);
        jc.set_array_item_string("tokens", i, "parent", id);

        if(entity->parent != NULL)
        {
            jid = face_belong_junct(entity, entity->parent);
        }

        jc.set_array_item_double("tokens", i, "pjid", jid);
        jc.set_array_item_double("tokens", i, "x", entity->state.x);
        jc.set_array_item_double("tokens", i, "y", entity->state.y);
        jc.set_array_item_double("tokens", i, "w", entity->state.width);
        jc.set_array_item_double("tokens", i, "h", entity->state.height);
    }

    jc.syncf();

    token_count ++;
    need_to_save = 1;
}

int main(int argc, char **argv)
{
#if 0
    pthread_t thread;

    init_hal();
    init_alg_configs();
    printf("init hal and alg ok\n");

    pthread_create(&thread, NULL, pthread_func_propel_boxes, NULL);

    loop_run();

#elif 0
    int i = 0;
    cv::Mat image;
    image = imread("../../libs/s_result_show.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat filled = fill_spots_ext(image);

    cv::Mat vert = analysis_bin_map_vertical(filled);
    cv::Mat hori = analysis_bin_map_horizon(filled);

    printf("\n");
    for( i = 0; i < vert.cols; i ++ )
    {
        printf("%d ", vert.data[i]);
    }
    printf("\n");
    printf("\n");
    for( i = 0; i < hori.rows; i ++ )
    {
        printf("%d ", hori.data[i]);
    }
    printf("\n");

    cv::Mat edge = edge_detect(filled);

    float angle = 0.0f;
    float max_aver = -1;
    float max_angle = 0.0f;
    cv::Mat rotated;
    int counter = 0;

    while(angle <= 360.0f)
    {
        counter = 0;
        rotated = rotate_image(filled, angle);
        cv::Mat _vert = analysis_bin_map_vertical(rotated);
        float aver = 0.0f;
        for( i = 0; i < _vert.cols; i ++ )
        {
            if(_vert.data[i] != 0)
                counter ++;
            aver += _vert.data[i];
        }
        aver = aver / counter;
        if(aver > max_aver) {
            max_angle = angle;
            max_aver = aver;
        }
        angle += 2.0f;
    }
    rotated = rotate_image(filled, max_angle);
    cliped = clipout_target_image(rotated);
    while(1)
    {
        imshow("rotated", rotated);
        imshow("origin_filled", filled);
        imshow("cliped", cliped);
        char c = cvWaitKey(33);
        if(c == 27) break;
    }



//    bpl_learner_token *token = bpl_learner_token::create();
//    token->set_compare_image(cliped.data, cliped.cols, cliped.rows);
//    token->set_gen_token_callback(gen_token_callback);
//    token->learn("prm.1002.dat", 0.65);
#endif
    int cmd;
    printf("[1]Bpl PRM learning. [2] Bpl Token Generation. 1 or 2 ?\n");
    scanf("%d", &cmd);

    switch(cmd)
    {
    case 1:
    {
        bpl_learner_prm *bpll = bpl_learner_prm::create("../../params/bpl/prm", 14);
        int limits[4] = { 4, 4, 4, 4 };
        bpll->set_junctions_limits(limits);
        bpll->set_number(4);
        bpll->learn();
        break;
    }
    case 2:
    {
        bpl_tmplt *tmplt = new bpl_tmplt();
        tmplt->load_templates_from_file("", TOKEN_WIDTH_HEIGHT);
        prm_set_templates(tmplt->tmplts());
        bpl_learner_token *token = bpl_learner_token::create();
        token->set_compare_image(cliped.data, cliped.cols, cliped.rows);
        token->set_gen_token_callback(on_bpl_img_callback, on_bpl_token_callback);
        token->learn("../../params/bpl/prm/prm.lr0001.dat", 0.65);

        token_count = 0;
        bpl_tmplt *tmplt2 = new bpl_tmplt();
        tmplt2->load_templates_from_file("", TOKEN_WIDTH_HEIGHT);
        prm_set_templates(tmplt2->tmplts());
        bpl_learner_token *token2 = bpl_learner_token::create();
        token2->set_compare_image(cliped.data, cliped.cols, cliped.rows);
        token2->set_gen_token_callback(on_bpl_img_callback2, on_bpl_token_callback2);
        token2->learn("../../params/bpl/prm/prm.lr0002.dat", 0.65);
        break;
    }
    case 3:
    {
//        bpl_voting *pvote = new bpl_voting(14,
//                                           "../../params/bpl/train_0014/tokens",
//                                           "./../params/bpl/train_0014/out",
//                                           256);
//        pvote->run();
//        delete pvote;
//        break;
    }
    }
    return 0;

}
