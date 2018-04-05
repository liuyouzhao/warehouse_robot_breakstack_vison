#include "bjam_ltjudgement.h"
#include "bjam_state.h"
#include "bpl_voting.h"
#include "mdl_box.h"
#include "mm.h"

#include <string.h>

float               bjam_ltjudgement::s_length_scale = 0.0f;
cv::Mat             bjam_ltjudgement::s_target_image;
int                 bjam_ltjudgement::s_left;
int                 bjam_ltjudgement::s_top;
int                 bjam_ltjudgement::s_right;
int                 bjam_ltjudgement::s_bottom;
int                 bjam_ltjudgement::s_template_pix_width;
int                 bjam_ltjudgement::s_template_pix_height;
sku_attribute_t     bjam_ltjudgement::s_current_sku;
float               bjam_ltjudgement::s_confidence = 0.0f;
int                 bjam_ltjudgement::s_focus_x = 0;
int                 bjam_ltjudgement::s_focus_y = 0;
int                 bjam_ltjudgement::s_focus_width = 0;
int                 bjam_ltjudgement::s_focus_height = 0;


static              int s_debug = 0;
cv::Mat clipout_target_image(cv::Mat target, int &l, int &r, int &t, int &b);
static int __search_first_rect(face_entity_t *root, cv::Mat image);
static int __calculate_border_score(int x, int y, int w, int h, int t, cv::Mat image);

static int s_startx = 0;
static int s_starty = 0;

static void __wait_esc()
{
    while(1)
    {
        char c = cv::waitKey(33);
        if(c == 27)
        {
            break;
        }
    }
}

static void __show_border_rect(int x, int y, int w, int h, int t, cv::Mat image)
{
    cv::Mat showimg;
    cv::cvtColor(image, showimg, cv::COLOR_GRAY2RGB);

    cv::rectangle(showimg,
                  cv::Rect(x - t, y - t, w + 2*t, h + 2*t),
                  cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::rectangle(showimg,
                  cv::Rect(x + t * 1.5, y + t * 1.5, w - 2*t * 1.5, h - 2*t * 1.5),
                  cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::imshow("border", showimg);
    __wait_esc();
}

static int __calculate_border_score(int x, int y, int w, int h, int t, cv::Mat image)
{
    int _x = x - t;
    int _y = y - t;
    int _x_to = x + w + t;
    int _y_to = y + h + t;
    int score = 0;
    int up = 0;
    int left = 0;
    int right = 0;
    int down = 0;
    int logic = 0;
//    int loss = 0;

    _x = _x < 0 ? 0 : _x;
    _y = _y < 0 ? 0 : _y;
//    if(_x_to > image.cols + t || _y_to > image.rows + t)
//    {
//        return 0;
//    }

    for(int i = _y; i < _y_to; i ++)
    {
        for(int j = _x; j < _x_to; j ++)
        {
            if(i >= image.rows || j >= image.cols || i < 0 || j < 0)
            {
                //loss ++;
                continue;
            }
            int dat = image.data[i * image.cols + j];
            if(dat != 0)
            {
                //score ++;
                if(i <= _y + t * 1.5)
                {
                    up ++;
                }
                if(j <= _x + t * 1.5)
                {
                    left ++;
                }
                if(i >= _y_to - t * 1.5)
                {
                    down ++;
                }
                if(j >= _x_to - t * 1.5)
                {
                    right ++;
                }
            }
        }
    }

    if(up > w * BJAM_EDGE_PARAM)
    {
        logic ++;
    }
    if(down > w * BJAM_EDGE_PARAM)
    {
        logic ++;
    }
    if(left > h * BJAM_EDGE_PARAM)
    {
        logic ++;
    }
    if(right > h * BJAM_EDGE_PARAM)
    {
        logic ++;
    }
    score = up + down + left + right;
    score = score * logic;

    printf("SCORE: %d\n", score);
    if(s_debug == 3)
        __show_border_rect(x, y, w, h, t, image);

    return score;
}

static int __empty_rect(int x, int y, int w, int h, cv::Mat image, int min)
{
    int count = 0;
    for(int i = y; i < y + h; i ++)
    {
        for(int j = x; j < x + w; j ++)
        {
            int dat = image.data[i * image.cols + j];
            if(dat != 0)
            {
                count ++;
            }
        }
    }
    if(count <= min)
    {
        return 1;
    }
    return 0;
}

static int __find_best_rect_score(face_entity_t *entity, std::vector<face_entity_t*> list, cv::Mat image)
{
    float scale = bjam_ltjudgement::scale();
    entity->state.x /= scale;
    entity->state.y /= scale;
    entity->state.width /= scale;
    entity->state.height /= scale;
    entity->state.width -= BJAM_SIZE_FIXED;
    entity->state.height -= BJAM_SIZE_FIXED;

    if(bjam_ltjudgement::s_focus_width == 0 && bjam_ltjudgement::s_focus_height == 0)
    {
        if(entity->state.width > entity->state.height)
        {
            bjam_ltjudgement::s_focus_width = entity->state.height;
            bjam_ltjudgement::s_focus_height = entity->state.width;
        }
        else
        {
            bjam_ltjudgement::s_focus_width = entity->state.width;
            bjam_ltjudgement::s_focus_height = entity->state.height;
        }
    }

    float ox = entity->state.x;
    float oy = entity->state.y;

    int score = 0;

    score = __search_first_rect(entity, image);

    return score;
}

static int __search_first_rect(face_entity_t *root, cv::Mat image)
{
    int score1 = 0;
    int max1 = 1;
    int w1 = root->state.width;
    int h1 = root->state.height;
    int compare = 0;

    for(int i = 0; i < BJAM_FIRST_NOISE_NUM; i ++)
    {
        int x1 = root->state.x + BJAM_FIRST_STEP;
        int y1 = root->state.y;

        /**
          (1) Horizontal
         * @brief empty
         */
        int empty = __empty_rect(x1, y1, BJAM_T, BJAM_T, image, BJAM_T);

        /*
          Going right
        */
        while(empty)
        {
            empty = __empty_rect(x1, y1, BJAM_T, BJAM_T, image, BJAM_T);
            x1 ++;
        }

        for(int t = 1; t < BJAM_T; t ++)
        {
            score1 = __calculate_border_score(x1, y1, w1, h1, t, image);
            max1 = score1 > max1 ? score1 : max1;
        }
        if(compare < max1)
        {
            root->state.x = x1;
            root->state.y = y1;
            root->state.width = w1;
            root->state.height = h1;
            compare = max1;
            /**
              For later usage, s_focus_x/y will be used later before picking
              */
            bjam_ltjudgement::s_focus_x = x1;
            bjam_ltjudgement::s_focus_y = y1;
        }
    }

    return max1 * BJAM_SCORE_ENHANCE;
}

static int __find_right_rect_score(face_entity_t *entity,
                                   std::vector<face_entity_t*> list,
                                   cv::Mat image, int offset)
{
    int score1 = 0;
    int max1 = 1;
    int compare = 0;
    int empty = 0;

    if(entity->state.x + BJAM_MAGIN >= image.cols)
    {
        return BJAM_GIVEUP;
    }

    int x1 = entity->state.x;
    int y1 = entity->state.y;
    int w1 = entity->state.width;
    int h1 = entity->state.height;

    /**
      (1) Horizontal
    */
    /* Going Down */
    if(x1 + (w1 * 0.8) <= image.cols + w1 * 0.3 &&
       y1 + (h1 * 0.8) <= image.rows + h1 * 0.3)
    {
        do
        {
            empty = __empty_rect(x1, y1, w1, BJAM_T, image, w1 / BJAM_T);
            y1 ++;
            if(y1 > image.rows - h1 + BJAM_T)
                break;
        }
        while(empty);

        if(empty)
        {
            max1 = BJAM_MIN_SCORE;
        }
        else
        {
            for(int t = 1; t < BJAM_T; t ++)
            {
                score1 = __calculate_border_score(x1, y1, w1, h1,
                                                  t, image);
                max1 = max1 < score1 ? score1 : max1;
                if(compare < max1)
                {
                    entity->state.x = x1 - t;
                    entity->state.y = y1 - t;
                    entity->state.width = w1;
                    entity->state.height = h1;
                }
            }
        }
    }
    else
    {
        max1 = BJAM_MIN_SCORE;
    }
    return max1;
}

static int __find_bottom_rect_score(face_entity_t *entity,
                                    std::vector<face_entity_t*> list,
                                    cv::Mat image,
                                    int offset)
{
    int score1 = 0;
    int max1 = 1;
    int compare = 0;
    int empty = 0;

    if(entity->state.y + BJAM_MAGIN >= image.rows)
    {
        return BJAM_GIVEUP;
    }

    int x1 = entity->state.x;
    int y1 = entity->state.y;
    int w1 = entity->state.width;
    int h1 = entity->state.height;

    /**
      (1) Horizontal
    */
    /* Going Right */
    if(x1 + (w1 * 0.8) <= image.cols + w1 * 0.3 &&
       y1 + (h1 * 0.8) <= image.rows + h1 * 0.3)
    {
        do
        {
            empty = __empty_rect(x1, y1, BJAM_T, h1, image, h1 / BJAM_T);
            x1 ++;
            if(x1 > image.cols - w1 + BJAM_T)
                break;
        }
        while(empty);

        if(empty)
        {
            max1 = BJAM_MIN_SCORE;
        }
        else
        {
            for(int t = 1; t < BJAM_T; t ++)
            {
                score1 = __calculate_border_score(x1, y1, w1, h1,
                                                  t, image);
                max1 = max1 < score1 ? score1 : max1;
                if(compare < max1)
                {
                    entity->state.x = x1 - t;
                    entity->state.y = y1 - t;
                    entity->state.width = w1;
                    entity->state.height = h1;
                }
            }
        }
    }
    else
    {
        max1 = BJAM_MIN_SCORE;
    }
    return max1;
}


static int __border_evaluate_callback(face_entity_t *entity, std::vector<face_entity_t*> list)
{
    cv::Mat image = bjam_ltjudgement::get_target_image();

    cv::Mat temp1;
    cv::dilate(image, temp1, cv::Mat());
    cv::Mat dot = temp1 - image;

    if(entity->parent == NULL)
    {

        int result = __find_best_rect_score(entity, list, dot);

        return result;
    }
    else
    {
        float scale = bjam_ltjudgement::scale();
        int px = entity->parent->state.x;
        int py = entity->parent->state.y;
        int pw = entity->parent->state.width;
        int ph = entity->parent->state.height;
        int ow = entity->state.width;
        int oh = entity->state.height;
        int sx, sy;

        entity->state.width /= scale;
        entity->state.height /= scale;
        entity->state.width -= BJAM_SIZE_FIXED;
        entity->state.height -= BJAM_SIZE_FIXED;

        int jid = face_belong_junct(entity, entity->parent);
        switch (jid) {
        case 0: /* Right */
        {
            int max = 0;
            py = 0;
            for(int f = 0; f < BJAM_MAGIN; f += BJAM_MAGIN_STEP)
            {
                entity->state.x = px + pw + f;
                entity->state.y = py;
                int score = __find_right_rect_score(entity, list, dot, BJAM_MAGIN);
                score = score / (f + 1);
                max = max < score ? score : max;
            }

            return max;
            break;
        }
        case 1:  /* Down */
        {
            int max = 0;
            for(int f = 0; f < BJAM_MAGIN; f += BJAM_MAGIN_STEP)
            {
                entity->state.x = px;
                entity->state.y = py + ph + f;
                int score = __find_bottom_rect_score(entity, list, dot, BJAM_MAGIN);
                score = score / (f + 1);
                max = max < score ? score : max;
            }
            return max;
            break;
        }
        case 2:
            printf("not implemented");
            break;
        case 3:
            printf("not implemented");
            break;
        default:
            break;
        }
    }
}

static int __show_result_callback(std::vector<face_entity_t*> list)
{
    if(s_debug < 2)
        return 0;

    cv::Mat showimg;
    cv::cvtColor(bjam_ltjudgement::get_target_image(), showimg, cv::COLOR_GRAY2RGB);

    for(int i = 0; i < list.size(); i ++)
    {
        face_entity_t *e = list[i];
        cv::rectangle(showimg,
                      cv::Rect(e->state.x, e->state.y, e->state.width, e->state.height),
                      cv::Scalar(0, 0, 255), 1, 8, 0);
    }

    cv::imshow("result", showimg);
    __wait_esc();

    return 0;
}

void bjam_ltjudgement::run(int &type, int debug)
{
    int t1 = 0;
    int t2 = 0;
    float c1 = 0.0f;
    float c2 = 0.0f;
    s_debug = debug;

    char token_path1[256] = {0};
    char token_path2[256] = {0};
    memset(token_path1, 0, 256);
    memset(token_path2, 0, 256);
    sprintf(token_path1, "../../params/bpl/%s/tokens", s_current_sku.train_1);
    sprintf(token_path2, "../../params/bpl/%s/tokens", s_current_sku.train_2);

    bpl_voting *pvote = new bpl_voting(1,
                                       token_path1,
                                       BJAM_JUDGEMENT_MAX_TOKENS);

    pvote->set_vote_border_rgb_callback(__border_evaluate_callback);
    pvote->set_show_result_callback(__show_result_callback);
    t1 = pvote->run(c1, s_debug);

    bpl_voting *pvote2 = new bpl_voting(2,
                                       token_path2,
                                       BJAM_JUDGEMENT_MAX_TOKENS);
    pvote2->set_vote_border_rgb_callback(__border_evaluate_callback);
    pvote2->set_show_result_callback(__show_result_callback);
    t2 = pvote2->run(c2, s_debug);

    delete pvote;
    delete pvote2;

    if(t1 == t2)
    {
        s_confidence = (c1 + c2) * 0.5;
        type = t1;
    }
    else
    {
        type = c1 > c2 ? t1 : t2;
        s_confidence = c1 > c2 ? c1 : c2;
    }


    if(s_debug == 1)
    {
        cv::Mat showimg;
        cv::cvtColor(get_target_image(), showimg, cv::COLOR_GRAY2RGB);

        if(type == 2)
        {
            cv::rectangle(showimg,
                          cv::Rect(s_focus_x, s_focus_y, s_focus_height, s_focus_width),
                          cv::Scalar(0, 0, 255), 4, 8, 0);
        }
        else
        {
            cv::rectangle(showimg,
                          cv::Rect(s_focus_x, s_focus_y, s_focus_width, s_focus_height),
                          cv::Scalar(0, 0, 255), 4, 8, 0);
        }
        cv::imshow("bpl", showimg);
    }
}

int bjam_ltjudgement::init_params(float ls, cv::Mat target_image, sku_attribute_t sku)
{
    s_length_scale = ls;
    s_target_image = target_image.clone();

    clipout_target_image(s_target_image, s_left, s_right, s_top, s_bottom);

    memcpy(&s_current_sku, &sku, sizeof(sku_attribute_t));
}
