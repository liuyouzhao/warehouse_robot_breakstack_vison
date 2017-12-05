/*
 * bpl_learner_token.cpp
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#include "bpl_learner_token.h"
#include "bpl_box_shape_proc.h"
#include "mm.h"
#include "opencv2/opencv.hpp"

static double loss_cross = 0.001;
static int speed = 10;


bpl_learner_token::bpl_learner_token() {
    m_token = (token_t*) DEBUG_MALLOC(sizeof(token_t));
    m_token->entities.clear();
    m_tender = (tender_t*) DEBUG_MALLOC(sizeof(tender_t));
    m_tender->img = NULL;
    bpl_token_img_callback = NULL;
}

bpl_learner_token::~bpl_learner_token() {
    if(m_tender->img)
    {
        DEBUG_FREE(m_tender->img);
        m_tender->img = NULL;
    }
    DEBUG_FREE(m_token);
    DEBUG_FREE(m_tender);
}

bpl_learner_token *bpl_learner_token::create()
{
    bpl_learner_token *tk = new bpl_learner_token();
    return tk;
}

static int *genimage(std::vector<face_entity_t*> entities, double &threhold, int &wout, int &hout)
{
    /* bounding box */
    int minx = 99999;
    int miny = 99999;
    int maxx = -99999;
    int maxy = -99999;
    for(int i = 0; i < entities.size(); i ++)
    {
        face_entity_t *en = entities[i];
        int x = en->state.x;
        int y = en->state.y;
        int w = en->state.width;
        int h = en->state.height;

        minx = minx > x ? x : minx;
        miny = miny > y ? y : miny;
        maxx = maxx < x + w ? x + w: maxx;
        maxy = maxy < y + h ? y + h: maxy;
    }
    int width = maxx - minx;
    int height = maxy - miny;

    int len = sizeof(int) * width * height;
    int *img = (int*)DEBUG_MALLOC(len);
    memset(img, 0, width * height * sizeof(int));
    for(int i = 0; i < entities.size(); i ++)
    {
        face_entity_t *en = entities[i];
        int x = en->state.x - minx;
        int y = en->state.y - miny;
        int w = en->state.width;
        int h = en->state.height;

        for(int y1 = y; y1 < y + h; y1 ++)
        {
            for(int x1 = x; x1 < x + w; x1 ++)
            {
                int l = y1 * width + x1;
                if(img[l] > 0)
                {
                    threhold *= (1.0f - loss_cross);
                }
                img[l] = 255 * (i + 1) / entities.size();
            }
        }
    }
    wout = width;
    hout = height;
    return img;
}

static void compare(int *i1, int *i2, int w1, int h1, int w2, int h2, double &threhold)
{
    int w = w1 < w2 ? w1 : w2;
    int h = h1 < h2 ? h1 : h2;
    int pall = 0;
    int p1 = 0;
    int p2 = 0;

    for(int i = 0; i < h1 * w1; i ++)
    {
        if(i1[i] > 0)
            p1 ++;
    }
    for(int i = 0; i < h2 * w2; i ++)
    {
        if(i2[i] > 0)
            p2 ++;
    }

    for(int i = 0; i < h; i ++)
    {
        for(int j = 0; j < w; j ++)
        {
            if(i1[i * w1 + j] > 0 && i2[i * w2 + j] > 0)
            {
                pall ++;
            }
        }
    }
    double p12 = (p1 + p2) / 2.0f;
    double r = (double)pall / p12;

    threhold = threhold * r;
}

static void recurse_entities_init(face_entity_t *root, std::vector<face_entity_t*> &list)
{
    root->state.x = -9999;
    root->state.y = -9999;
    root->state.width = root->ptmplt->width;
    root->state.height = root->ptmplt->height;

    list.push_back(root);
    for(int i = 0; i < 4; i ++)
    {
        if(root->juncts[i] != NULL && root->juncts[i]->jid != -1)
        {
            for(int j = 0; j < root->juncts[i]->pointers.size(); j ++)
            {
                face_entity_t *pt = (face_entity_t*) root->juncts[i]->pointers[j];
                recurse_entities_init(pt, list);
            }
        }
    }
}

static void recurse_entities(face_entity_t *root, std::vector<face_entity_t*> &list)
{
    list.push_back(root);
    for(int i = 0; i < 4; i ++)
    {
        if(root->juncts[i] != NULL && root->juncts[i]->jid != -1)
        {
            for(int j = 0; j < root->juncts[i]->pointers.size(); j ++)
            {
                face_entity_t *pt = (face_entity_t*) root->juncts[i]->pointers[j];
                recurse_entities(pt, list);
            }
        }
    }
}


static void dump_entities(std::vector<face_entity_t*> entities)
{
    for(int i = 0; i < entities.size(); i ++)
    {
        face_entity_t *p = entities[i];
        printf("[%d] %d %d %d %d\n", p->ptmplt->id, p->state.x, p->state.y,
                                    p->state.width, p->state.height);
    }
}

static void generate_token(std::vector<face_entity_t*> es, token_t *token)
{
    token->entities.clear();
    for(int i = 0; i < es.size(); i ++)
    {
        token->entities.push_back(es[i]);
    }
}

void bpl_learner_token::on_token_image_callback(face_entity_t *root)
{
    int w, h;
    token_t *token = new token_t();
    std::vector<face_entity_t*> es;
    recurse_entities(root, es);
    generate_token(es, token);
    dump_entities(es);
    printf("\n\n");

    double thre = 1.0;
    int *img = genimage(es, thre, w, h);

    if(bpl_token_tokens_callback != NULL)
    {
        bpl_token_tokens_callback(token);
    }
    if(bpl_token_img_callback != NULL)
    {
        bpl_token_img_callback(img, w, h, thre);
    }
    DEBUG_FREE(img);
    delete token;
}

static void flip_xy( face_entity_t *face )
{
    int w = face->state.width;
    face->state.width = face->state.height;
    face->state.height = w;
}


void bpl_learner_token::generate_all_posibilities_ext(  std::vector<face_entity_t*> &list,
                                                        int index,
                                                        face_entity_t *root  )
{
    if(index >= list.size())
    {
        return;
    }
    face_entity_t *seed = list[index];

    int yf = 0;
    int yt = 0;

    if(seed->parent == NULL)
    {
        generate_all_posibilities(list, index + 1, root);
        return;
    }
    int junct = face_belong_junct(seed, seed->parent);

    /* right && left */
    for(int l = 0; l < 2; l ++)
    {
        flip_xy(seed);
        if(junct == 0 || junct == 2)
        {
            seed->state.x = junct == 0 ?
                    seed->parent->state.x + seed->parent->state.width :
                    seed->parent->state.x - seed->state.width;

            joints_t jts = seed->parent->ptmplt->joints[junct];
            for(int i = 0; i < jts.num; i ++)
            {
                seed->state.y = seed->parent->state.y + jts.offsets[i];
                if(bpl_box_shape_proc::find_if_conflict(list,
                                    seed->state.x, seed->state.y,
                                    seed->state.width, seed->state.height, index))
                {
                    continue;
                }

                if(index + 1 >= list.size())
                {
                    on_token_image_callback(root);
                }
                else
                {
                    generate_all_posibilities(list, index + 1, root);
                }
            }
        }
        /* up && down */
        else if(junct == 1 || junct == 3)
        {
            int xf = seed->parent->state.x - seed->state.width + 1;
            int xt = seed->parent->state.x + seed->parent->state.width - 1;

            seed->state.y = junct == 1 ?
                    seed->parent->state.y + seed->parent->state.height :
                    seed->parent->state.y - seed->state.height;

            joints_t jts = seed->parent->ptmplt->joints[junct];
            for(int i = 0; i < jts.num; i ++)
            {
                seed->state.x = seed->parent->state.x + jts.offsets[i];
                if(bpl_box_shape_proc::find_if_conflict(list,
                                    seed->state.x, seed->state.y,
                                    seed->state.width, seed->state.height, index))
                {
                    continue;
                }

                if(index + 1 >= list.size())
                {
                    on_token_image_callback(root);
                }
                else
                {
                    generate_all_posibilities(list, index + 1, root);
                }
            }
        }
        seed->state.y = seed->state.x = 0;
    }
}

void bpl_learner_token::generate_all_posibilities(  std::vector<face_entity_t*> &list,
                                                    int index,
                                                    face_entity_t *root  )
{
    if(index >= list.size())
    {
        return;
    }
    face_entity_t *seed = list[index];

    int yf = 0;
    int yt = 0;

    if(seed->parent == NULL)
    {
        for(int l = 0; l < 2; l ++)
        {
            seed->state.x = 0;
            seed->state.y = 0;
            flip_xy(seed);
            generate_all_posibilities(list, index + 1, root);
        }
        return;
    }
    int junct = face_belong_junct(seed, seed->parent);

    /* right && left */
    for(int l = 0; l < 2; l ++)
    {
        flip_xy(seed);
        if(junct == 0 || junct == 2)
        {
            seed->state.x = junct == 0 ?
                    seed->parent->state.x + seed->parent->state.width :
                    seed->parent->state.x - seed->state.width;

            joints_t jts = seed->parent->ptmplt->joints[junct];

            seed->state.y = seed->parent->state.y + jts.offsets[0];
            if(bpl_box_shape_proc::find_if_conflict(list,
                                seed->state.x, seed->state.y,
                                seed->state.width, seed->state.height, index))
            {
                printf("conflict\n");
                continue;
            }

            if(index + 1 >= list.size())
            {
                on_token_image_callback(root);
            }
            else
            {
                generate_all_posibilities(list, index + 1, root);
            }


        }
        /* up && down */
        else if(junct == 1 || junct == 3)
        {
            int xf = seed->parent->state.x - seed->state.width + 1;
            int xt = seed->parent->state.x + seed->parent->state.width - 1;

            seed->state.y = junct == 1 ?
                    seed->parent->state.y + seed->parent->state.height :
                    seed->parent->state.y - seed->state.height;

            joints_t jts = seed->parent->ptmplt->joints[junct];

            seed->state.x = seed->parent->state.x + jts.offsets[0];
            if(bpl_box_shape_proc::find_if_conflict(list,
                                seed->state.x, seed->state.y,
                                seed->state.width, seed->state.height, index))
            {
                printf("conflict\n");
                continue;
            }

            if(index + 1 >= list.size())
            {
                on_token_image_callback(root);
            }
            else
            {
                generate_all_posibilities(list, index + 1, root);
            }


        }
        seed->state.y = seed->state.x = -9999;
    }
}

void bpl_learner_token::generate_tokens(face_entity_t *root, double threhold)
{
    double thre = 1.0;
    double max_thre = -1.0;
    int maxi = -1;
    int w, h;

    std::vector<face_entity_t*> list;
    recurse_entities_init(root, list);
    generate_all_posibilities(list, 0, root);
}

void bpl_learner_token::set_compare_image(int *dat, int w, int h)
{
    if(m_tender->img)
    {
        free(m_tender->img);
        m_tender->img = (int*) DEBUG_MALLOC(w * h * sizeof(int));
    }
    memcpy(m_tender->img, dat, w * h * sizeof(int));
    m_tender->width = w;
    m_tender->height = h;
}

void bpl_learner_token::set_compare_image(unsigned char *dat, int w, int h)
{
    if(m_tender->img)
    {
        DEBUG_FREE(m_tender->img);
    }
    m_tender->img = (int*) DEBUG_MALLOC(w * h * sizeof(int));
    for(int i = 0; i < w * h; i ++)
    {
        m_tender->img[i] = (int)dat[i];
    }
    m_tender->width = w;
    m_tender->height = h;
}

void bpl_learner_token::learn(const char *file, double threhold)
{
    prm_open_file_read(file);

    face_entity_t *next = prm_getnext_entity();
    int counter = 0;
    while(next != NULL)
    {
        generate_tokens(next, threhold);

        face_tree_destroy(&next);

        next = prm_getnext_entity();

        printf("\n");
        if(counter % 5 == 0)
        {
            DEBUG_MM_DUMP();
        }
        counter ++;
    }
    prm_close_file();
}
