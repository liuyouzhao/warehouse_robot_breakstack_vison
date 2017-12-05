/*
 * bpl_learner_token.h
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_LEARNER_TOKEN_H_
#define CORE_RECOGNITION_BPL_LEARNER_TOKEN_H_

#include <vector>
#include "mdl_box.h"

typedef struct token_s
{
    int id;

    /* root entities list */
    std::vector<face_entity_t*> entities;
    std::vector<double> threholds;
    void *baye;
} token_t;

typedef struct tender_s
{
    int *img;
    int width;
    int height;
} tender_t;


class bpl_learner_token {
public:
    virtual ~bpl_learner_token();

    static bpl_learner_token *create();

    void set_compare_image(int *dat, int w, int h);
    void set_compare_image(unsigned char *dat, int w, int h);
    inline void set_gen_token_callback(void (*cb)(int*,int, int, double), void (*cbt)(token_t *t))
    {
        bpl_token_img_callback = cb;
        bpl_token_tokens_callback = cbt;
    }

    void generate_tokens(face_entity_t *root, double threhold);
    void generate_all_posibilities( std::vector<face_entity_t*> &list,
                                    int index,
                                    face_entity_t *root  );
    void generate_all_posibilities_ext(  std::vector<face_entity_t*> &list,
                                         int index,
                                         face_entity_t *root  );
    void on_token_image_callback(face_entity_t *root);

    std::vector<token_t*> get_best_tokens();

    void learn(const char *file, double threhold);
private:
    bpl_learner_token();
    token_t 	*m_token;
    tender_t 	*m_tender;

    void (*bpl_token_img_callback)(int *data, int w, int h, double result);
    void (*bpl_token_tokens_callback)(token_t *token);
};

#endif /* CORE_RECOGNITION_BPL_LEARNER_TOKEN_H_ */
