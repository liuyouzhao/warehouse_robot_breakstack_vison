#ifndef BPL_TOKEN2IMAGE_H
#define BPL_TOKEN2IMAGE_H

#include "mdl_box.h"

class bpl_token2image
{
public:
    static void token2image(std::vector<face_entity_t*> token_entities, int **img, int *wout, int *hout);
    static void token2image_dbg(std::vector<face_entity_t*> token_entities, int **img, int *wout, int *hout);
};

#endif // BPL_TOKEN2IMAGE_H
