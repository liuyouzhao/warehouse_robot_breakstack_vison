#include "bpl_token2image.h"
#include "mm.h"

static int *genimage(std::vector<face_entity_t*> entities, int &wout, int &hout, int dbg)
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
                if(dbg)
                    img[l] = 255 * (i + 1) / entities.size();
                else
                    img[l] = 255;
            }
        }
    }
    wout = width;
    hout = height;
    return img;
}

void bpl_token2image::token2image(std::vector<face_entity_t *> token_entities, int **img, int *wout, int *hout)
{
    int w = 0;
    int h = 0;
    int *imgtmp = genimage(token_entities, w, h, 0);
    *img = imgtmp;
    *wout = w;
    *hout = h;
}

void bpl_token2image::token2image_dbg(std::vector<face_entity_t*> token_entities, int **img, int *wout, int *hout)
{
    int w = 0;
    int h = 0;
    int *imgtmp = genimage(token_entities, w, h, 1);
    *img = imgtmp;
    *wout = w;
    *hout = h;
}
