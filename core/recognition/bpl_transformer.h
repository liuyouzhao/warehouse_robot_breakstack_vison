#ifndef BPL_TRANSFORMER_H
#define BPL_TRANSFORMER_H

#include <vector>
#include "mdl_box.h"

class bpl_transformer
{
public:
    bpl_transformer();

    int transform(face_entity_t *entity);

    /**
     * @brief set_target
     * set target image and info
     * @param img                           bytes pixels(binary)
     * @param x                             x focus point (top priory)
     * @param y                             y focus point (top priory)
     * @param w                             width
     * @param h                             height
     */
    void set_target(char *img, int x, int y, int w, int h);


private:
    char        *m_image;

    int         m_x;
    int         m_y;
    int         m_width;
    int         m_height;
};

#endif // BPL_TRANSFORMER_H
