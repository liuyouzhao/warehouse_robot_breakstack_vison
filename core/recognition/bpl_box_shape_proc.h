#ifndef BPL_BOX_SHAPE_PROC_H
#define BPL_BOX_SHAPE_PROC_H
#include <vector>
#include "mdl_box.h"

class bpl_box_shape_proc
{
public:

    static int box_conflict( int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2);
    static int find_if_conflict( std::vector<face_entity_t*> list, int x, int y, int w, int h, int index );
};

#endif // BPL_BOX_SHAPE_PROC_H
