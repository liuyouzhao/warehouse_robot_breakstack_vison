#include "bpl_box_shape_proc.h"

int bpl_box_shape_proc::box_conflict( int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2)
{
    return !(x2 >= x1 + w1 || x1 >= x2 + w2 || y1 >= y2 + h2 || y2 >= y1 + h1);
}

int bpl_box_shape_proc::find_if_conflict( std::vector<face_entity_t*> list, int x, int y, int w, int h, int index )
{
    int conflict = 0;
    for(int i = 0; i < list.size(); i ++)
    {
        if(i == index)
            continue;
        face_entity_t *entity = list[i];
        conflict = box_conflict(entity->state.x, entity->state.y, entity->state.width, entity->state.height,
                     x, y, w, h);
        if(conflict)
        {
            return 1;
        }
    }
    return 0;
}
