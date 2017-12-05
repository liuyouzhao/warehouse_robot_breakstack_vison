#include "bpl_transformer.h"
#include "bpl_box_shape_proc.h"
#include "bpl_param.h"
#include "mm.h"
#include <string.h>

bpl_transformer::bpl_transformer()
{
    m_image = NULL;
}

void bpl_transformer::set_target(char *img, int x, int y, int w, int h)
{
    m_width = w;
    m_height = h;
    m_x = x;
    m_y = y;

    if(m_image)
    {
        DEBUG_FREE(m_image);
    }

    m_image = (char*) DEBUG_MALLOC(w * h);
    memcpy(m_image, img, w * h);
}


int bpl_transformer::transform(face_entity_t* entity)
{
#if 0
    int max_score = 0;

    if(m_image == NULL)
    {
        return -1;
    }
    /**
      For root
      **/
    face_entity_t *root = entities[0];
    if(root->parent != NULL)
    {
        printf("[Criticel!] %s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
        return -1;
    }

    /**
      For left boundary
      **/
    if(m_x > 0)
    {
        /**
          If root has left node
          **/
        if(root->juncts[JUNC_LEFT] != NULL && root->juncts[JUNC_LEFT]->pointers.size() > 0)
        {
            face_entity_t *seed = root->juncts[JUNC_LEFT]->pointers[0];
            seed->state.x -= m_x;

            int id = -1;
            for(int i = 0; i < entities.size(); i ++)
            {
                face_entity_t *f = entities[i];
                if(strcmp(f->id, seed->id))
                {
                    id = i;
                    break;
                }
            }
            int conflict = bpl_box_shape_proc::find_if_conflict(entities,
                                                                seed->state.x,
                                                                seed->state.y,
                                                                seed->state.width,
                                                                seed->state.height,
                                                                id);
            if(conflict)
            {

            }
        }

        /**
          If root has bottom node
         */
        if(root->juncts[JUNC_BOTTOM] != NULL && root->juncts[JUNC_BOTTOM]->pointers.size() > 0)
        {
            for(int i = 0; i < root->juncts[JUNC_LEFT]->pointers.size(); i ++)
            {
                face_entity_t *seed = root->juncts[JUNC_LEFT]->pointers[0];
                seed->state.x -= m_x;
            }
        }
    }
#endif
    /**
      First, we do not need to transform, just confront with standard full jam
    */
    return 0;
}
