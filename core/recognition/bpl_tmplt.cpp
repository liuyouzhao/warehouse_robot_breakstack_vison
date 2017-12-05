/*
 * bpl_tmplt_mngr.cpp
 *
 *  Created on: May 5, 2017
 *      Author: hujia
 */

#include "bpl_tmplt.h"
#include "mm.h"

bpl_tmplt::bpl_tmplt() {
    // TODO Auto-generated constructor stub

}

bpl_tmplt::~bpl_tmplt() {
    // TODO Auto-generated destructor stub
}

/**
 * fake data
 * 20x133
 * 47x125
 * (173-153, 160-27)
 * 148-101, 144-19
 *
 *
 * //98,62---->155,163(57,101)
   //46,93---->100,150(54,57)
 */
int bpl_tmplt::load_templates_from_file(const char *file, int w, int h)
{
    /* fake implement */
#if 1
    for(int i = 0; i < 9; i ++)
    {
        face_template_t *t1x = new face_template_t(w, h, i);
        for(int j = 0; j < 4; j ++)
        {
            t1x->joints[j].jid = j;
            for(int n = 0; n < 32; n ++)
            {
                t1x->joints[j].num = 3;
            }
            if(j == 0 || j == 2)
            {
                t1x->joints[j].offsets[0] = 0;
                t1x->joints[j].offsets[1] = 133/2;
                t1x->joints[j].offsets[2] = 133;
            }
            else if(j == 1 || j == 3)
            {
                t1x->joints[j].offsets[0] = 0;
                t1x->joints[j].offsets[1] = 65/2;
                t1x->joints[j].offsets[2] = 65;
            }
        }

        m_face_templates.push_back(t1x);
    }
#elif 0
    face_template_t *t1x = new face_template_t(20, 133, 0);
    face_template_t *t1y = new face_template_t(20, 133, 1);
    face_template_t *t2x = new face_template_t(54, 125, 2);
    //face_template_t *t2y = new face_template_t(125, 47, 3);

    m_face_templates.clear();
    m_face_templates.push_back(t1x);
    m_face_templates.push_back(t2x);
    m_face_templates.push_back(t1y);
    //m_face_templates.push_back(t2y);
#else
    face_template_t *t1 = new face_template_t(101, 57, 0);
    face_template_t *t2 = new face_template_t(54, 57, 1);
    m_face_templates.clear();
    m_face_templates.push_back(t1);
    m_face_templates.push_back(t2);
#endif
}

void bpl_tmplt::destroy_templates()
{
    for(int i = 0; i < m_face_templates.size(); i ++)
    {
        delete m_face_templates[i];
    }
    m_face_templates.clear();
}


int bpl_tmplt::get_unlocked_template_number()
{
    int counter = 0;
    for(int i = 0; i < m_face_templates.size(); i ++)
    {
        if(m_face_templates[i]->locked == 0)
        {
            counter ++;
        }
    }
    return counter;
}

int bpl_tmplt::has_next_template(std::vector<face_template_t*> vtemps)
{
    for(int i = 0; i < vtemps.size(); i ++)
    {
        if(vtemps[i]->locked == 0)
        {
            return 1;
        }
    }
    return 0;
}

face_template_t *bpl_tmplt::pick_next_template(std::vector<face_template_t*> vtemps)
{
    for(int i = 0; i < vtemps.size(); i ++)
    {
        if(vtemps[i]->locked == 0)
        {
            return (vtemps[i]);
        }
    }
    return NULL;
}

void bpl_tmplt::unlock_all_box_templates(std::vector< face_template_t* > m_face_templates)
{
    for(int i = 0; i < m_face_templates.size(); i ++)
    {
        m_face_templates[i]->locked = 0;
    }
}

void bpl_tmplt::get_ids(int **ids, int *num)
{
    int siz = m_face_templates.size();
    *num = siz;
    *ids = (int*)DEBUG_MALLOC(siz * sizeof(int));
    for(int i = 0; i < siz; i ++)
    {
        (*ids)[i] = m_face_templates[i]->id;
    }
}
