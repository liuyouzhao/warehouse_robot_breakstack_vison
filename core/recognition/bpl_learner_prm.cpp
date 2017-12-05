/*
 * bpl_learner.cpp
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "bpl_learner_prm.h"
#include "mm.h"
#include "jconf.h"

bpl_learner_prm::bpl_learner_prm(const char *path, int id):
m_id(id),
m_number(3)
{
    memset(m_file_name, 0, 128);
    sprintf(m_file_name, "%s/prm.lr%04d.dat", path, m_id);
    m_junction_limits[0] = 2;   m_junction_limits[1] = 2;
    m_junction_limits[2] = 2;   m_junction_limits[3] = 2;
}

bpl_learner_prm::~bpl_learner_prm()
{
}

bpl_learner_prm *bpl_learner_prm::create(const char *path, int id)
{
    bpl_learner_prm *prm = new bpl_learner_prm(path, id);
    return prm;
}

void bpl_learner_prm::destroy(bpl_learner_prm **p)
{
    delete *p;
    *p = NULL;
}

void bpl_learner_prm::set_junctions_limits(int limits[4])
{
    int i = 0;
    for( ; i < 4; i ++ )
    {
        m_junction_limits[i] = limits[i];
    }
}

int bpl_learner_prm::learn()
{
    learn_recursor();
#if DEBUG
    DEBUG_MM_DUMP();
#endif
}

void bpl_learner_prm::push_junctions(prm_t *face, std::vector<junction_t*> &junctions)
{
    for(int i = 0; i < 4; i ++)
    {
        junction_t *p = new junction_t();
        p->owner = face;
        p->pointers.clear();
        p->jid = i;
        junctions.push_back(p);
    }
}

/**
* bin: 1111 means all, 1000 only junction-1
*/
void bpl_learner_prm::push_junctions_excepts(prm_t *face,
                                             std::vector<junction_t*> &junctions, int bin)
{
    int n = 0;
    for(int i = 0; i < 4; i ++)
    {
        junction_t *p = new junction_t();
        p->owner = face;
        p->pointers.clear();
        n = (8 >> i);
        if(bin & n)
        {
            p->jid = i;
        }
        else
        {
            p->jid = JR_INVALID_JUNCTION;
        }
        junctions.push_back(p);
    }
}

void bpl_learner_prm::pop_junctions(std::vector<junction_t*> &junctions)
{
    for(int i = 0; i < 4; i ++)
    {
        int index = junctions.size() - 1 - i;
        delete junctions[index];
    }
    for(int i = 0; i < 4; i ++)
    {
        junctions.pop_back();
    }
}

void bpl_learner_prm::relate_recurse( std::vector<junction_t*> &junctions,
                                      prm_t *prm_node,
                                      std::vector<prm_t*> &order,
                                      int layer )
{
    prm_t *next = NULL;
    int n = 0;

    if(junctions.size() == 0)
    {
        /**
         * @brief push_junctions_excepts
         * If this is the first node of all,
         * we focus on top-left point, top prior, so we cut out the top junction.
         */
        push_junctions_excepts(prm_node, junctions, JR_WITHOUT_TOP);
    }

    /**
      Get the next possible node
    */
    next = order[layer];

    /**
      For each junction in current possible junctions
     **/
    for( n = 0; n < junctions.size(); n ++)
    {
        junction_t *p = junctions[n];

        if( p->jid == JR_INVALID_JUNCTION )
        {
            continue;
        }
        next->deeps[p->jid] = ((prm_t*)(p->owner))->deeps[p->jid] + 1;

        /**
         Judge if deep is output limit, or
         junction's jid is -1, means invalidate, we continue.
        **/
        if( next->deeps[p->jid] >= m_junction_limits[p->jid] )
        {
            continue;
        }

        p->pointers.push_back(next);

        push_junctions(next, junctions);

        if(layer + 1 < order.size())
        {
            relate_recurse(junctions, next, order, layer + 1);
        }
        else
        {
            for(int i = 0; i < junctions.size(); i ++)
            {
                for( int j = 0; j < junctions[i]->pointers.size(); j ++)
                {
#if DEBUG
                    std::cout << junctions[i]->jid << "[" <<
                                 ((prm_t*)(junctions[i]->owner))->id << "]" << "->" <<
                                 ((prm_t*)(junctions[i]->pointers[j]))->id << std::endl;
#endif
                    if(((prm_t*)(junctions[i]->owner))->id == ((prm_t*)(junctions[i]->pointers[j]))->id)
                    {
                        printf("error\n");
                    }
                    __write_learning_data( junctions[i]->jid,
                                           ((prm_t*)(junctions[i]->owner))->id,
                                           ((prm_t*)(junctions[i]->pointers[j]))->id );
                }
            }
#if DEBUG
            printf("\n");
#endif
            prm_open_file(m_file_name);
            prm_begin_append();
            prm_end_append();
            prm_close_file();
        }

        pop_junctions(junctions);
        std::vector<void*>::iterator iter = p->pointers.begin();
        while(1)
        {
            if((*iter) == next)
            {
                p->pointers.erase(iter);
                break;
            }
            iter ++;
        }
    }
}

/*
 -----------
 | C |
 | C |
 | C |
 | B |
 | B |
 | B |
 | A |      |C|
 | A |      |B|
 | A |      |A|
 * */

int bpl_learner_prm::learn_recursor()
{
    int i = 0;
    std::vector<prm_t*> prm_nodes;
    prm_t *p_prm_node;

    for( i = 0; i < m_number; i ++ )
    {
        p_prm_node = (prm_t*) DEBUG_MALLOC(sizeof(prm_t));
        p_prm_node->id = i;
        memset(p_prm_node->deeps, 0, 4 * sizeof(int));
        prm_nodes.push_back(p_prm_node);
    }

    std::vector<junction_t*> junctions;
    relate_recurse(junctions, prm_nodes[0], prm_nodes, 1);

    for(int j = 0; j < junctions.size(); j ++)
    {
        DEBUG_FREE(junctions[j]);
    }
    junctions.clear();

    for( i = 0; i < m_number; i ++ )
    {
        DEBUG_FREE(prm_nodes[i]);
    }
    prm_nodes.clear();

    return 0;
}

int bpl_learner_prm::__write_learning_data(int jid, int id, int tid)
{
    prm_open_file(m_file_name);
    prm_append(id, tid, jid);
    prm_close_file();
}
