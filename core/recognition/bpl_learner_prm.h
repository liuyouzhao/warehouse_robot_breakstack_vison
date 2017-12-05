/*
 * bpl_learner.h
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_LEARNER_PRM_H_
#define CORE_RECOGNITION_BPL_LEARNER_PRM_H_

#include <vector>
#include "mdl_box.h"
#include "cjson.h"


/**
 * @brief The en_junction_rule enum
 *           []0001
 *           |
 * 0010[]<---[] --->[] 1000
 *           |
 *           []0100
 */
enum en_junction_rule
{
    JR_WITHOUT_TOP = 14,
    JR_WITHOUT_BOTTOM = 11,
    JR_WITHOUT_LEFT = 13,
    JR_WITHOUT_RIGHT = 7,

    JR_INVALID_JUNCTION = -1
};

typedef struct prm_s
{
    int id;
    int deeps[4];
} prm_t;

class bpl_tmplt;
class bpl_learner_prm {
public:
    static bpl_learner_prm *create(const char *path, int id);
    static void destroy(bpl_learner_prm **p);

    virtual ~bpl_learner_prm();

    /** Extract API
     * @brief learn                     user interface learn function
     * @return
     */
    int learn();

    /** Extract API
     * @brief file                      get current corresponding file
     * @return
     */
    inline char *file()	{	return m_file_name;	}

    /**
     * @brief set_number                set how many nodes involved by this prm
     * @param num
     */
    inline void set_number(int num) {   m_number = num; }

    void set_junctions_limits(int limits[4]);

protected:

    /**
     * @brief learn_recursor            main learning function
     * @return
     */
    int learn_recursor();

    /**
     * @brief push_junctions
     * @param node                      current prm node
     * @param junctions                 all valid empty junctions
     */
    void push_junctions(prm_t *node, std::vector<junction_t*> &junctions);

    /**
     * @brief push_junctions_excepts
     * @param node                      current prm node
     * @param junctions                 all valid empty junctions
     * @param bin                       1111 means all, 1000 only junction-1
     */
    void push_junctions_excepts(prm_t *node, std::vector<junction_t*> &junctions, int bin);

    /**
     * @brief pop_junctions
     * @param junctions                 pop last pushed junctions
     */
    void pop_junctions(std::vector<junction_t*> &junctions);

    /**
     * @brief relate_recurse            learning function recursively
     * @param junctions
     * @param face
     * @param order
     * @param layer
     */
    void relate_recurse( std::vector<junction_t*> &junctions,
                         prm_t *face,
                         std::vector<prm_t*> &order,
                         int layer );

private:
    bpl_learner_prm(const char *path, int id);
    int __write_learning_data(int jid, int id, int tid);

    std::vector<junction_t*> 	m_junctions;
    char 						m_file_name[128];
    int 						m_id;
    int                         m_number;
    int                         m_junction_limits[4];
};

#endif /* CORE_RECOGNITION_BPL_LEARNER_PRM_H_ */
