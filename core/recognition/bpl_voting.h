#ifndef BPL_VOTING_H
#define BPL_VOTING_H

#include <vector>
#include "cjson.h"
#include "bpl_learner_token.h"
#include "bpl_transformer.h"
#include "mdl_box.h"

/**
 * @struct token_probability_t
 * Token evaluation struct,
 * be used to do voting.
 */
typedef struct token_probability_s
{
    /**
     * @brief type
     * The active or positive type.
     * In our issue, stands for rectangle direction
     */
    int type;

    /**
     * @brief entities
     * Corresponding entities, read from files
     */
    std::vector<face_entity_t*> entities;

    /**
     * @brief score
     * The best score of this token, compare with test data.
     */
    std::vector<int> scores;
    int score;
} token_probability_t;

enum en_vote_type
{
    VOTE_VERTICLE = 1,
    VOTE_HORIZONTAL = 2
};

/**
 * @brief The bpl_voting class
 */
class bpl_voting
{
public:
    /**
     * @brief bpl_voting                main construction method
     * @param id                        index for trainning data association
     * @param trainfolder               explicately figure out train-data-folder.
     * @param outfolder                 output result folder
     * @param maxnum                    the token files' max number
     */
    bpl_voting(int id, const char *trainfolder, int maxnum);
    ~bpl_voting();

    /**
     * @brief set_img_compare_callback
     * @param cb                        callback pointer
     */
    inline void set_vote_border_rgb_callback(int (*cb)(face_entity_t* entity, std::vector<face_entity_t*> list))
    {   m_vote_border_rgb = cb; }
    inline void set_show_result_callback(int (*cb)(std::vector<face_entity_t*> list))
    {   m_show_tmp_result = cb; }

    /**
     * @brief run
     * Run voting procedure
     *
     * @return
     * Success 0, Failed or any error -1
     */
    int run(float &rate, int debug = 0);

    /**
     * @brief get_type1_score, get_type2_score
     * type score1, type score2
     * 1, 2 stand for box-face directions
     * 1 is verticle, 2 is horizontal
     * return m_ts1, m_ts2
     */
    inline int get_type1_score()    {   return m_ts1;   }
    inline int get_type2_score()    {   return m_ts2;   }
    inline void set_type1_score(int ts1)    {    m_ts1 = ts1;   }
    inline void set_type2_score(int ts2)    {    m_ts2 = ts2;   }

protected:
    void setup_entities(cJSON *token, token_probability_t *tp, int num);
    void unsetup_entities(token_probability_t *tp);
    void process_token_probability(token_probability_t *tp);
    void statistics_probability(int *type1_score, int *type2_score);
    int vote_border_lines(face_entity_t *current_entity, std::vector<face_entity_t*> list);
    void vote_recurse(token_probability_t *token, face_entity_t *root);

private:
    bpl_transformer                     *m_p_transformer;
    std::vector<token_probability_t*>   m_tokens;
    char                                *m_trainfolder;
    int                                 m_id;
    int                                 m_maxnum;

    int                                 m_ts1;
    int                                 m_ts2;

    int (*m_vote_border_rgb) (face_entity_t* entity, std::vector<face_entity_t*> list);
    int (*m_show_tmp_result) (std::vector<face_entity_t*> list);

    int                                 m_debug;
};

#endif // BPL_VOTING_H
