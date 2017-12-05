#include "bpl_voting.h"

#include <string.h>

#include "jconf.h"
#include "mm.h"
#include "bpl_token2image.h"
#include "bpl_param.h"
#include "bayesian.h"

bpl_voting::bpl_voting( int id,
                        const char *trainfolder,
                        int maxnum )
{
    m_maxnum = maxnum;
    m_id = id;

    m_trainfolder = (char*) DEBUG_MALLOC(256);
    memset(m_trainfolder, 0, 256);
    memcpy(m_trainfolder, trainfolder, strlen(trainfolder));

    m_ts1 = 0;
    m_ts2 = 0;

    m_p_transformer = NULL;

    m_vote_border_rgb = NULL;
    m_show_tmp_result = NULL;
}

bpl_voting::~bpl_voting()
{
    DEBUG_FREE(m_trainfolder);
}

void bpl_voting::setup_entities(cJSON *token, token_probability_t *tp, int num)
{
    tp->entities.clear();
    for(int j = 0; j < num; j ++)
    {
        //face_entity_t *en = (face_entity_t*) DEBUG_MALLOC(sizeof(face_entity_t));
        face_entity_t *en = face_entity_create();

        cJSON *tk = cJSON_GetArrayItem(token, j);

        cJSON *id = cJSON_GetObjectItem(tk, "id");
        cJSON *parent = cJSON_GetObjectItem(tk, "parent");
        cJSON *pjid = cJSON_GetObjectItem(tk, "pjid");
        cJSON *x = cJSON_GetObjectItem(tk, "x");
        cJSON *y = cJSON_GetObjectItem(tk, "y");
        cJSON *w = cJSON_GetObjectItem(tk, "w");
        cJSON *h = cJSON_GetObjectItem(tk, "h");

        for(int i = 0; i < (int) (tp->entities.size()); i ++)
        {
            face_entity_t *fe = tp->entities[i];
            if(strcmp(fe->id, parent->valuestring) == 0)
            {
                en->parent = fe;
                fe->juncts[(int)(pjid->valuedouble)]->pointers.push_back(en);
                break;
            }
        }
        memcpy(en->id, id->valuestring, strlen(id->valuestring));
        en->state.x = x->valuedouble;
        en->state.y = y->valuedouble;
        en->state.width = w->valuedouble;
        en->state.height = h->valuedouble;

        tp->entities.push_back(en);

        if(en->parent == NULL)
        {
            if(en->state.width > en->state.height)
            {
                tp->type = VOTE_HORIZONTAL;
            }
            else
            {
                tp->type = VOTE_VERTICLE;
            }
        }
    }
}

void bpl_voting::unsetup_entities(token_probability_t *tp)
{
    for(int i = 0; i < (int) (tp->entities.size()); i ++)
    {
        face_entity_t *en = tp->entities[i];
        face_entity_destroy(&en);
    }
    tp->entities.clear();
}

/**
  RGB border lines judgement, set areas

  ---------------------------
  |  |------------------ |  |
  |  |                   |  |
  |  |                   |  |
  |  |                   |  |
  |  |                   |  |
  |  |------------------ |  |
  ---------------------------

  */
int bpl_voting::vote_border_lines(face_entity_t *current_entity, std::vector<face_entity_t*> list)
{
    int s = 0;
    if(m_vote_border_rgb)
    {
        s = m_vote_border_rgb(current_entity, list);
    }
    return s;
}

/**
 * @brief vote_recurse
 * Recursively all entities in a given token, after this procedure,
 * token will own a nice score, use which to do judgement.
 * @param token
 * @param root
 * @param voter
 * @param type
 */
void bpl_voting::vote_recurse(token_probability_t *token, face_entity_t *root)
{
    int score1 = 0;
    int max1 = 0;

    score1 = vote_border_lines(root, token->entities);

    if(m_debug)
        printf("score: %d\n", score1);

    /**
      TODO: change here to bayesian functions
      The code below cannot correctly normalize
    */
    token->scores.push_back(score1);

    token->score += score1;
//    /**
//      TODO: Here normalization is not proper
//    */
//    token->score *= 0.5;

    /**
      If score is NOT enough
      */
    if(token->score < VOTE_SERFICENTCY_SCORE)
    {
        for(int i = 0; i < 4; i ++)
        {
            for(int j = 0; root->juncts[i] != NULL &&
                    j < (int) (root->juncts[i]->pointers.size()); j ++)
            {
                face_entity_t *seed = (face_entity_t*) root->juncts[i]->pointers[j];
                vote_recurse(token, seed);
            }
        }
    }
}

void bpl_voting::process_token_probability(token_probability_t *tp)
{
    vote_recurse(tp, tp->entities[0]);

}

void bpl_voting::statistics_probability(int *type1_score, int *type2_score)
{
    *type1_score = 0;
    *type2_score = 0;
    int count1 = 0;
    int count2 = 0;
    int max1 = 0;
    int max2 = 0;
    for(int i = 0; i < (int)(m_tokens.size()); i ++)
    {
        token_probability_t *tp = m_tokens[i];
        switch(tp->type)
        {
        case VOTE_VERTICLE:
            *type1_score += tp->score;
            count1 ++;
            max1 = max1 < tp->score ? tp->score : max1;
            break;
        case VOTE_HORIZONTAL:
            *type2_score += tp->score;
            count2 ++;
            max2 = max2 < tp->score ? tp->score : max2;
            break;
        }

        hot_data_t hd;
        hd.type = tp->type;
        for(int j = 0; j < tp->scores.size(); j ++)
        {
            int s = tp->scores[j];

            hd.event_scores.push_back(s);
        }
        bayesian::append_list(hd);
    }
}

int bpl_voting::run(float &rate, int debug)
{
    m_debug = debug;
    m_tokens.clear();
    for( int i = 0; i < m_maxnum; i ++ )
    {
        jconf jc(m_trainfolder, "token", i);

        cJSON *json = jc.ptrj();

        cJSON *token = cJSON_GetObjectItem(json, "tokens");

        int num = cJSON_GetArraySize(token);

        token_probability_t *tp = new token_probability_t();

        setup_entities(token, tp, num);

        process_token_probability(tp);

        if(m_show_tmp_result)
        {
            m_show_tmp_result(tp->entities);
        }

        if(m_debug)
            printf("[%d]vote: %d => %d\n", i + 1, tp->type, tp->score);

        unsetup_entities(tp);

        /**
         * Save tp without entity objects.
         * Just for the score.
         */
        m_tokens.push_back(tp);
    }
    bayesian::begin();
    statistics_probability(&m_ts1, &m_ts2);
    bayesian::process_sequence_ext(m_debug);
    bayesian::end();

    rate = bayesian::s_best_prob;


    for(int i = 0; i < m_tokens.size(); i ++)
    {
        token_probability_t *tp = m_tokens[i];
        if(tp != NULL)
        {
            delete tp;
        }
    }
    m_tokens.clear();

    return bayesian::s_best_type;
}
