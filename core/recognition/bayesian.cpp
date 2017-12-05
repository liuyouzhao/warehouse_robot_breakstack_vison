/*
 * bayeisan.cpp
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#include "bayesian.h"
#include <stdio.h>

std::vector< hot_data_t > bayesian::s_data_form;
int bayesian::s_best_data_id = -1;
int bayesian::s_best_type = -1;
double bayesian::s_best_prob = 0.0f;

bayesian::bayesian() {
	// TODO Auto-generated constructor stub

}

bayesian::~bayesian() {
	// TODO Auto-generated destructor stub
}

float bayesian::process_sequence(int debug)
{
    static char event_names[] = {
        'A',
        'B',
        'C',
        'D',
        'E',
        'F'
    };
    if(s_data_form.size() == 0)
    {
        return 0.0f;
    }
    /* Statistics first event sum value */
    int sum = 0;

    /**
      P(ABCD) = P(D|ABC) * P(C|AB) * P(B|A) * P(A|E)
      */
    for(int i = 0; i < s_data_form.size(); i ++)
    {
        int coop_score = 1;
        for(int j = 0; j < s_data_form[i].event_scores.size(); j ++)
        {
            int s = s_data_form[i].event_scores[j] / 10;
            if(s != 0)
            {
                if(s == 1)
                    coop_score /= 2;
                else
                    coop_score *= s;
            }
        }
        s_data_form[i].result = (double)coop_score;
        sum += coop_score;
    }

    double s_type1_prob = 0.0f;
    double s_type2_prob = 0.0f;
    double max_prob = 0.0f;
    for(int i = 0; i < s_data_form.size(); i ++)
    {
        s_data_form[i].result = s_data_form[i].result / (double)sum;
        if(debug)
        {
            printf("[%d]t-%d->%f\n", i + 1, s_data_form[i].type, s_data_form[i].result);
            for(int j = 0; j < s_data_form[i].event_scores.size(); j ++)
            {
                int prob = s_data_form[i].event_scores[j];
                printf("%c: %d\n", event_names[j], prob);
            }
        }

        if(s_data_form[i].type == 1)
        {
            s_type1_prob = s_type1_prob < s_data_form[i].result ? s_data_form[i].result : s_type1_prob;
        }
        else if(s_data_form[i].type == 2)
        {
            s_type2_prob = s_type2_prob < s_data_form[i].result ? s_data_form[i].result : s_type2_prob;
        }
        if(max_prob < s_data_form[i].result)
        {
            s_best_data_id = i;
        }
    }

    s_best_prob = s_type1_prob > s_type2_prob ? s_type1_prob : s_type2_prob;
    s_best_prob = s_best_prob / (s_type1_prob + s_type2_prob);
    s_best_type = s_type1_prob > s_type2_prob ? 1 : 2;
}

float bayesian::process_sequence_ext(int debug)
{
    static char event_names[] = {
        'A',
        'B',
        'C',
        'D',
        'E',
        'F'
    };
    if(s_data_form.size() == 0)
    {
        return 0.0f;
    }
    hot_data_t first = s_data_form[0];
    int event_siz = first.event_scores.size();
    int siz = s_data_form.size();

    /* Statistics first event sum value */
    int sums[event_siz];
    double whole = 0;

    for(int i = 0; i < event_siz; i ++)
    {
        sums[i] = 0;
    }

    for(int j = 0; j < s_data_form.size(); j ++)
    {
        for(int i = 0; i < s_data_form[j].event_scores.size(); i ++)
        {
            int s = s_data_form[j].event_scores[i];
            sums[i] += s;
            s_data_form[j].help = 0;
            whole += s;
        }
    }

    for(int i = 0; i < s_data_form.size(); i ++)
    {
        s_data_form[i].cond_prob.clear();
        for(int j = 0; j < s_data_form[i].event_scores.size(); j ++)
        {
            int s = s_data_form[i].event_scores[j];
            double prob = (double)s / sums[j];
            if(s == 0)
            {
                prob = 1.0 / (double)siz;
            }

            s_data_form[i].cond_prob.push_back(prob);
            s_data_form[i].help += s_data_form[i].event_scores[j];
        }
        s_data_form[i].help = s_data_form[i].help / whole;
    }

    /**
      P(ABCD) = P(D|ABC) * P(C|AB) * P(B|A) * P(A|E)
      */
    double norm = 0.0f;
    for(int i = 0; i < s_data_form.size(); i ++)
    {
        double prb = 1.0f;
        for(int j = 0; j < s_data_form[i].cond_prob.size(); j ++)
        {
            prb *= s_data_form[i].cond_prob[j];
        }
        s_data_form[i].result = prb;
        norm += prb;
    }

    /**
      Normalize probs
      */
    for(int i = 0; i < s_data_form.size(); i ++)
    {
        s_data_form[i].result /= norm;
        s_data_form[i].result = 0.5 * (s_data_form[i].help + s_data_form[i].result);
    }

    double s_type1_prob = 0.0f;
    double s_type2_prob = 0.0f;
    double max_prob = 0.0f;
    for(int i = 0; i < s_data_form.size(); i ++)
    {
        if(debug)
        {
            printf("[%d]t-%d->%f\n", s_data_form[i].type,
                   i + 1, s_data_form[i].result);
            for(int j = 0; j < s_data_form[i].event_scores.size(); j ++)
            {
                float prob = s_data_form[i].cond_prob[j];
                printf("%c: %f\n", event_names[j], prob);
            }
        }

        if(s_data_form[i].type == 1)
        {
            s_type1_prob += s_data_form[i].result;
        }
        else if(s_data_form[i].type == 2)
        {
            s_type2_prob += s_data_form[i].result;
        }
        if(max_prob < s_data_form[i].result)
        {
            s_best_data_id = i;
        }
    }

    s_best_prob = s_type1_prob > s_type2_prob ? s_type1_prob : s_type2_prob;
    s_best_type = s_type1_prob > s_type2_prob ? 1 : 2;
}

void bayesian::append_list(hot_data_t dat)
{
    bayesian::s_data_form.push_back(dat);
}

void bayesian::begin()
{
    bayesian::s_data_form.clear();
}

void bayesian::end()
{
    bayesian::s_data_form.clear();
}
