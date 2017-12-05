/*
 * bayeisan.h
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BAYEISAN_H_
#define CORE_RECOGNITION_BAYEISAN_H_

#include <vector>

typedef struct hot_data_s
{
    std::vector<int> event_scores;

    /**
     * @brief cond_prob
     * Conditional probabilistic
     * P(D|ABC) P(C|AB) P(B|A) P(A)
     */
    std::vector<double> cond_prob;
    int type;

    /**
     * @brief result
     */
    double result;
    double help;
} hot_data_t;

class bayesian {
public:
    bayesian();
    virtual ~bayesian();

    static float process_sequence(int debug = 0);
    static float process_sequence_ext(int debug = 0);
    static void append_list(hot_data_t dat);
    static void begin();
    static void end();

    static int s_best_data_id;
    static int s_best_type;
    static double s_best_prob;
private:
    static std::vector< hot_data_t > s_data_form;
};

#endif /* CORE_RECOGNITION_BAYEISAN_H_ */
