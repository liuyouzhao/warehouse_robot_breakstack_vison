/*
 * bpl_param.h
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_PARAM_H_
#define CORE_RECOGNITION_BPL_PARAM_H_

#include "cjson.h"

#define JUNC_RIGHT 0
#define JUNC_BOTTOM 1
#define JUNC_LEFT 2
#define JUNC_TOP 3

#define TRANSFORM_STEP_LEN 5

#define TRANSFORM_MIN_MATCH_EXTENT 1.0

#define VOTE_SERFICENTCY_SCORE 50000000
#define VOTE_BORDERLINE_SCORE_DECAY 0.12
#define VOTE_RGB_TIMES 5

#define VOTE_WEIGHT_DEPTH 0.0
#define VOTE_WEIGHT_RGB 1.0

#define VOTE_ROOT_WEIGHT 3.0

class bpl_param {
public:
	bpl_param();
	virtual ~bpl_param();

	void print();
	double get(const char *key);
	int set(const char *key, double v);
private:

	cJSON *m_pjson;
};

#endif /* CORE_RECOGNITION_BPL_PARAM_H_ */
