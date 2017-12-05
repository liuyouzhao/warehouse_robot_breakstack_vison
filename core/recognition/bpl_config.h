/*
 * bpl_config.h
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_CONFIG_H_
#define CORE_RECOGNITION_BPL_CONFIG_H_

#include "cjson.h"

template <class T>
class bpl_config
{
public:
	bpl_config();
	virtual ~bpl_config();

	void print();
	int set(const char *key, T t);
	T get(const char *key);
private:
	static cJSON *m_pjson;
};

#endif /* CORE_RECOGNITION_BPL_CONFIG_H_ */
