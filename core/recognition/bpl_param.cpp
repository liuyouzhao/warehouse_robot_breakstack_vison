/*
 * bpl_param.cpp
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#include <stdio.h>
#include <stdlib.h>
#include "bpl_param.h"

bpl_param::bpl_param() {
	m_pjson = cJSON_CreateObject();
}

bpl_param::~bpl_param() {
	cJSON_Delete(m_pjson);
	m_pjson = NULL;
}

int bpl_param::set(const char *key, double v)
{
	if(key == NULL)
	{
		return -1;
	}
	cJSON_AddItemToObject(m_pjson, key, cJSON_CreateNumber(v));
	return 0;
}

double bpl_param::get(const char *key)
{
	cJSON *vitem = cJSON_GetObjectItem(m_pjson, key);
	if(vitem == NULL)
	{
		return 0xdeadbead;
	}
	return vitem->valuedouble;
}

void bpl_param::print()
{
	char *txt = cJSON_Print(m_pjson);
	printf("%s\n", txt);
	free(txt);
}
