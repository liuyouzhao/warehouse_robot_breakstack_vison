/*
 * bpl_config.cpp
 *
 *  Created on: May 8, 2017
 *      Author: hujia
 */

#include "bpl_config.h"
#include <typeinfo>
#include <stdio.h>
#include <stdlib.h>

template <class T>
bpl_config<T>::bpl_config() {
	m_pjson = cJSON_CreateObject();
}

template<class T>
bpl_config<T>::~bpl_config() {
	cJSON_Delete(m_pjson);
	m_pjson = NULL;
}

template<class T>
T bpl_config<T>::get(const char *key)
{
	cJSON *vitem = cJSON_GetObjectItem(m_pjson, key);
	if(vitem == NULL)
	{
		return 0xdeadbead;
	}

	if(vitem->type == cJSON_Number)
		return vitem->valuedouble;
	else if(vitem->type == cJSON_String)
		return vitem->valuestring;
}

template<class T>
int bpl_config<T>::set(const char *key, T v)
{
	if(key == NULL)
	{
		return -1;
	}
	if(typeid(v) == typeid(int) || typeid(v) == typeid(double))
	{
		cJSON_AddItemToObject(m_pjson, key, cJSON_CreateNumber(v));
	}
	else if(typeid(v) == typeid(char*))
	{
		cJSON_AddItemToObject(m_pjson, key, cJSON_CreateString(v));
	}
	return 0;
}

template<class T>
void bpl_config<T>::print()
{
	char *txt = cJSON_Print(m_pjson);
	printf("%s\n", txt);
	free(txt);
}
