/*
 * mm.cpp
 *
 *  Created on: May 4, 2017
 *      Author: hujia
 */

#include <stdio.h>
#include <vector>
#include <string.h>
#include "mm.h"

static std::vector< heap_mem_node_t* > s_heap_ref;


void *__debug_malloc__(int s, const char *file, int line)
{
	void *p = malloc(s);
	heap_mem_node_t *t = new heap_mem_node_t(p, (char*)file, line, s);
	s_heap_ref.push_back(t);
	return p;
}

void __debug_free__(void *addr)
{
	int find = 0;
	std::vector< heap_mem_node_t* >::iterator iter = s_heap_ref.begin();
	for(; iter != s_heap_ref.end(); iter ++ )
	{
		if( (*iter)->addr == addr )
		{
			delete *iter;
			s_heap_ref.erase(iter);
			find = 1;
			break;
		}
	}
	free(addr);
}

void __dbg_mm_dump__()
{
	std::vector< heap_mem_node_t* >::iterator iter = s_heap_ref.begin();
	for(; iter != s_heap_ref.end(); iter ++ )
	{
		printf("{%p}[%dBytes]%s\n", (*iter)->addr, (*iter)->size, (*iter)->locate);
	}
}
