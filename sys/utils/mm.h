/*
 * mm.h
 *
 *  Created on: May 4, 2017
 *      Author: hujia
 */

#ifndef SYS_UTILS_MM_H_
#define SYS_UTILS_MM_H_

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string.h>

typedef struct heap_mem_node_s
{
	heap_mem_node_s(void *address, char *locat, int line, int siz)
	{
		memset(locate, 0, 256);
		sprintf(locate, "%s:%d", locat, line);
		addr = address;
		size = siz;
	}
	void *addr;
	char locate[256];
	int size;
} heap_mem_node_t;


void *__debug_malloc__(int s, const char *file, int line);
void __debug_free__(void *addr);
void __dbg_mm_dump__();

#define DEBUG_MALLOC(s) __debug_malloc__(s, __FILE__, __LINE__)
#define DEBUG_FREE(p) __debug_free__(p)
#define DEBUG_MM_DUMP() __dbg_mm_dump__()

#endif /* SYS_UTILS_MM_H_ */
