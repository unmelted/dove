#pragma once
#include <stdlib.h>

//#define CLASS_DEBUG
#define MEM_DEBUG

#ifdef MEM_DEBUG

void meminfo();
void *_g_os_malloc(size_t size, const char *pfile, const char *pfunc, const int line);
void *_g_os_calloc(size_t ecnt, size_t esize, const char *pfile, const char *pfunc, const int line);
void g_os_free(void *p);


#define g_os_malloc(x) _g_os_malloc(x, __FILE__, __func__, __LINE__ )
#define g_os_calloc(x,y) _g_os_calloc(x,y, __FILE__, __func__, __LINE__ )

#else
void *g_os_malloc(size_t size);
void *g_os_calloc(size_t ecnt, size_t esize);
void g_os_free(void *p);
#endif
