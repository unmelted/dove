
#include "os_api.hpp"
#include <stdio.h>
//#include <execinfo.h>
#include <map>
#include <mutex>
#ifdef MEM_DEBUG
long long current_size;
long long peak_size;

using namespace std;

typedef struct _MemInfo
{
    void *p;
    const char *file;
    const char *func;
    int line;
    size_t size;
} MemInfo;

mutex os_mt;
map<long long,MemInfo *> m;
void meminfo()
{
    os_mt.lock();
#if 1
    for(auto it = m.begin(); it != m.end(); it++){
        MemInfo *info = it->second;
        printf("%s %s %d (%ld) %p\n",info->file, info->func,info->line,info->size,info->p);
	}
#endif

    printf("ramained map size = %lu\n",m.size());
    printf("peak size = %lld\n",peak_size);
    printf("current size = %lld\n",current_size);
    os_mt.unlock();
}

void *_g_os_malloc(size_t size, const char *pfile, const char *pfunc, const int line)
{
    void *p = NULL;
	p = (void *)malloc(size);

	if( p != NULL )
	{
        os_mt.lock();
        MemInfo *new_info = (MemInfo *)malloc( sizeof( MemInfo ));
        new_info->p = p;
        new_info->file = pfile;
        new_info->func = pfunc;
        new_info->line = line;
        new_info->size = size;

        current_size += size;
        if(current_size > peak_size) peak_size = current_size;


        m.insert(make_pair((long long)p,new_info));
        os_mt.unlock();
        return p;
	}

	return NULL;
}

void *_g_os_calloc(size_t ecnt, size_t esize, const char *pfile, const char *pfunc, const int line)
{
    void *p = NULL;
    size_t size = ecnt*esize;
	p = (void *)calloc(ecnt,esize);

	if( p != NULL )
	{
        os_mt.lock();
        MemInfo *new_info = (MemInfo *)malloc( sizeof( MemInfo ));
        new_info->p = p;
        new_info->file = pfile;
        new_info->func = pfunc;
        new_info->line = line;
        new_info->size = size;

        current_size += size;
        if(current_size > peak_size) peak_size = current_size;


        m.insert(make_pair((long long)p,new_info));
        os_mt.unlock();
        return p;
	}

	return NULL;
}

void g_os_free(void *p)
{
//    printf(" !! %p ", p);

    os_mt.lock();
    map<long long,MemInfo *>::iterator iter;
    iter = m.find((long long )p);
    //printf("free iter %p %p (%p)\n",iter,m.end(),p);
    if(iter != m.end())
    {
        MemInfo *info =iter->second;
        current_size -= info->size;
        free(info);


        m.erase(m.find((long long )p));
    }
    os_mt.unlock();
    free(p);
}
#else

void *g_os_malloc(size_t size)
{
    return malloc(size);
}
void *g_os_calloc(size_t ecnt, size_t esize){
    return calloc(ecnt,esize);
}
void g_os_free(void *p)
{
    free(p);
}
#endif
