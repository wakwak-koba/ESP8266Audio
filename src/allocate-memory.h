#ifndef _ALLOCATE_MEMORY_H_
#define _ALLOCATE_MEMORY_H_

#if defined(ESP32)
  #include <esp_heap_caps.h>

  #define __malloc(s)  heap_caps_malloc(s, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
  
  static void * __calloc(size_t n, size_t size) {
    void * p = __malloc(n * size);
    if(p)
      memset(p, 0, n * size);
    return p;
  }
  
  static void * __realloc(void *old, size_t size) {
    if(!old)
      return __malloc(size);
    
    void * p = __malloc(size);
    if(p) {
      memcpy(p, old, size);
      free(old);
    }
    return p;
  }
#else
  #include <stdlib.h>
  
  #define __malloc(s)  malloc(s)
  #define __calloc     calloc
  #define __realloc    realloc
#endif

#endif