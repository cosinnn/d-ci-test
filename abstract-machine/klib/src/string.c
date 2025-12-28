#include <klib.h>
#include <klib-macros.h>
#include <stdint.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)

size_t strlen(const char *s) {
  const char *p = s;
  while(*p)
  {
    p++;
  }
  return p-s;
}

char *strcpy(char *dst, const char *src) {
  const char *p = src;
  char *q = dst;
  while((*dst++ = *p++));    
  return q;
}

char *strncpy(char *dst, const char *src, size_t n) {
  const char *p = src;
  char *q = dst;
  while(n >0 && *p!='\0')
  {
    *dst++ = *p++;
    n--;
  }
  while(n>0)
  {
    *dst++ = '\0';
    n--;
  }
  return q;
}

char *strcat(char *dst, const char *src) {
  const char *p = src;
  char *q = dst;
  while(*dst !='\0')
  {
    dst ++;
  }
  while((*dst++ = *p++));
  return q;
}

int strcmp(const char *s1, const char *s2) {
  while(*s1!='\0' && *s2!='\0')
  {
    if(*s1 != *s2)
    {
      return *s1 - *s2;
    }
    s1++;
    s2++;
  }
  return *s1 - *s2;
}

int strncmp(const char *s1, const char *s2, size_t n) {
  while( *s1 !='\0'&& *s2 !='\0' && n>0)
  {
    if(*s1 != *s2)
    {
      return *s1-*s2;
    }
    s1++;
    s2++;
    n--;
  }
  return *s1 - *s2;
}

void *memset(void *s, int c, size_t n) {
  unsigned char *p = (unsigned char *)s;
  unsigned char q = (unsigned char)c;
  while(n>0)
  {
    *p = q;
    p++;
    n--;
  }
  return s;
}

void *memmove(void *dst, const void *src, size_t n) {
  unsigned char *p = (unsigned char *)dst;
  unsigned char *q = (unsigned char *)src;
  if(p < q)
  {
    while(n>0)
    {
    *p++ = *q++;
    n--;
    }
  }
  else
  {
    p+=n-1;
    q+=n-1;
    while(n>0)
    {
      *p-- = *q--;
      n--;
    }
  }
  return dst;
}

void *memcpy(void *out, const void *in, size_t n) {
  unsigned char *p = (unsigned char *)out;
  unsigned char *q = (unsigned char *)in;
  while(n--)
  {
    *p++ = *q++;
  }
  return out;
}

int memcmp(const void *s1, const void *s2, size_t n) {
  const unsigned char *p = (unsigned char *)s1;
  const unsigned char *q = (unsigned char *)s2;  
  while(n>0)
  {
    if(*p!=*q)
    {
      return *p - *q;
    }
    p++;
    q++;
    n--;
  }
  return 0;
}

#endif
