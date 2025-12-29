#include <klib.h>
#include <klib-macros.h>
#include <stdint.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)

size_t strlen(const char *s) {
  size_t result = 0;
  while(*s!='\0')
  {
    result++;
    s++;
  }
  return result;
}

char *strcpy(char *dst, const char *src) {
  char *result = dst;
  do{
    *dst = *src;
    dst++;
  }while(*src++!='\0');
  return result;
}

char *strncpy(char *dst, const char *src, size_t n) {
  char *result = dst;
  while(n>0&&*src!='\0')
  {
    *dst = *src;
    src++;
    dst++;
    n--;
  }
  while(n>0)
  {
    *dst++ = '\0';
    n--;
  }
  return result;
}

char *strcat(char *dst, const char *src) {
  char *result = dst;
  while(*dst!='\0')
  {
    dst++;
  }
  strcpy(dst,src);
  return result;
}

int strcmp(const char *s1, const char *s2) {
  while (*s1 && *s2 && *s1 == *s2) 
  {
    s1++;
    s2++;
  }
  return *(unsigned char *)s1 - *(unsigned char *)s2;
}

int strncmp(const char *s1, const char *s2, size_t n) {
  if(n==0)
    return 0;
  while (*s1 && *s2 && *s1 == *s2&&--n) 
  {
    s1++;
    s2++;
  }
  return *(unsigned char *)s1 - *(unsigned char *)s2;
}

void *memset(void *s, int c, size_t n) {
  unsigned char *p = s;
  while(n--)
  {
    *p = (unsigned char) c;
    p++;
  }
  return s;
}

void *memmove(void *dst, const void *src, size_t n) {
  unsigned char *d = dst;
  const unsigned char *s = src;
  if(d<s)
  {
    while(n--)
    {
      *d++=*s++;
    }
  }
  else
  {
    d+=n;
    s+=n;
    while(n--)
    {
      *--d=*--s;
    }
  }
  return dst;
}

void *memcpy(void *out, const void *in, size_t n) {
    if (!out || !in) return out;
    unsigned char *o = out;
    const unsigned char *i = in;
    while (n--) {
        *o++ = *i++;
    }
    return out;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *a = s1;
    const unsigned char *b = s2;
    for (size_t i = 0; i < n; i++) {
        if (a[i] != b[i]) {
            return (int)a[i] - (int)b[i];
        }
    }
    return 0;
}

#endif
