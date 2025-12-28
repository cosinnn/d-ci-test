#include <am.h>
#include <klib.h>
#include <klib-macros.h>
#include <stdarg.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)
static char buf[1024];
int printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap,fmt);
  int n = vsprintf(buf,fmt,ap);
  va_end(ap);
  for(int i =0;i <n;i++)
  {
    putch(buf[i]);
  }
  return n;
}

int vsprintf(char *out, const char *fmt, va_list ap) {
  char *start = out;
  while(*fmt)
  {
    if(*fmt == '%')
    {
      fmt ++;
      if(*fmt == 's')
      {
        char *str = va_arg(ap, char*);
        while(*str)
        {
          *out++ = *str++;
        }
        fmt++;
      }else if(*fmt == 'd')
      {
        int num = va_arg(ap,int);
        char buffer[32];
        int i = 0,is_negative = 0;
        if(num < 0)
        {
          is_negative = 1;
          num = -num;
        }
        do{
          buffer[i++] = (num%10) + '0';
          num/=10;
        }while(num);
        if(is_negative) buffer[i++] = '-';
        while(i--)
        {
          *out++ =buffer[i];
        }
        fmt++;
      }else{
        *out++ = '%';
        if(*fmt) *out++ = *fmt++;
      }      
    }else{
      *out ++ = *fmt++;
    }
  }
  *out = '\0';
  return out - start;
}

int sprintf(char *out, const char *fmt, ...) {
  va_list ap;
  va_start(ap,fmt);
  int n = vsprintf(out,fmt,ap);
  va_end(ap);
  return n;
}

int snprintf(char *out, size_t n, const char *fmt, ...) {
  va_list ap;
  va_start(ap,fmt);
  int ret = vsnprintf(out,n,fmt,ap);
  va_end(ap);
  return ret;
}

int vsnprintf(char *out, size_t n, const char *fmt, va_list ap) {
  char *start = out;
  size_t left = n;
  while(*fmt)
  {
    if(*fmt == '%')
    {
      fmt++;
      if(*fmt == 's')
      {
        char *str = va_arg(ap,char*);
        while(*str && left >1)
        {
          *out++ = *str++;
          left --;
        }
        fmt++;
      }else if(*fmt == 'd')
      {
        int num = va_arg(ap,int);
        char buffer[32];
        int i=0,is_negative =0;
        if(num<0){
          is_negative = 1;
          num =-num;
        }
        do{
          buffer[i++] = (num %10) +'0';
          num/=10;
        }while(num);
        if(is_negative) buffer[i++] ='-';
        while(i--&&left>1){
          *out++ = buffer[i];
          left --;
        }
        fmt ++;
      }else{
        if(left > 1){
          *out++ ='%';
          left--;
        }
        if(*fmt&&left>1){
          *out++ = *fmt++;
          left--;
        }
      }
    }else {
      if(left >1){
        *out++ = *fmt++;
        left --;
      }else{
        fmt++;
      }
    }
  }
  if(left >0){
    *out = '\0';
  }else if(n >0){
    out[-1] = '\0';
  }
  return out -start;
}

#endif
