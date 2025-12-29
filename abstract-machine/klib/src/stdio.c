#include <am.h>
#include <klib.h>
#include <klib-macros.h>
#include <stdarg.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)
static int int_to_str(int num, char *buf);
int vsprintf(char *out, const char *fmt, va_list ap);
int vsnprintf(char *out, size_t n, const char *fmt, va_list ap);
static int uint_to_hex(unsigned int num, char *buf) {
    char hex_chars[] = "0123456789abcdef";
    char temp[8]; 
    int i = 0, j = 0;

    if (num == 0) {
        buf[j++] = '0';
        buf[j] = '\0';
        return 1;
    }

    while (num > 0) {
        temp[i++] = hex_chars[num & 0xF];
        num >>= 4;
    }

    while (i > 0) {
        buf[j++] = temp[--i];
    }

    buf[j] = '\0';
    return j;
}
static char sprint_buf[1024];
int printf(const char *fmt, ...) {
  va_list args;
  int n;
  va_start(args, fmt);
  n = vsprintf(sprint_buf, fmt, args);
  va_end(args);
  putstr(sprint_buf);
  return n;
}

int vsprintf(char *out, const char *fmt, va_list ap) {
  char *p = out;
    while (*fmt) {
        if (*fmt != '%') {
            *p++ = *fmt++;
        } else {
            fmt++;
            switch (*fmt++) {
                case 'd': {
                    int val = va_arg(ap, int);
                    char buf[12];
                    int len = int_to_str(val, buf);
                    for (int i = 0; i < len; ++i)
                        *p++ = buf[i];
                    break;
                }
                case 's': {
                    char *s = va_arg(ap, char*);
                    if (s == NULL) s = "(null)";
                    while (*s) {
                        *p++ = *s++;
                    }
                    break;
                }
                case 'x': {
                    unsigned int val = va_arg(ap, unsigned int);
                    char buf[8];
                    int len = uint_to_hex(val, buf);
                    for (int i = 0; i < len; ++i)
                        *p++ = buf[i];
                    break;
                }
                case 'c': {
                   
                    char c = (char)va_arg(ap, int);
                    *p++ = c;
                    break;
                }
                default: {
                    
                    *p++ = '%';
                    *p++ = *(fmt - 1);
                    break;
                }
            }
        }
    }
    *p = '\0'; 
    return p - out;
}

static int int_to_str(int num, char *buf) {
    char temp[12]; 
    int i = 0, j = 0;
    int is_negative = 0;
    if (num == 0) {
        buf[j++] = '0';
        buf[j] = '\0';
        return 1;
    }
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }
    while (num > 0) {
        temp[i++] = '0' + (num % 10);
        num /= 10;
    }
    if (is_negative) {
        buf[j++] = '-';
    }
    while (i > 0) {
        buf[j++] = temp[--i];
    }
    buf[j] = '\0';
    return j;
}

int sprintf(char *out, const char *fmt, ...) {
  va_list ap;
    va_start(ap, fmt);
    int ret = vsprintf(out, fmt, ap);
    va_end(ap);
    return ret;
}

int snprintf(char *out, size_t n, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = vsnprintf(out, n, fmt, ap);
    va_end(ap);
    return ret;
}

int vsnprintf(char *out, size_t n, const char *fmt, va_list ap) {
    if (n == 0) {
        return 0;
    }

    char *p = out;
    const char *chr = fmt;
    int count = 0;

    while (*chr && count < (int)(n - 1)) {
        if (*chr != '%') {
            *p++ = *chr++;
            count++;
        } else {
            chr++;
            if (*chr == '\0') break;

            switch (*chr) {
                case 'd': {
                    int val = va_arg(ap, int);
                    char buf[12];
                    int len = int_to_str(val, buf);
                    for (int i = 0; i < len && count < (int)(n - 1); ++i) {
                        *p++ = buf[i];
                        count++;
                    }
                    break;
                }
                case 's': {
                    char *s = va_arg(ap, char*);
                    if (!s) s = "(null)";
                    while (*s && count < (int)(n - 1)) {
                        *p++ = *s++;
                        count++;
                    }
                    break;
                }
                case 'x': {
                    unsigned int val = va_arg(ap, unsigned int);
                    char buf[8];
                    int len = uint_to_hex(val, buf);
                    for (int i = 0; i < len && count < (int)(n - 1); ++i) {
                        *p++ = buf[i];
                        count++;
                    }
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(ap, int);
                    if (count < (int)(n - 1)) {
                        *p++ = c;
                        count++;
                    }
                    break;
                }
                case 'p': {
                    void *ptr = va_arg(ap, void*);
                    if (count + 2 < (int)n) {
                        *p++ = '0';
                        *p++ = 'x';
                        count += 2;
                        char hex_buf[16];
                        int hex_count = 0;
                        unsigned long num = (unsigned long)ptr;
                        if (num == 0) {
                            if (count < (int)(n - 1)) {
                                *p++ = '0';
                                count++;
                            }
                        } else {
                            const char *digits = "0123456789abcdef";
                            while (num > 0 && count < (int)(n - 1)) {
                                hex_buf[hex_count++] = digits[num % 16];
                                num /= 16;
                            }
                            while (hex_count > 0 && count < (int)(n - 1)) {
                                *p++ = hex_buf[--hex_count];
                                count++;
                            }
                        }
                    }
                    break;
                }
                case '%': {
                    if (count < (int)(n - 1)) {
                        *p++ = '%';
                        count++;
                    }
                    break;
                }
                default: {
                    if (count + 1 < (int)(n - 1)) {
                        *p++ = '%';
                        *p++ = *chr;
                        count += 2;
                    } else {
                        if (count < (int)(n - 1)) {
                            *p++ = '%';
                            count++;
                        }
                    }
                    break;
                }
            }
            chr++;
        }
    }

    if (n > 0) {
        *p = '\0';
    }

    return count; 
}

#endif
