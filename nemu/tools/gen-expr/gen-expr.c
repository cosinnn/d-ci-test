/***************************************************************************************
* Copyright (c) 2014-2024 Zihao Yu, Nanjing University
*
* NEMU is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <string.h>

// this should be enough
static char code_buf[65536 + 128] = {}; // a little larger than `buf`
static char *code_format =
"#include <stdio.h>\n"
"int main() { "
"  unsigned result = %s; "
"  printf(\"%%u\", result); "
"  return 0; "
"}";

uint32_t choose(uint32_t n) {
  return rand() % n;
}

// 动态缓冲区
char *dynamic_buf = NULL;
size_t dynamic_buf_size = 0;
size_t dynamic_buf_used = 0;


void ensure_dynamic_buf_size(size_t needed) {
  if (dynamic_buf_used + needed >= dynamic_buf_size) {
    size_t new_size = dynamic_buf_size == 0 ? 128 : dynamic_buf_size * 2;
    while (new_size < dynamic_buf_used + needed) {
      new_size *= 2;
    }
    char *new_buf = realloc(dynamic_buf, new_size);//返回一个指向新分配的内存块的指针，并且新内存块的大小为new_size
    if (new_buf == NULL) {
      free(dynamic_buf);
      fprintf(stderr, "Memory allocation failed\n");
      exit(1);
    }
    dynamic_buf = new_buf;
    dynamic_buf_size = new_size;
  }
}

// 向动态缓冲区写入字符
void gen(char c) {
  ensure_dynamic_buf_size(2); // 1 字符 + 1 终止符
  dynamic_buf[dynamic_buf_used++] = c;
  dynamic_buf[dynamic_buf_used] = '\0';
}

// 向动态缓冲区写入随机操作符
void gen_rand_op() {
  char ops[4] = {'+', '-', '*', '/'};
  int op = choose(4);
  if (choose(2)) {
    gen(' ');
  }
  gen(ops[op]);
  if (choose(2)) {
    gen(' ');
  }
}

// 向动态缓冲区写入随机数字
void gen_num() {
  int num = choose(100);
  if (choose(2)) {
    gen(' ');
  }
  char num_str[10];//用于存储数字的字符串形式
  int len = snprintf(num_str, sizeof(num_str), "%u", num);//数字储到num_str
  for (int i = 0; i < len; i++) {
    gen(num_str[i]);
  }
  if (choose(2)) {
    gen(' ');
  }
}

void gen_rand_expr() {
  switch (choose(3)) {
  case 0: gen_num(); break;
  case 1: gen('('); gen_rand_expr(); gen(')'); break;
  default: gen_rand_expr(); gen_rand_op(); gen_rand_expr(); break;
  }
}

int main(int argc, char *argv[]) {
  int seed = time(0);
  srand(seed);
  int loop = 1;//默认情况下，循环次数loop被设置为1
  if (argc > 1) {
    sscanf(argv[1], "%d", &loop);//将第一个参数解析为整数并赋值给loop
  }
  int i;
  for (i = 0; i < loop; i++) {
    dynamic_buf_used = 0; // 重置动态缓冲区的使用量
    gen_rand_expr();

    // 将动态缓冲区的内容写入 code_buf
    snprintf(code_buf, sizeof(code_buf), code_format, dynamic_buf);
//code_format,将dynamic_buf中的表达式嵌入到C代码模板中
//snprintf函数将dynamic_buf中的内容格式化后存储到code_buf中
    FILE *fp = fopen("/tmp/.code.c", "w");
    assert(fp != NULL);
    fputs(code_buf, fp);//将code_buf中的内容写入文件
    fclose(fp);

    int ret = system("gcc -Wall -Werror /tmp/.code.c -o /tmp/.expr");
    if (ret != 0) continue;//如果编译失败,跳过循环

    fp = popen("/tmp/.expr", "r");
    assert(fp != NULL);

    int result;
    ret = fscanf(fp, "%d", &result);//使用fscanf从程序的输出中读取一个整数结果
    pclose(fp);

    printf("%u %s\n", result, dynamic_buf);
  }
  free(dynamic_buf); // 释放动态缓冲区
  return 0;
}