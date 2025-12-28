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

#include <isa.h>
#include <cpu/cpu.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "sdb.h"
#include "memory/paddr.h"

static int is_batch_mode = false;

void init_regex();
void init_wp_pool();

/* We use the `readline' library to provide more flexibility to read from stdin. */
static char* rl_gets() {
  static char *line_read = NULL;//line_read：这是一个指向字符串的指针，用于存储用户输入的行

  if (line_read) {
    free(line_read);
    line_read = NULL;
  }

  line_read = readline("(nemu) ");//用于处理用户输入，支持自动补全、历史记录等
//readline 函数会读取用户输入的一行，并返回一个指向动态分配的字符串的指针
//如果用户直接按回车键（没有输入任何内容），line_read 可能为 NULL。
  if (line_read && *line_read) {
    add_history(line_read);//将输入的行添加到历史记录
  }

  return line_read;
}

static int cmd_c(char *args) {
  cpu_exec(-1);
  return 0;
}


static int cmd_q(char *args) {
  nemu_state.state = NEMU_QUIT;
  return -1;
}

static int cmd_help(char *args);
static int cmd_si(char *args){
  char *arg = strtok(NULL, " ");
  int i = 0;
  if(arg == NULL)
	{
		cpu_exec(1);
	}
  else 
  {
    i = strtol(arg,NULL,10);
    cpu_exec(i);   
  }
  return 0;
}
void display_wp();
static int cmd_info(char *args){
  char *arg = strtok(NULL, " ");
  if(strcmp(arg,"r") == 0){
    isa_reg_display();
    
  }
  else if( strcmp(arg,"w")==0)
  {
    display_wp();
    
  }
  return 0;
}
static int cmd_x(char *args){
  char *argone = strtok(NULL," ");
  char *argtwo = strtok(NULL," ");
  int i = strtol(argone,NULL,10);
  uint32_t address = strtol(argtwo,NULL,16);
  
  for(int j = 0;j<i;j++)
  {
    printf("0x%08x: 0x%08x\n",address+j*4,paddr_read(address+j*4,4));
  }
  return 0;
}
word_t expr(char *e, bool *success);
static int cmd_p(char *args){
  // char *arg = strtok(args," ");
  if(args ==NULL)
  {
    printf("empty exprerssion\n");
  }
  else {
    bool success = false;
    uint32_t result = expr(args,&success);
    if(success)
    {
      printf("result = %u\n",result);
    }
    else {
      printf("Error\n");
    }
  }
  return 0;
}
static int cmd_ext(char *args){//const 修饰的是 char，表示指针所指向的内容（字符）是不能被修改的。
  const char *input_file_path = "/home/huang/ysyx-workbench/nemu/tools/gen-expr/input";
  FILE *input_file = fopen(input_file_path,"r");
  if(input_file == NULL)
  {
    perror("Failed to open input file");
    return -1;
  }
  char line[4096];//定义了一个字符数组 line用于存储从文件中读取的每一行
  int all_test = 0,past_test = 0;
  while(fgets(line,sizeof(line),input_file)){//fgets 函数逐行读取文件内容，每次读取一行存储到 line 中
    char *expected_result_str = strtok(line," ");
    char *expr_str = strtok(NULL,"\n");
    if (expected_result_str == NULL || expr_str == NULL) {
      fprintf(stderr, "Invalid test case format: %s", line);
      continue;
    }//stderr 是C语言标准库中的一个宏，表示标准错误流
    int expected_result = atoi(expected_result_str);//使用 atoi 函数将预期结果从字符串转换为整数。
    bool success = false;
    word_t result = expr(expr_str,&success);
    all_test ++;
    if(success && result == expected_result)
    {
      past_test ++;
    }    
  }
  printf("%d test,%d passed\n",all_test,past_test);
  fclose(input_file);
  return 0;
}
void set_wp(char *arg, word_t value);
void delete_wp(int n);
static int cmd_w(char *args)
{
  if(args == NULL)
  {
    printf("Unknown input, the standard format is 'w EXPR'\n");
    return 0;
  }
  bool success;
  word_t res = expr(args, &success);
  if(!success)
    printf("The expression is problematic\n");
  else 
    set_wp(args, res);
  
  return 0;
}
static int cmd_d(char *args)
{
  if(args == NULL)
  {
    printf("Unknown input, the standard format is 'd N'\n");
    return 0;
  }
  char *arg = strtok(NULL, " ");
  int n = strtol(arg, NULL, 10);
  delete_wp(n);
  return 0;
}
static struct {
  const char *name;
  const char *description;
  int (*handler) (char *);//指向处理函数的指针。这个函数接受一个字符串参数 char *，并返回一个整数。
} cmd_table [] = {
  { "help", "Display information about all supported commands", cmd_help },
  { "c", "Continue the execution of the program", cmd_c },
  { "q", "Exit NEMU", cmd_q },

  /* TODO: Add more commands */
  { "si", "Execute the program step by step and pause after executing N instructions. If N is not specified, the default value is 1",cmd_si},
  { "info", "Print the program state", cmd_info},
  { "x", "scanf men", cmd_x},
  { "p", "expression evaluation",cmd_p},
  { "ext","test",cmd_ext},
  { "w","watchpoint",cmd_w},
  { "d","deletepoint",cmd_d},
};

#define NR_CMD ARRLEN(cmd_table)//ARRLEN 是一个宏，用于计算数组的长度

static int cmd_help(char *args) {
  /* extract the first argument */
  char *arg = strtok(NULL, " ");
  int i;

  if (arg == NULL) {
    /* no argument given */
    for (i = 0; i < NR_CMD; i ++) {
      printf("%s - %s\n", cmd_table[i].name, cmd_table[i].description);
    }
  }
  else {
    for (i = 0; i < NR_CMD; i ++) {
      if (strcmp(arg, cmd_table[i].name) == 0) {
        printf("%s - %s\n", cmd_table[i].name, cmd_table[i].description);
        return 0;
      }
    }
    printf("Unknown command '%s'\n", arg);
  }
  return 0;
}

void sdb_set_batch_mode() {
  is_batch_mode = true;
}

void sdb_mainloop() {
  if (is_batch_mode) {
    cmd_c(NULL);
    return;
  }
//如果程序处于批处理模式，则调用 cmd_c(NULL) 函数并退出主循环。不是与用户交互
  for (char *str; (str = rl_gets()) != NULL; ) {
    char *str_end = str + strlen(str);
//rl_gets()会返回一个指向字符串的指针,如果读取失败（例如输入流结束），它会返回NULL
    /* extract the first token as the command */
    char *cmd = strtok(str, " ");
    if (cmd == NULL) { continue; }

    /* treat the remaining string as the arguments,
     * which may need further parsing
     */
    char *args = cmd + strlen(cmd) + 1;//命令行参数 
    if (args >= str_end) {
      args = NULL;
    }

#ifdef CONFIG_DEVICE
    extern void sdl_clear_event_queue();
    sdl_clear_event_queue();
#endif

    int i;
    for (i = 0; i < NR_CMD; i ++) {
      if (strcmp(cmd, cmd_table[i].name) == 0) {
        if (cmd_table[i].handler(args) < 0) { return; }
        break;
      }
    }

    if (i == NR_CMD) { printf("Unknown command '%s'\n", cmd); }
  }
}

void init_sdb() {
  /* Compile the regular expressions. */
  init_regex();
  /* Initialize the watchpoint pool. */
  init_wp_pool();
}
