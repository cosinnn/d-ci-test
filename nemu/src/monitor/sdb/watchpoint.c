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

#include "sdb.h"

#define NR_WP 32

typedef struct watchpoint {
  int NO;
  struct watchpoint *next;

  /* TODO: Add more members if necessary */
  word_t old_value;
  char expr[100];  
} WP;
#ifndef CONFIG_WATCHPOINT
void init_wp_pool() {}
void set_wp(char *arg, word_t value) {}
void delete_wp(int n) {}
void display_wp() {}
void scan_wp() {}
#endif
#ifdef CONFIG_WATCHPOINT
static WP wp_pool[NR_WP] = {};
static WP *head = NULL, *free_ = NULL;

void init_wp_pool() {
  int i;
  for (i = 0; i < NR_WP; i ++) {
    wp_pool[i].NO = i;
    wp_pool[i].next = (i == NR_WP - 1 ? NULL : &wp_pool[i + 1]);
  }

  head = NULL;
  free_ = wp_pool;
}

/* TODO: Implement the functionality of watchpoint */
WP* new_wp()
{
  if(free_ == NULL)
  {
    printf("Unused watchpoint\n");
    assert(0);
  } 
  WP* tfp = free_;
  free_ = free_->next;
  tfp->next = head;//将新分配的监视点 tfp 的 next 指针指向当前正在使用的监视点链表的头部
  head = tfp;
  return tfp;
}
// free_ -> [1] -> [2] -> NULL
// head -> [0] -> NULL
// free_ -> [2] -> NULL
// head -> [1] -> [0] -> NULL
// free_ -> NULL
// head -> [2] -> [1] -> [0] -> NULL
void free_wp(WP *wp)
{
  if(wp == NULL)
  {
    printf("No watchpoints are using\n");
    assert(0);
  }
  // if(wp->next == NULL)
  // {
  //   wp->next = free_;
  //   free_ = wp;
  //   head = NULL;
  //   return;
  // }
  if(wp->next == NULL)
  {
    if(head == wp)
    {
        head = NULL;
    }
    else
    {
        WP *tmp = head;
        while(tmp->next != wp)
        {
            tmp = tmp->next;
        }
        tmp->next = NULL;
    }
    wp->next = free_;
    free_ = wp;
    return;
}

  else if(wp->next != NULL && wp == head)
  {
    head = wp->next;//指向下一个监视点
    wp->next = free_;
    free_ = wp;
    printf("New head: %p\n", (void *)head);
    return;
  }
  else {
  WP *tmp = head;
  while(tmp->next)
  {
    if(tmp->next == wp)
      break;
    tmp = tmp->next;
  }
  tmp->next = wp->next;
  wp->next = free_;
  free_ = wp;
}
}
 void set_wp(char *arg, word_t value)   //set the watchpoint
{
  WP *new = new_wp();
  new->old_value = value;
  strcpy(new->expr, arg);//将传入的表达式 arg 复制到新监视点的 expr 成员中
  printf("Hardware watchpoint %d: %s\n", new->NO, new->expr);
}
void scan_wp()    //scan all watchpoints
{
  WP *tmp = head;
  while(tmp)
  {
    bool success;
    word_t new_value = expr(tmp->expr, &success);
    if(new_value != tmp->old_value)
    {
      printf("The watchpoint %d: %schanged\n", tmp->NO, tmp->expr);
      printf("Old value = %u\n", tmp->old_value);
      printf("New value = %u\n", new_value);
      tmp->old_value = new_value;
      nemu_state.state = NEMU_STOP;
      printf("The user has triggered the watchpoint.\n");
      return;
    }
    tmp = tmp->next;
  }
  
}
void display_wp()   //display all watchpoints
{
  WP *tmp = head;
  if(tmp == NULL)
    printf("No watchpoints.\n");
  else
  {
    printf("%-20s%-20s%-20s\n", "Num", "expression", "Value");
    while(tmp)
    {
      printf("%-20d%-20s%-20u\n", tmp->NO, tmp->expr, tmp->old_value);
      tmp = tmp->next;
    }
  }
}
void delete_wp(int n)  //delete the watchpoint, its NO is n.
{
  WP *tmp = head;
  if(tmp == NULL)
    printf("The NO %d watchpoint does't exist.\n", n);
  else
  {
    while(tmp->NO != n)
      tmp = tmp->next;
    if(tmp == NULL)
      printf("The NO %d watchpoint does't exist.\n", n);
    else
    {
      free_wp(tmp);
      printf("Deleted\n");
    }
  }
} 
#endif

