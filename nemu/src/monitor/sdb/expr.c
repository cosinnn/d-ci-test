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

/* We use the POSIX regex functions to process regular expressions.
 * Type 'man regex' for more information about POSIX regex functions.
 */
#include <regex.h>

enum {
  TK_NOTYPE = 256, TK_EQ=255,
                                                               
  /* TODO: Add more token types */
  TK_NEG=254,TK_POS=253,TK_DEREF=252,TK_NEQ=251,TK_GT=250,TK_LT=249,TK_GE=248,TK_LE=247,TK_AND=246,TK_OR=245,TK_NUM=244,TK_REG=243,
  TK_MINUS=242,TK_MUL=241,TK_DIV=240,TK_LPAREN=239,TK_RPAREN=238,TK_PLUS=237,
};

static struct rule {
  const char *regex;
  int token_type;
} rules[] = {

  /* TODO: Add more rules.
   * Pay attention to the precedence level of different rules.
   */
  {" +", TK_NOTYPE},
  //{"[0-9]+",TK_NUM},
  {"(0[xX][0-9A-Fa-f]+|\\b[0-9]+\\b)",TK_NUM},
  {"\\-",TK_MINUS},
  {"\\*",TK_MUL},
  {"\\/",TK_DIV},
  {"\\(",TK_LPAREN},
  {"\\)",TK_RPAREN},
  {"\\+",TK_PLUS},         // plus
  {"==", TK_EQ}, 
  {"<", TK_LT}, 
  {">", TK_GT}, 
  {"<=", TK_LE}, 
  {">=", TK_GE},
  {"!=", TK_NEQ},
	{"&&", TK_AND}, 
  {"\\|\\|", TK_OR}, 
  {"\\$[a-zA-Z0-9_]+", TK_REG},      // equal
};

#define NR_REGEX ARRLEN(rules)

static regex_t re[NR_REGEX] = {};

/* Rules are used for many times.
 * Therefore we compile them only once before any usage.
 */
void init_regex() {
  int i;
  char error_msg[128];
  int ret;

  for (i = 0; i < NR_REGEX; i ++) {
    ret = regcomp(&re[i], rules[i].regex, REG_EXTENDED);
    if (ret != 0) {
      regerror(ret, &re[i], error_msg, 128);
      panic("regex compilation failed: %s\n%s", error_msg, rules[i].regex);
    }
  }
}

typedef struct token {
  int type;
  char str[32];
} Token;

static Token tokens[4096] __attribute__((used)) = {};
static int nr_token __attribute__((used))  = 0;

static bool make_token(char *e) {
  int position = 0;
  int i;
  regmatch_t pmatch;

  nr_token = 0;

  while (e[position] != '\0') {
 
    /* Try all rules one by one. */
    while (e[position] == ' ' || e[position] == '\t') {
      position++;
 
    }
    if (e[position] == '\0') {
      break; // End of string
    }
    for (i = 0; i < NR_REGEX; i ++) {
      if (regexec(&re[i], e + position, 1, &pmatch, 0) == 0 && pmatch.rm_so == 0) {
        char *substr_start = e + position;
        int substr_len = pmatch.rm_eo;

        Log("match rules[%d] = \"%s\" at position %d with len %d: %.*s",
            i, rules[i].regex, position, substr_len, substr_len, substr_start);

        position += substr_len;
        /* TODO: Now a new token is recognized with rules[i]. Add codes
         * to record the token in the array `tokens'. For certain types
         * of tokens, some extra actions should be performed.
         */
        if(rules[i].token_type == TK_NOTYPE)
        {
          position++;
        }
        switch (rules[i].token_type) {
          case TK_NUM:
          //case TK_REG:
            strncpy(tokens[nr_token].str,substr_start,substr_len);
            tokens[nr_token].str[substr_len] = '\0'; 
            tokens[nr_token].type = rules[i].token_type;
            break;
          case TK_REG:
            strncpy(tokens[nr_token].str,substr_start,substr_len);
            tokens[nr_token].str[substr_len] = '\0'; 
            tokens[nr_token].type = rules[i].token_type;
            break;           
          // case '+':
          // case '-':
          // case '*':
          //   if(nr_token == 0||!OFTYPES(tokens[nr_token - 1].type, bound_types)){
          //     switch(rules[i].token_type){
          //       case '-':tokens[nr_token].type = TK_NEG;
          //       break;
          //       case '+':tokens[nr_token].type = TK_POS;
          //       break;
          //       case '*':tokens[nr_token].type = TK_DEREF;
          //     }
          //   }else{
          //       tokens[nr_token].type = rules[i].token_type;                
          //   }
          //   break;
          
          case TK_MUL:tokens[nr_token].type = TK_MUL;break;
          case TK_PLUS:tokens[nr_token].type = TK_PLUS;break;
          case TK_MINUS:tokens[nr_token].type = TK_MINUS;break;
          case TK_DIV: tokens[nr_token].type =TK_DIV;break;
          case TK_LPAREN:tokens[nr_token].type =TK_LPAREN;break;
          case TK_RPAREN:tokens[nr_token].type =TK_RPAREN;break;
          case TK_EQ:tokens[nr_token].type = TK_EQ;break;
          case TK_NEQ:tokens[nr_token].type = TK_NEQ;break;
          case TK_AND:tokens[nr_token].type = TK_AND;break;
          case TK_NOTYPE:continue;
          default: panic("Unknown token type");
        }
        nr_token ++;
        break;
      }
    }
    if (i == NR_REGEX) {
      printf("no match at position %d\n%s\n%*.s^\n", position, e, position, "");
      return false;
    }
  }
  return true;
}

bool check_parentheses(int p,int q){
  if(tokens[p].type == TK_LPAREN&&  tokens[q].type==TK_RPAREN)
  {
    int count =0;
    for(int i = p+1;i<=q-1;i++)
    {
      if(tokens[i].type == TK_LPAREN)
      {           
        count++;
      }
      else if(tokens[i].type == TK_RPAREN)
      {
         count--;
      }        
      if(count<0)
        {
          return false;
        }
    }   
    return count == 0;
  }
  return false;  
}
int find_op(int p,int q){
  int d= -1;
  int a = 0;
  int c = -1;
  for(int i = p;i<=q;i++)
  {
    //if(tokens[p].type != TK_MINUS||tokens[i].type != TK_DIV||tokens[i].type !=TK_MUL)

    
    if(tokens[i].type == TK_NUM)
    {
      continue;
    }
    else if(tokens[i].type ==TK_LPAREN)
    {
      a++;
    }
    else if(tokens[i].type ==TK_RPAREN)
    {
      if(a==0){
        return -1;
      }
      a--;
    } 
    else if(!(tokens[i].type == TK_MINUS || tokens[i].type == TK_DIV || tokens[i].type == TK_MUL || tokens[i].type == TK_PLUS||tokens[i].type == TK_AND||tokens[i].type == TK_EQ||tokens[i].type == TK_NEQ||tokens[i].type == TK_DEREF))
    {
      continue;
    }
    else{
      if(a>0)
      {
        continue;
      }
      else 
      {

      
        int b;
        switch(tokens[i].type)
        { 
        // case TK_NEG:
        case TK_DEREF:
            b = 1;break;
          case TK_MUL:
          case TK_DIV:
            b = 2;break;
          case TK_PLUS:
          case TK_MINUS:
            b = 3;break;
        // case TK_GT:   
        // case TK_LT: 
        // case TK_GE: 
        // case TK_LE: 
        //   b = 4; break;
          case TK_EQ:  
          case TK_NEQ: 
            b = 4; break;
          case TK_AND:
            b = 5;break;
        // case TK_OR:
        //   b = 7;break;
           default:assert(0);
        }
      
        if(b>c)
        {                                             
         c = b;
         d = i;
        }
        else if(b==c)
        {d = i;}
      }
    }
  }
  return d;
}
word_t isa_reg_str2val(const char *s,bool *success);
word_t paddr_read(paddr_t addr, int len);
word_t eval (int p,int q,bool *success)
{


  if (p > q) {
    /* Bad expression */
    printf("%d %d %d\n",p,q,nr_token);
    printf("Error: Bad expression (empty)\n");
    *success = false;
    return -1;
  }
  else if (p == q) {
    /* Single token.
     * For now this token should be a number.
     * Return the value of the number.
     */
    if(tokens[p].type == TK_NUM)
    {
      if(strncmp("0x",tokens[p].str,2) ==0 || strncmp("0X",tokens[p].str,2)== 0){
        return strtol(tokens[p].str,NULL,16);}
       else 
       {return strtol(tokens[p].str,NULL,10);}
      //return strtol(tokens[p].str,NULL,10);
    }
    else{
      return isa_reg_str2val(tokens[p].str,success);
      // printf("Error:Single token is not a number!\n");
      // return -1;
    }
  }
  else if (check_parentheses(p, q)) {
    /* The expression is surrounded by a matched pair of parentheses.
     * If that is the case, just throw away the parentheses.
     */
    return eval(p + 1, q - 1,success);
  }
  else {

    int op = find_op(p,q);
    if(op == -1)
    {
      printf("Error: No valid operator found\n");
      return -1;
    }
    if(tokens[op].type == TK_DEREF)
    {
      uint32_t address = eval(op+1,q,success);
      return paddr_read(address,4);
    }
    int val1 = eval(p, op - 1,success);
    // if( val1 == -1)
    // {
    //   return -1;
    // }
    int val2 = eval(op + 1, q,success);
    // if(val2 == -1)
    // {
    //   return -1;
    // }

    switch (tokens[op].type) {
      case TK_PLUS: return val1 + val2;
      case TK_MINUS: return val1 - val2;
      case TK_MUL: return val1 * val2;
      case TK_DIV: if(val2 == 0)
      {
        printf("Error:have_0\n");
        return -1;
      }
      return val1 / val2;
      case TK_EQ:return val1 == val2;
      case TK_NEQ:return val1 != val2;
      case TK_AND:return val1 && val2;
      case TK_DEREF:
      default: assert(0);
    }
  }
}
bool certain_type(int index) {

  if (index < 0) return true;

  switch (tokens[index].type) {
    case TK_LPAREN: 
    
    case TK_PLUS:   
    case TK_MINUS:  
    case TK_MUL:    
    case TK_DIV:    
    case TK_DEREF:
    case TK_AND:
    case TK_EQ:
    case TK_NEQ: 
      
      return true;
    case TK_RPAREN:
    default:
      return false;
  }
}
word_t expr(char *e, bool *success) {
  //printf("%s\n", e);
  if (!make_token(e)) {
    *success = false;
    return 0;
  }
  int p =0;
  int q = nr_token -1;

  for(int i = 0;i<nr_token;i++)
  {
    if (tokens[i].type == TK_MUL && (i == 0 || certain_type(i-1)) ) {
    tokens[i].type = TK_DEREF;
    }
  }

  /* TODO: Insert codes to evaluate the expression. */
   
  *success = true;
  word_t result  = eval(p,q,success);
  return result;
  return 0;
}
