#ifndef __SHELL_H__
#define __SHELL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define SHELL_BUFFER_LEN	(200)
#define MAX_SHELL_ENTRYS	(20)
#define MAX_ARGC			(15)
#define ITEMNUM(x)          (sizeof(x) / sizeof((x)[0]))     // tömb elemeinek száma


typedef int16_t (*SHELL_GETCHAR) (void);
typedef void    (*SHELL_PUTCHAR) (char);


typedef struct {
  SHELL_GETCHAR   getchar;
  SHELL_PUTCHAR   putchar;
  char            buffer[SHELL_BUFFER_LEN];
  int16_t         buffer_idx;
} SHELL_TERMINAL_ST;



extern int  Shell       (SHELL_TERMINAL_ST *terminal, char * str);
extern void ShellTask   (void);


#ifdef __cplusplus
}
#endif

#endif //__SHELL_H__
