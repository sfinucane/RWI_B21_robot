#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include "sock.h"
#include "headClient.h"

#define       FACE_HOST         "flo2"
#define       FACE_PORT         3000

int head_initialized = 0;
int sock;

int head_connect(void)
{
  int err;
  
  sock = connect_to_server(FACE_HOST, FACE_PORT);
  if(sock >= 0) {
    head_initialized = 1;
    return 0;
  }
  else
    return -1;
}

void head_disconnect(void)
{
  close_connection(sock);
}

void read_prompt(void)
{
  int n, i;
  char c, prompt[20];

  do {
    for(i = 0; i < 5; i++)
      prompt[i] = prompt[i + 1];
    n = readn(sock, &(prompt[5]), 1);
  } while(strncmp(prompt, "head: ", 6));
}

void head_pantilt_command(char *str)
{
  char cmd[20];

  read_prompt();
  sprintf(cmd, "PT%s\r\n", str);
  writen(sock, cmd, strlen(cmd));
}

void head_move_eyes(int arg)
{
  char cmd[20];

  read_prompt();
  sprintf(cmd, "fe%d\r\n", arg);
  writen(sock, cmd, strlen(cmd));
}

void head_move_mouth(int arg)
{
  char cmd[20];

  read_prompt();
  sprintf(cmd, "fm%d\r\n", arg);
  writen(sock, cmd, strlen(cmd));
}

void head_move_brows(int arg)
{
  char cmd[20];

  read_prompt();
  sprintf(cmd, "fb%d\r\n", arg);
  writen(sock, cmd, strlen(cmd));
}
