/* headServer - by Mike Montemerlo, December 1999 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "serial.h"
#include "sock.h"
#include "face.h"

#define        PTU_PORT              "/dev/ttyS2"
#define        MAX_CONNECTIONS       5

typedef struct {
  int sock;
  int which;
  int active;
  pthread_t thd;
} connection;

int server_done = 0;
int serial_port;
connection conn[MAX_CONNECTIONS];
int connection_num = 0;
face_state fs;
pthread_mutex_t pantilt_mutex;

void signal_handler(int x)
{
  if(x == SIGINT)
    server_done = 1;
}

void send_pantilt_command(char *str)
{
  pthread_mutex_lock(&pantilt_mutex);
  writen(serial_port, str, strlen(str));
  if(str[strlen(str) - 1] != ' ')
    writen(serial_port, " ", 1);
  pthread_mutex_unlock(&pantilt_mutex);
}

void *handle_shell(void *x)
{
  connection c = *((connection *)x);
  char cmd[50], response[50];
  int mark, err, i, done;

  done = 0;
  do {
    sprintf(cmd, "head: ");
    writen(c.sock, cmd, 7);
    mark = 0;
    do {
      err = readn(c.sock, response + mark, 1);
      mark++;
    } while(response[mark - 1] != '\n' && err == 1 && mark < 50);
    if(mark < 50 && err == 1) {
      for(i = 0; i < mark; i++)
	if(response[i] >= 'a' && response[i] <= 'z')
	  response[i] = response[i] - 'a' + 'A';
      response[mark - 2] = '\0';
      if(strncmp(response, "QUIT", 4) == 0)
	done = 1;
      else if(strncmp(response, "PT", 2) == 0) {
	printf("Pantilt command: %s %d\n", response + 2, strlen(response + 2));
	send_pantilt_command(response + 2);
      }
      else if(strncmp(response, "FE", 2) == 0) {
	printf("Eyes command: %s %d\n", response + 2, strlen(response + 2));
	FaceSetEyes(&fs, atoi(response + 2));
      }
      else if(strncmp(response, "FM", 2) == 0) {
	printf("Mouth command: %s %d\n", response + 2, strlen(response + 2));
	FaceSetMouth(&fs, atoi(response + 2));
      }
      else if(strncmp(response, "FB", 2) == 0) {
	printf("Brows command: %s %d\n", response + 2, strlen(response + 2));
	FaceSetBrows(&fs, atoi(response + 2));
      }
      response[0] = '\0';
    }
    else if(err != 1)
      done = 1;
  } while(!done);
  close_connection(c.sock);
  c.active = 0;
  connection_num--;
}

int main(int argc, char **argv)
{
  int listen_sock, sock, port, i, err;
  char cmd[50];

  if(argc != 2) {
    fprintf(stderr, "Usage: %s portnum\n", argv[0]);
    exit(1);
  }
  port = atoi(argv[1]);
  listen_sock = open_tcp_server(port);
  if(listen_sock < 0) {
    fprintf(stderr, "Error: could not open socket %d for listening.\n", port);
    exit(1);
  }
  serial_port = open_serialport(PTU_PORT);
  if(serial_port < 0) {
    fprintf(stderr, "Error: could not open serial port %s\n", PTU_PORT);
    exit(1);
  }
  send_pantilt_command("PP0 ");
  send_pantilt_command("TP0 ");
  
  err = FaceOpen(&fs);
  if(err) {
    fprintf(stderr, "Error: could not open robot face.\n");
    exit(1);
  }
  FaceReset(&fs);

  signal(SIGINT, signal_handler);

  pthread_mutex_init(&pantilt_mutex, NULL);

  for(i = 0; i < MAX_CONNECTIONS; i++)
    conn[i].active = 0;
  do {
    sock = listen_for_connection(listen_sock);
    if(connection_num == MAX_CONNECTIONS)
      close_connection(sock);
    else {
      i = 0;
      while(conn[i].active)
	i++;
      conn[i].sock = sock;
      conn[i].which = i;
      pthread_create(&(conn[i].thd), NULL, handle_shell, 
		     &(conn[i]));
      connection_num++;
    }
  } while(!server_done);

  pthread_mutex_destroy(&pantilt_mutex);

  FaceClose(&fs);
  close_serialport(serial_port);
  close_connection(listen_sock);
  printf("Closing server.\n");
  return 0;
}
