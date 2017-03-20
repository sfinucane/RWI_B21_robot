/* sock.c - by Mike Montemerlo, December 1999 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/timeb.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include "sock.h"

extern int errno;

void sock_signal_handler(int x)
{
  if(x == SIGINT)
    printf("sigint\n");
  else
    printf("sigpipe\n");
}

ssize_t writen(int fd, const void *vptr, size_t n)
{
  size_t nleft;
  ssize_t nwritten;
  const char *ptr;
  fd_set writesock, errsock;

  ptr = vptr;
  nleft = n;
  while(nleft > 0) {
    FD_ZERO(&writesock);
    FD_ZERO(&errsock);
    FD_SET(fd, &writesock);
    FD_SET(fd, &errsock);
    if(select(fd+1, NULL, &writesock, &errsock, NULL)<0)
      if(errno == EINTR)
	nwritten = 0;
      else
	return -1;
    if(FD_ISSET(fd, &errsock)) {
      return -1;
    }
    if((nwritten = write(fd, ptr, nleft)) <= 0) {
      if(errno == EINTR)
	nwritten = 0;
      else
	return (-1);
    }
    nleft -= nwritten;
    ptr += nwritten;
  }
  return (n);
}

ssize_t readn(int fd, void *vptr, size_t n)
{
  size_t nleft;
  ssize_t nread;
  char *ptr;
  fd_set readsock, errsock;
  
  ptr = vptr;
  nleft = n;
  while(nleft > 0) {
    FD_ZERO(&readsock);
    FD_ZERO(&errsock);
    FD_SET(fd, &readsock);
    FD_SET(fd, &errsock);
    if(select(fd+1, &readsock, NULL, &errsock, NULL)<0)
      if(errno == EINTR)
	nread = 0;
      else
	return -1;
    if(FD_ISSET(fd, &errsock)) {
      return -1;
    }
    if((nread = read(fd, ptr, nleft)) < 0) {
      if(errno == EINTR)
	nread = 0;
      else
	return (-1);
    } 
    else if(nread == 0)
      break;
    nleft -= nread;
    ptr += nread;
  }
  return (n);
}

int open_tcp_server(int port)
{
  int listenfd, err;
  struct sockaddr_in servaddr;

  signal(SIGPIPE, sock_signal_handler);
  signal(SIGINT, sock_signal_handler);
  listenfd = socket(AF_INET, SOCK_STREAM, 0);
  if(listenfd < 0)
    return -1;
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(port);
  err = bind(listenfd, (SA *)&servaddr, sizeof(servaddr));
  if(err < 0)
    return -1;
  printf("Opened a TCP server on port %d\n", port);
  return listenfd;
}

int listen_for_connection(int listenfd)
{
  int connfd;
  fd_set readsock, errsock;
  struct sockaddr_in cliaddr;
  socklen_t clilen;

  listen(listenfd, 5);
  FD_ZERO(&readsock);
  FD_SET(listenfd, &readsock);
  FD_ZERO(&errsock);
  FD_SET(listenfd, &errsock);
  if(select(listenfd + 1, &readsock, NULL, &errsock, NULL) < 0)
    return -1;
  connfd = accept(listenfd, (SA *)&cliaddr, &clilen);
  if(connfd < 0)
    return -1;
  printf("Connected socket.\n");
  return connfd;
}

void close_connection(int sock)
{
  printf("Disconnected socket.\n");
  close(sock);
}

int connect_to_server(char *host, int port)
{
  int sockfd;
  struct hostent *addr;
  unsigned long addr_tmp;
  struct sockaddr_in servaddr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0)
    return -1;
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  if(atoi(host) > 0)
    servaddr.sin_addr.s_addr=inet_addr(host);
  else {
    if((addr = gethostbyname(host))==NULL)
      return -1;
    bcopy(addr->h_addr, (char *)&addr_tmp, addr->h_length);
    servaddr.sin_addr.s_addr = addr_tmp;
  }
  servaddr.sin_port = htons(port);
  if(connect(sockfd, (SA *)&servaddr, sizeof(servaddr)) < 0)
    return -1;
  return sockfd;
}

