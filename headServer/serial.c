/* serial.c - by Mike Montemerlo, December 1999 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>

int open_serialport(char *dev)
{
  int fd;
  struct termios newtio;

  fd = open(dev, O_RDWR | O_NOCTTY);
  if(fd == -1) {
    fprintf(stderr, "Couldn't open serial port %s\n", dev);
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  return fd;
}

void close_serialport(int fd)
{
  close(fd);
}
