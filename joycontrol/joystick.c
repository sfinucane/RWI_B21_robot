#include <math.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "sys/joystick.h"
#include "beejoystick.h"

#define         JOYSTICK_DEVICE         "/dev/js0"

int joystick_fd;
unsigned char axes, buttons;

int init_joystick(int *a, int *b)
{
  if((joystick_fd = open(JOYSTICK_DEVICE, O_RDONLY)) < 0) {
    perror("init_joystick");
    return -1;
  }
  ioctl(joystick_fd, JSIOCGAXES, &axes);
  ioctl(joystick_fd, JSIOCGBUTTONS, &buttons);
  *a=axes;
  *b=buttons;
  return 0;
}

void close_joystick(void)
{
  close(joystick_fd);
}

void get_joystick(int a[], int b[])
{
  struct js_event js;
  fd_set readset, errset;
  int queue_not_empty;
  struct timeval tv;

  tv.tv_sec=0;
  tv.tv_usec=0;
  do {
    FD_ZERO(&readset);
    FD_SET(joystick_fd, &readset);
    queue_not_empty=select(joystick_fd+1, &readset, NULL, NULL, &tv);
    if(queue_not_empty) {
      read(joystick_fd, &js, sizeof(struct js_event));
      switch(js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_BUTTON:
	b[js.number] = js.value;
	break;
      case JS_EVENT_AXIS:
	a[js.number] = js.value;
	if(abs(a[js.number])<0.3*32767)
	  a[js.number]=0;
	else if(a[js.number]>0)
	  a[js.number]=(a[js.number]-0.3*32767)/(0.7*32767)*32767.0;
	else if(a[js.number]<0)
	a[js.number]=(a[js.number]+0.3*32767)/(0.7*32767)*32767.0;
	break;
      }
    }
  } while(queue_not_empty);
}
