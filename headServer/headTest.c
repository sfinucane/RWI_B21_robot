#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "headClient.h"

int main(int argc, char **argv)
{
  int err;

  err = head_connect();
  if(err < 0) {
    fprintf(stderr, "Error: could not connect to headServer.\n");
    exit(1);
  }

  while(1) {
    head_pantilt_command("TP500");

    head_pantilt_command("PP1000");
    head_move_eyes(50);
    sleep(1);
    head_pantilt_command("PP-1000");
    head_move_eyes(-50);
    sleep(1);

    head_pantilt_command("TP-500");

    head_pantilt_command("PP-1000");
    head_move_eyes(50);
    sleep(1);
    head_pantilt_command("PP1000");
    head_move_eyes(-50);
    sleep(1);
  }

  head_disconnect();
}
