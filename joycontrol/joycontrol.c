#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "tcx.h"
#include "bee.h"
#include "sys/joystick.h"

int a, b;
int axis[20], button[20];

int main(int argc, char **argv)
{
  int done = 0;
  float x, y;
  float Old_X = 0, Old_Y = 0;
  int Num_Sonars;
  float *sonar_array;
  int Index;
 
  if(init_joystick(&a, &b)) {
    printf("error: couldn't open joystick.\n");
    exit(1);
  }
  
  beeInitialize("JOYSTICK");
  beeSonarStartRegularUpdate();
  beeLaserStartRegularUpdate();

  beeSetTransVelocity(0);
  beeSetRotVelocity(0);
  while(!done) {
    get_joystick(axis, button);
    x = axis[0]/32767.0*50.0;
    y = -axis[1]/32767.0*20.0;
    if (fabs(x) < 1)
      x = 0;
    if (fabs(y) < 1)
      y = 0;

    if(button[0])
      done = 1;
    
    beeSetRotVelocity(fabs(x));
    if (fabs(x) > 0) {
      if (x > 0)
	beeRotatePositive(); 
      else
	beeRotateNegative();
    }
    
    
    if (fabs(y) > 5) { 
      beeSetTransVelocity(fabs(y));
      if (y > 0)
	beeTranslatePositive();
      else
	beeTranslateNegative();
    } else {
      y = 0;
      beeSetTransVelocity(y);
    }
  
    if (Old_X != x || Old_Y != y)
      printf("x, y = %8.2f %8.2f : Rotate : %8.2f, Translate : %8.2f\n", x, y, 
	     fabs(x), fabs(y));

    Old_X = x;
    Old_Y = y;
    
    Num_Sonars = 16;
    sonar_array = beeGetSonarValue(&Num_Sonars);

    /*    l = y * 200.0 - x * 100.0;
    if (l > 200.0) 
      l = 200.0;
    else if (l < -200.0)
      l = -200;
    r = y * 200.0 + x * 100.0;
    if (r > 200.0) 
      r = 200.0;
    else if (r < -200.0)
      r = -200;*/
    
    usleep(200000);
  }
  close_joystick();
}
