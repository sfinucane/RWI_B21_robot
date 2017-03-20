#include "bee.h"
#include <stdio.h>

void mycallback(CameraImageType * image) 
{
   fprintf(stderr, "%p", image) ;
   if (image) 
   {
      fprintf(stderr, "%d, %d, %p, %p, %p, %d", image->xsize , image->ysize ,
       image->red, image->green, image->blue, image->numGrabber) ;
   }
}
int main()
{
   int returnval ;
   CameraImageType * image ;
   beeInitialize() ;
   beeCameraConnect() ;
   beeCameraImageRegisterCallback( mycallback) ;  
   beeCameraSubscribe(0, 2, 0, 0, 320, 240, 160, 120) ;
   sleep(20) ;
   beeCameraUnsubscribe(0) ;
   fprintf(stderr, "stop") ;
   sleep(10) ;
}


