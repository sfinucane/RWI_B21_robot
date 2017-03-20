
#include "colorFilter.h"

char rgbImage[320*240*3];

int main(int argc, char** argv) {
  openAndReadPPM(rgbImage, argv[1]); 
  preProcessImage(rgbImage);
  return 0;
} 
