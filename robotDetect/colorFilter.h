int preProcessImage(char *rgbImage);
int openAndReadPPM(char* rgbImage, char *filename);
int fastGetMarkerCoordinates(char* image, int bpp, int imgDecimation, int* resultX, int* resultY);
int getAngleAndDistance(int x, float* laserSweep,  int *markerAngle, float *markerDistance);
void printAngleAndDistance(char* rgbImage, int bpp, int imgDecimation, float* laserSweep);
void insertMarker(char* image, int xPos, int yPos, int bpp);
void insertGrid(char* image, int bpp);
