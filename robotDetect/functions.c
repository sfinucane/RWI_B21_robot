
/* ------------------------------------------------------------ 
 * deg2Rad(int x)
 * converts degrees into radians
 * Taken from bee/src/localize/function.c
 * -----------------------------------------------------------*/
float deg2Rad(int x) {
  return x * 0.017453293;
}

float
rad2Deg(float x)
{
  return x * 57.29578;
}



/* ---------------------------------------------------------
 * getMax() returns the maximum value of array data.
 * --------------------------------------------------------*/
float getMax(float* data, int count) {
  float max = data[0];
  int i;
  for (i = 0; i < count; i++) {
    if (data[i] > max) max = data[i];
  }
  return max;
}

