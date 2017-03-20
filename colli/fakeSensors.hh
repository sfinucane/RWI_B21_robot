
#ifdef __cplusplus
extern "C" {
#endif
void
installSimMap(FILE *mapfd);
int
getDistance(float fromX, float fromY, float fromZ,
	    float toX, float toY,
	    float *distance);
#ifdef __cplusplus
}
#endif
