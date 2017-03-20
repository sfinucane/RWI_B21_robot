
#ifdef __cplusplus
extern "C" {
#endif
extern float _mySIN(float), _myCOS(float),_myTAN(float);
void initTrigFkt();
#ifdef __cplusplus
}
#endif

#define mySIN(x) sin(x)
#define myCOS(x) cos(x)
#define myTAN(x) tan(x)
