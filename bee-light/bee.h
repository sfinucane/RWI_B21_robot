#ifndef NULL
#define NULL 0
#endif

#ifndef bool 
#define bool int
#endif

/**********************************************************************
 *                                                                    *
 * The following lines are prototype declaration for BeeSoft functions*
 *                                                                    *
 *                                                                    *
 **********************************************************************/   

/* initialize a BeeSoft Light user process */
extern void beeInitialize(char *moduleName) ;

/* move a BeeSoft Light robot along a straight line */
extern bool beeTranslateBy(float distance) ;

/* move a BeeSoft Light robot along a straight line */
extern bool beeTranslatePositive(void)  ;
extern bool beeTranslateNegative(void)  ;

/* set the translation velocity of a BeeSoft Light robot */
extern bool beeSetTransVelocity(float velocity)  ;

/* set the translation acceleration of a BeeSoft Light robot */
extern bool beeSetTransAcceleration(float acceleration) ;

/* rotate a BeeSoft Light robot's orientation */
extern bool beeRotateBy(float degree) ;

/* rotate a BeeSoft Light robot's orientation */
extern bool beeRotatePositive(void) ;
extern bool beeRotateNegative(void)  ;

/* set the rotation velocity of a BeeSoft Light robot */
extern bool beeSetRotVelocity(float velocity) ;

/* set the rotation acceleration of a BeeSoft Light robot */
extern bool beeSetRotAcceleration(float acceleration) ;

/* stop a BeeSoft Light robot when it is moving */
extern bool beeStop()  ;

extern bool beeApproachRelative(float rel_target_x,
                            float rel_target_y) ;
extern bool beeApproachAbsolute(float target_x, float target_y)  ;

/* move a BeeSoft Light robot to a relative position */
extern bool beeGotoRelative(float rel_target_x, float rel_target_y) ;

/* move a BeeSoft Light robot to an absolute position */
extern bool beeGotoAbsolute(float target_x, float target_y)  ;

/* The following defines data structure for robot position      */
/* beeGetCurrentPosition() pass a pointer to the data structure */
/* as an argument.                                              */

typedef struct {
    float pos_x ;
    float pos_y ;
    float orientation ;
} robot_position ;

/* provide current position of a BeeSoft Light robot */
extern robot_position * beeGetCurrentPosition()  ;


/* The following defines data structure for robot speed         */
/* beeGetCurrentSpeed() pass a pointer to the data structure    */
/* as an argument.                                              */

typedef struct {
    float trans_speed ;
    float rot_speed ;
} robot_speed ;

/* provide current speed of a BeeSoft Light robot */
extern robot_speed * beeGetCurrentSpeed()  ;

/* trigger to regularly update sonar data of a BeeSoft Light robot */
extern bool beeSonarStartRegularUpdate() ;

/* stop the regular update operations */
extern bool beeSonarStopRegularUpdate() ;

/* get sonar value */
extern float *beeGetSonarValue(int * number_of_reading)  ;
/* register a callback function to collect sonar data of a BeeSoft Light robot */
extern bool beeSonarRegisterCallback(void (*callbackHandler)(float*))  ;

/* trigger to regularly update infrared data of a BeeSoft Light robot */
extern bool beeIRStartRegularUpdate() ;

/* stop the regular update operations */
extern bool beeIRStopRegularUpdate() ;

/* Infrared sensors are arranged in three rows */
#define UPPERROW 1
#define LOWERROW 2
#define DROW 3

/* get infrared value */
extern int * beeGetIRValue(int rowno, int * number_or_reading)  ;

/* register a callback function to collect infrared data of a BeeSoft Light robot */
extern bool beeIRRegisterCallback(void (*callbackHandler)(int*,int*,int*)) ;

/* trigger to regularly update laser data of a BeeSoft Light robot */
extern bool beeLaserStartRegularUpdate() ;

/* stop the regular update operations */
extern bool beeLaserStopRegularUpdate() ;

/* Laser */

#define FRONT_LASER 0
#define BACK_LASER  1

/* get laser value */
extern int * beeGetLaserValue(int whichone, int * number_of_reading);

/* register a callback function to collect laser data of a BeeSoft Light robot */
extern bool beeLaserRegisterCallback(void (*callbackHandler)
                                  (int, float, float, int*,
                                    int, float, float, int*)) ;
/* define camera reply type */

typedef struct {
  int                xsize, ysize; /* image size (number of pixels)*/
  unsigned char     *red;       /* image data                   */
  unsigned char     *green;     /* image data                   */
  unsigned char     *blue;      /* image data                   */
  int                numGrabber; /* zero for first graber, ... */
} CameraImageType;
extern CameraImageType * beeCameraRequestImage( int numGrabber, 
		            int orig_image_xmin,
		            int orig_image_ymin,
		            int orig_image_xsize,
		            int orig_image_ysize,
		            int return_image_xsize,
		            int return_image_ysize) ;


extern bool beeCameraConnect() ;
extern bool beeCameraSubscribe(int numGrabber, int numberOfImage,
                                 int image_xmin,  int image_ymin,
                                 int image_xsize, int image_ysize,
                          int return_image_xsize, int return_image_ysize);

extern bool beeCameraUnsubscribe(int numGrabber );


extern bool beeCameraImageRegisterCallback(void (*callbackHandler)(CameraImageType *) );
extern int  beeCameraRequestShmId(int numGrabber ) ;
extern bool beeCameraSaveFile(int numGrabber, char *filename, int num ) ;
extern bool beeCameraLoadFile(int numGrabber, char *filename );

