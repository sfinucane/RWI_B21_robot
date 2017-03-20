/*
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File name:                   bee.c
 *****
 ***** Part of:                     BeeSoft Light Software
 *****
 ***** Creator:                     The Machine Learning Lab, Carnegie Mellon
 *****                              University 
 *****
 ***** Date of creation:            Feb 1999
 *****
 *****
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:04:13 $
 *****
 ***** $Author: rstone $
 *****
 *****
 ***** Contact thrun@cs.cmu.edu 
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include "tcx.h"
#include "tcxP.h"

//#define USER_debug

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#define TCX_define_variables /* this makes sure variables are installed */ 

#define bool int

#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "LOCALIZE-messages.h"
#include "PLAN-messages.h"
#include "CAMERA-messages.h"

#define TCX_USER_MODULE_NAME "USER"

/* define id of infrared sensor rows */
#define UPPERROW 1
#define LOWERROW 2
#define DROW 3

/* define id of laser sensor */
#define FRONT_LASER 0
#define REAR_LASER 1
#include "beeSoftVersion.h"


/* Base and sensor call back function will keep updating the following data
 * variable.
 * Keep it public to references from code outside the file. 
 * User defined callback functions will be able to access those momery
 * space.   
 * ROBOT_STATE stores data replied from colliBase server.
 * sonar_values store sonar sensor data replied from colliBase server.
 * ir_values store infrared sensor data replied from colliBase server.
 * There are three arrays of infrared sensors.
 * laser_reading store laser data replied from colliBase server.
 * There are two arrays of laser sensors.
 *
 */

static struct 
{
  float  pos_x;                 
  float  pos_y;                 
  float  orientation; 

  float  rot_current_speed;     
  float  trans_current_speed;   
  float  rot_current_acceleration ;
  float  trans_current_acceleration ;
   
} ROBOT_STATE;

static float sonar_values[24];
static int ir_upperrow_values[24];
static int ir_lowerrow_values[24];
static int ir_drow_values[8];
static int   laser_f_numberOfReadings;
static float laser_f_angleResolution;
static float laser_f_startAngle;
static int * laser_f_reading;
static int   laser_r_numberOfReadings;
static float laser_r_angleResolution;
static float laser_r_startAngle;  
static int * laser_r_reading;

/* The following arrays are clone of the the sensor readings declared above 
 * They are open to access by user process. When user process invokes
 * beeGetSonarValue() or beeGetIRValue() or beeGetLaserValue(), those
 * functions make duplicates from the readings declared above to the 
 * arrays below. The reason why I make sensor readings two different
 * arrays is that pub_xxxxx_values[] are open to user process. It is
 * possible that user process is accessing the readings while the
 * background callback thread is updating the readings.
 * Some lines implements the mutex mechanisms to prevent from 
 * concurrent access.
 */ 
 
float pub_sonar_values[24];
int pub_ir_upperrow_values[24];
int pub_ir_lowerrow_values[24];
int pub_ir_drow_values[8];
int * pub_laser_f_reading;
int * pub_laser_r_reading;
/* Define camera image type */
/* There are three methods to retrieve camera image :
 *   1. retrieve image data direcctly. If camera server is running on a
 *      remote machine, there is a big amount of data transmitting on the
 *      network.
 *   2. get share memory id first, and then access the memory space with the
 *      id
 *   3. request to the server to save image in a file. 
 *
 */
typedef struct {
  int                xsize, ysize; /* image size (number of pixels)*/
  unsigned char     *red;       /* image data                   */
  unsigned char     *green;     /* image data                   */
  unsigned char     *blue;      /* image data                   */
  int                numGrabber; /* zero for first graber, ... */
} CameraImageType; 

typedef struct cameraShmIdType { 
  int shmid;                    /* id of the shm segment */
  int numGrabber;               /* zero for first grabber, ... */
} cameraShmIdType;

typedef struct cameraFileType {
  int dummy;
} cameraFileType;

CameraImageType cameraImageReply ;   

/* Store the memory address of users' defined callback function */
static void (*UserSonarCallbackHandler) (float*) = NULL ;
static void (*UserIRCallbackHandler)(int*, int*, int*) = NULL ;
static void (*UserLaserCallbackHandler)(int, float, float, int*, int, float, float, int*) = NULL ; 
static void (*UserCameraCallbackHandler) (CameraImageType* ) = NULL ;

/* correctionParameter is updated by LOCALIZE_reply_handler */
/* It is used to compute the coordinate correction between baseServer 
 * and global map
 */

typedef struct {
  float x;
  float y;
  float rot;
  int type;
  int initialized;
} correctionParameter;

correctionParameter correction ;

/* The following variables are used to indicate whether or not the
 * sensor and base values which are stored above are valid 
 */
static bool known_pos = FALSE;
static bool known_sonar_val = FALSE ; 
static bool known_ir_val = FALSE;
static bool known_camera_image[2] = {FALSE , FALSE} ;
static bool correctionParametersKnown = FALSE;


static int tcx_initialized = 0 ;

BASE_register_auto_update_type baseUpdate;


/* The following functions are used to translate coordinates between 
 * robot base server and map 
 */

/* Translate coordinate from robot base server to map */
static void
compute_forward_correction(float robot_x,
                           float robot_y,
                           float robot_orientation,
                           float corr_x,
                           float corr_y,
                           float corr_angle, /* in deg. */
                           int   corr_type,
                           float *corr_robot_x,
                           float *corr_robot_y,
                           float *corr_robot_orientation)
{
  float corr_angle_2pi;

  *corr_robot_orientation = robot_orientation - corr_angle;
  for (; *corr_robot_orientation <= -180.0;) *corr_robot_orientation += 360.0;
  for (; *corr_robot_orientation >   180.0;) *corr_robot_orientation -= 360.0;

  corr_angle_2pi = corr_angle * M_PI / 180.0;

  *corr_robot_x = ((robot_x - corr_x) * cos(corr_angle_2pi))
    + ((robot_y - corr_y) * sin(corr_angle_2pi));

  *corr_robot_y = ((robot_y - corr_y) * cos(corr_angle_2pi))
    - ((robot_x - corr_x) * sin(corr_angle_2pi));
}

/* Translate coordinate from map to robot base server  */ 

static void
compute_backward_correction(float corr_robot_x,
                            float corr_robot_y,
                            float corr_robot_orientation,
                            float corr_x,
                            float corr_y,
                            float corr_angle, /* in deg. */
                            int   corr_type,
                            float *robot_x,
                            float *robot_y,
                            float *robot_orientation)

{
  float corr_angle_2pi;

  *robot_orientation  = corr_angle + corr_robot_orientation;
  for (; *robot_orientation <= -180.0;) *robot_orientation += 360.0;
  for (; *robot_orientation >   180.0;) *robot_orientation -= 360.0;

  corr_angle_2pi = corr_angle * M_PI / 180.0;

  *robot_x = corr_x + (corr_robot_x * cos(corr_angle_2pi))
    - (corr_robot_y * sin(corr_angle_2pi));

  *robot_y = corr_y + (corr_robot_y * cos(corr_angle_2pi))
    + (corr_robot_x * sin(corr_angle_2pi));
}


/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

static void interrupt_handler(int sig)
{

    fprintf(stderr, "User terminated the program\n");
    fflush(stderr);
    tcxCloseAll() ;
    exit(0);
}

/************************************************************************
 *
 *   NAME:         USER_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void USER_close_handler(char *name, TCX_MODULE_PTR module)
{
    if (!strcmp(name, "TCX Server"))
    { 
        /* TCX shut down */
        tcx_initialized = FALSE ;
        fprintf(stderr, "TCX Server shut down\n" );
    }
    if ( module == BASE)
    {
	fprintf(stderr, "%s disconnected.\n", "BASE");
	BASE = NULL;
    }
    if ( module == LOCALIZE) 
    {
	fprintf( stderr, "%s disconnected.\n", "LOCALIZE");
	LOCALIZE = NULL;
    }
    if ( module == PLAN )
    {
        PLAN = NULL ;
        fprintf( stderr, "%s disconnected. \n", "plan") ;
    }
    if (module == CAMERA )
    {
        CAMERA = NULL ;
        fprintf( stderr, "%s disconnected. \n", "cameraServer") ;
    }
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

/* The following two variables store the destination of former beeApproachAbsolute()
 * function. Because robot orientation will drift, LOCALIZE_update_status_reply_handler()
 * need to keep re-sending the destination to the robot. 
 * They are set in beeApproachAbsolute() , reset in BASE_update_status_reply_handler(),
 * and used in LOCALIZE_update_status_reply_handler()
 */
static float approach_des_x ;
static float approach_des_y ;
static float compu_distance(float x1, float y1, float x2, float y2)
{
    float temp_x, temp_y ;
    temp_x = (x2 - x1) * (x2 - x1) ;
    temp_y = (y2 - y1) * (y2 - y1) ;
    return (sqrt(temp_x + temp_y) ) ;
}

int mutex_ROBOT_status_flag[2] = {0,0} ;
int mutex_ROBOT_status_stuck = 0 ;

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status)
{
#ifdef USER_debug
    fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
    fprintf(stderr, "robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif
    mutex_ROBOT_status_flag[0] = 1 ;
    while(mutex_ROBOT_status_flag[1] && mutex_ROBOT_status_stuck == 0) ;

    ROBOT_STATE.pos_x  = status->pos_x;
    ROBOT_STATE.pos_y  = status->pos_y;
    ROBOT_STATE.orientation  = status->orientation;
    ROBOT_STATE.rot_current_speed = status->rot_current_speed;     
    ROBOT_STATE.trans_current_speed = status->trans_current_speed;  
    ROBOT_STATE.rot_current_acceleration = status->rot_acceleration;
    ROBOT_STATE.trans_current_acceleration = status->trans_acceleration;
    
    mutex_ROBOT_status_flag[0] = 0 ;

    if(approach_des_x || approach_des_y)
    {
        if(compu_distance(approach_des_x, approach_des_y, ROBOT_STATE.pos_x, ROBOT_STATE.pos_y) < 100 ) 
        {
           /* Almost reach the destination! Reset the destination to zero to stop 
            * LOCALIZE_update_status_reply_handler() continuing sending request to
            * baseServer
            */
            approach_des_x = 0 ;
            approach_des_y = 0 ;
        }
    }

    known_pos = TRUE;

    tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_sonar_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
/* mutex mechanism */
int mutex_sonar_flag[2] = {0,0} ;
int mutex_sonar_stuck = 0 ;
void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar)
{
    int i;

#ifdef USER_debug
    fprintf(stderr, "TCX: Received a SONAR_sonar_reply message.\n");
    fprintf(stderr, "\n");
#endif
    mutex_sonar_flag[0] = 1 ;
    while(mutex_sonar_flag[1] && mutex_sonar_stuck ==0) ;

    for (i = 0; i < 24; i++)
        sonar_values[i] = sonar->values[i] ;
    
    mutex_sonar_flag[0] = 0 ;

    known_sonar_val = TRUE ;
  
    tcxFree("SONAR_sonar_reply", sonar);

    /* The following callback function is running on the 
     * background thread too. If user defines too long
     * callback function, the background thread could not
     * collect next sonar reading quickly 
     */    
    if(UserSonarCallbackHandler!=NULL)
        (* UserSonarCallbackHandler)(sonar_values) ;
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_ir_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

/* mutex mechanism */
int mutex_ir_flag[2] = {0,0} ;
int mutex_ir_stuck = 0 ;
void SONAR_ir_reply_handler (TCX_REF_PTR          ref,
			     SONAR_ir_reply_ptr   ir)
{
    int i ;
#ifdef USER_debug
    fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
#endif   

    mutex_ir_flag[0] = 1 ;
    while(mutex_ir_flag[1] && mutex_ir_stuck==0) ;

    for (i = 0; i < 24; i++)
        ir_upperrow_values[i] = ir->upperrow[i] ;
    for (i = 0; i < 24; i++)
        ir_lowerrow_values[i] = ir->lowerrow[i] ;
    for (i = 0; i < 8; i++)
        ir_drow_values[i] = ir->drow[i] ;

    mutex_ir_flag[0] = 0 ;

    known_ir_val = TRUE ;
    tcxFree("SONAR_ir_reply", ir);
    if(UserIRCallbackHandler)
        (* (UserIRCallbackHandler)) (ir_upperrow_values,
                                     ir_lowerrow_values, 
                                     ir_drow_values) ;      
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         LASER_laser_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

/* mutex mechanism */
int mutex_laser_flag[2] = {0,0} ;
int mutex_laser_stuck = 0 ;
void LASER_laser_reply_handler( TCX_REF_PTR            ref,
				 LASER_laser_reply_ptr laser)
{
    int i ;
#ifdef USER_debug
    fprintf(stderr, "TCX: Received a LASER_laser_reply message.\n");
#endif   
    if (laser_f_reading==NULL) 
    {
        laser_f_numberOfReadings = laser->f_numberOfReadings ;
        laser_f_startAngle = laser->f_startAngle;
        laser_f_angleResolution = laser->f_angleResolution ;
        laser_f_reading = (int *)malloc ( laser->f_numberOfReadings * sizeof(int));
    }
    if (pub_laser_f_reading==NULL) 
        pub_laser_f_reading = (int *)malloc ( laser->f_numberOfReadings * sizeof(int));
    if (laser_r_reading==NULL)
    {
        laser_r_angleResolution = laser->r_angleResolution;;
        laser_r_numberOfReadings = laser->r_numberOfReadings;
        laser_r_reading = (int *)malloc ( laser->r_numberOfReadings * sizeof(int));
        laser_r_startAngle = laser->r_startAngle;
    } 
    if (pub_laser_r_reading==NULL)
        pub_laser_r_reading = (int *)malloc ( laser->f_numberOfReadings * sizeof
(int)); 
    /* Get the front readings. */
    mutex_laser_flag[0] = 1 ;
    while(mutex_laser_flag[1] && mutex_laser_stuck==0) 
    for (i = 0; i < laser->f_numberOfReadings; i++)
        laser_f_reading[i] = laser->f_reading[i] ;
    /* Get the rear readings. */
    for (i = 0; i < laser->r_numberOfReadings; i++) 
        laser_r_reading[i] = laser->r_reading[i] ;
    mutex_laser_flag[0] = 0 ;
    tcxFree("LASER_laser_reply", laser); ;
    if(UserLaserCallbackHandler!=NULL)
        (* UserLaserCallbackHandler)(laser_f_numberOfReadings, 
                                     laser_f_angleResolution, 
                                     laser_f_startAngle, 
                                     laser_f_reading,
                                     laser_r_numberOfReadings,
                                     laser_r_angleResolution,
                                     laser_r_startAngle,
                                     laser_r_reading) ;
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_image_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int waitingForCameraImageReply = 0 ;
/* When beeCameraRequestImage() ( defined later ) is executed, it will
 * block until the user program receives a CAMERA_image_reply message.
 * It does it by running a
 *     while(waitingForCameraImageReply) ;
 * loop. beeCameraRequestImage() set waitingForCameraImageReply to 1. Then
 * CAMERA_image_reply_handler() reset it to 0. Then beeCameraRequestImage()
 * leaves the loop and return.
 */
void CAMERA_image_reply_handler(TCX_REF_PTR             ref,
				CAMERA_image_reply_ptr cameraImage)
{
    int numGrabber = cameraImage->numGrabber; 
    cameraImageReply.xsize = cameraImage->xsize;
    cameraImageReply.ysize = cameraImage->ysize;
    cameraImageReply.red = cameraImage->red ;
    cameraImageReply.green = cameraImage->green ;
    cameraImageReply.blue = cameraImage->blue ;
    cameraImageReply.numGrabber = cameraImage->numGrabber;
    waitingForCameraImageReply = 0 ;
    if (UserCameraCallbackHandler) 
        (* UserCameraCallbackHandler) (&cameraImageReply) ;
    tcxFree("CAMERA_image_reply", cameraImage);
}

/* When beeCameraRequestShmId() ( defined later ) is executed, it will
 * block until the user program receives a CAMERA_shmid_reply message.
 * It does it by running a
 *     while(waitingForCameraShmIdReply) ;
 * loop. beeCameraLoadFile() set waitingForCameraShmIdReply to 1. Then
 * CAMERA_shmid_reply_handler() reset it to 0. Then beeCameraRequestShmId()
 * leaves the loop and return.
 */
int waitingForCameraShmIdReply = 0 ;
int cameraReplyShmId = 0 ;
void CAMERA_shmid_reply_handler( TCX_REF_PTR            ref,
				 CAMERA_shmid_reply_ptr shmid)
{ 
    cameraReplyShmId  = shmid->shmid; 
    waitingForCameraShmIdReply = 0 ;
 
    tcxFree("CAMERA_shmid_reply", shmid);

}

void CAMERA_load_reply_handler( TCX_REF_PTR           ref,
				CAMERA_load_reply_ptr cameraFile)
{
    tcxFree("CAMERA_load_reply", cameraFile);
}



void LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
    unsigned int rob;
  
#ifdef USER_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
#endif
  
    /* Set the new correction parameters. */
    correction.x = status->corrX;
    correction.y = status->corrY;
    correction.rot = status->corrRot;
    correction.type = status->corrType;
    if (approach_des_x || approach_des_y) 
    {
        BASE_goto_absolute_type goto_absolute_type ;
        goto_absolute_type.new_target = 1 ;
        goto_absolute_type.abs_target_x  = approach_des_x ;
        goto_absolute_type.abs_target_y  = approach_des_y ;
        tcxSendMsg(BASE, "BASE_goto_absolute", &goto_absolute_type);
    }
  
    if (status->numberOfLocalMaxima < 3)
        correctionParametersKnown = TRUE;
    
    else
        correctionParametersKnown = FALSE;

    tcxFree("LOCALIZE_update_status_reply", status); 
}

void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
                            LOCALIZE_map_reply_ptr map)
{
    tcxFree("LOCALIZE_map_reply", map);
}

void
LOCALIZE_samples_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_samples_reply_ptr samples)
{
  tcxFree("LOCALIZE_samples_reply", samples);
}



void PLAN_action_reply_handler(TCX_REF_PTR              ref,
                               PLAN_action_reply_ptr    action)
{
    tcxFree("PLAN_action_reply", action);
}

void PLAN_status_reply_handler(TCX_REF_PTR              ref,
                               PLAN_status_reply_ptr    status)
{
    tcxFree("PLAN_status_reply", status);
}


/**** the following handlers won't be used! *****/

void BASE_init_robot_reply_handler( TCX_REF_PTR  ref,
				    int          *data)
{
    tcxFree("BASE_init_robot_reply", data);  
}

void BASE_robot_position_reply_handler( TCX_REF_PTR                   ref,
					BASE_robot_position_reply_ptr pos)
{
    tcxFree("BASE_robot_position_reply", pos); 
}

void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
    tcxFree("BASE_action_executed_reply", data);
}
/*************************************************************************
 *
 * The recvLoopThread will be running in a separate thread.
 * The procedure is to collect and dispatch sensor data in the background
 * without halting user's foreground task.
 * The thread is created inside eeInitialize() defined in latter context 
 *
 ******************************************************************/ 

static void recvLoopThread(void) 
{
    struct timeval TCX_waiting_time = {0, 0};
    while(1) 
        tcxRecvLoop((void *) &TCX_waiting_time); 
}

/************************************************************************
 *
 *
 *   BeeSoft Light new created Application Programming Interface library 
 *
 *
 *
 ************************************************************************/

/************************************************************************
 *
 *   NAME:         beeInitialize()
 *                 
 *   FUNCTION:     initializes TCX communication and connects
 *                 to the BASE, LOCALIZE, and PLAN
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void beeInitialize(char *moduleName)
{
    char* tcxMachine = NULL;
  
    LOCALIZE_register_auto_update_type localizeUpdate;

    TCX_REG_MSG_TYPE TCX_message_array[] = {
        BASE_messages,
        SONAR_messages,
        LASER_messages,
        PLAN_messages,
        LOCALIZE_messages, 
        CAMERA_messages
    } ;

    pthread_t recvLoopThreadStructure ;

    if (moduleName == NULL) 
      moduleName = TCX_USER_MODULE_NAME;
 
    baseUpdate.subscribe_status_report = 1;
    baseUpdate.subscribe_sonar_report  = 0;
    baseUpdate.subscribe_colli_report  = 0;
    baseUpdate.subscribe_laser_report  = 0;
    baseUpdate.subscribe_ir_report     = 0;
  

    fprintf(stderr, "Connecting to TCX...");
    tcxMachine = (char *) getenv("TCXHOST");   

    if (!tcxMachine) {
        fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
        tcxMachine="localhost";
    }

    tcxInitialize(moduleName, (void *) tcxMachine);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
                       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
                       NULL, 0);
    fprintf(stderr, "done.\n");

  /*.. if this doesn't work, you might either have to start tcxServer,
   * or you forgot to set the environment variable TCXHOST appropriately:
   * e.g.       setenv TCXHOST caligula
   */

  
    tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		        / sizeof(TCX_REG_MSG_TYPE));
  

    tcxRegisterHandlers(BASE_reply_handler_array,
		        sizeof(BASE_reply_handler_array)
		        / sizeof(TCX_REG_HND_TYPE));

    tcxRegisterHandlers(SONAR_reply_handler_array,
		        sizeof(SONAR_reply_handler_array)
		        / sizeof(TCX_REG_HND_TYPE));
  
    tcxRegisterHandlers(LASER_reply_handler_array,
		        sizeof(LASER_reply_handler_array)
		        / sizeof(TCX_REG_HND_TYPE));

    tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

    tcxRegisterHandlers(CAMERA_reply_handler_array,
		      sizeof(CAMERA_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
 
    tcxRegisterCloseHnd(USER_close_handler);


    fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
//    BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME);
    fprintf(stderr, "done.\n");

    fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
    LOCALIZE = tcxConnectOptional(TCX_LOCALIZE_MODULE_NAME);
    fprintf(stderr, "done.\n");
    if(LOCALIZE)  
          localizeUpdate.subscribe = 1;
    else localizeUpdate.subscribe = 0 ;
    

    fprintf(stderr, "Connecting to %s...", TCX_PLAN_MODULE_NAME);
    PLAN = tcxConnectOptional(TCX_PLAN_MODULE_NAME);
    fprintf(stderr, "done.\n");
   
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);

    if(localizeUpdate.subscribe)
        tcxSendMsg(LOCALIZE, "LOCALIZE_register_auto_update", &localizeUpdate);
  

    if( pthread_create( &recvLoopThreadStructure, 
                        NULL,
			(void *) recvLoopThread, NULL) == 0)
    {
#ifdef USER_debug
         fprintf(stderr,"Create thread successfully\n") ;
#endif 
    }
#ifdef USER_debug
    else fprintf(stderr,"Fail to create thread\n") ;
#endif

    signal(SIGTERM, &interrupt_handler); /* kill interupt handler */
    signal(SIGINT,  &interrupt_handler); /* control-C interupt handler */ 
   
    tcx_initialized = TRUE ;
}


/*******************************************************************
*                                                                 *
*                        MOTION FUNCTIONS                         *
*                                                                 *
*******************************************************************/

/* move a BeeSoft Light robot along a straight line */
bool beeTranslateBy(float distance)
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
     fprintf(stderr, "\nTranslate Robot %f ", distance);
#endif
     tcxSendMsg(BASE, "BASE_translate_by", &distance) ;
     return TRUE ;
}  

/* move a BeeSoft Light robot along a straight line */
bool beeTranslatePositive(void) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "Translate Positive\n");
#endif
    tcxSendMsg(BASE, "BASE_translate_forward", NULL);
    return TRUE ;

}

/* move a BeeSoft Light robot along a straight line */
bool beeTranslateNegative(void) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
        fprintf(stderr, "Translate Negative\n");
#endif
    tcxSendMsg(BASE, "BASE_translate_backward", NULL);
    return TRUE ;
   
}


/* rotate a BeeSoft Light robot's orientation */
bool beeRotateBy(float degree)
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
        fprintf(stderr, "Rotate rotation %f\n", degree);
#endif
    tcxSendMsg(BASE, "BASE_rotate_by", &degree);
    return TRUE ;
}
 
/* rotate a BeeSoft Light robot's orientation */
bool beeRotatePositive(void) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
        fprintf(stderr, "Rotate clockwise\n");
#endif
    tcxSendMsg(BASE, "BASE_rotate_clockwise", NULL);
    return TRUE ;
    
}

/* rotate a BeeSoft Light robot's orientation */
bool beeRotateNegative(void) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "Rotate anticlockwise\n");
#endif
    tcxSendMsg(BASE, "BASE_rotate_anticlockwise", NULL);
    return TRUE ;
 
}

/* set the translation velocity of a BeeSoft Light robot */
bool beeSetTransVelocity(float velocity) 
{
    BASE_set_velocity_type velocity_type ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
    velocity_type.trans_velocity = velocity ;
    velocity_type.rot_velocity = ROBOT_STATE.rot_current_speed;     
    ROBOT_STATE.trans_current_speed = velocity;
    tcxSendMsg(BASE, "BASE_set_velocity", &velocity_type);
    return TRUE ;
 
}

/* set the rotation velocity of a BeeSoft Light robot */
bool beeSetRotVelocity(float velocity)
{
    BASE_set_velocity_type velocity_type ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    velocity_type.trans_velocity = ROBOT_STATE.trans_current_speed ;
    velocity_type.rot_velocity = velocity ;     
    ROBOT_STATE.rot_current_speed = velocity;
    tcxSendMsg(BASE, "BASE_set_velocity", &velocity_type);
    return TRUE ;

}

/* set the translation acceleration of a BeeSoft Light robot */
bool beeSetTransAcceleration(float acceleration) 
{
    BASE_set_acceleration_type acceleration_type ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
    acceleration_type.trans_acceleration = acceleration ;
    acceleration_type.rot_acceleration = ROBOT_STATE.rot_current_acceleration ;     
    tcxSendMsg(BASE, "BASE_set_acceleration", &acceleration_type);
    return TRUE ;
 
}
 
/* set the rotation acceleration of a BeeSoft Light robot */
bool beeSetRotAcceleration(float acceleration)
{
    BASE_set_acceleration_type acceleration_type ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }

    acceleration_type.trans_acceleration = ROBOT_STATE.trans_current_acceleration ;
    acceleration_type.rot_acceleration = acceleration ;     
    tcxSendMsg(BASE, "BASE_set_acceleration", &acceleration_type);
    return TRUE ;

}

/* stop a BeeSoft Light robot when it is moving */
bool beeStop() 
{
    int stop = 1 ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
#ifdef USER_debug
        fprintf(stderr, "Stop\n");
#endif
    tcxSendMsg(BASE, "BASE_stop_robot", NULL);
    tcxSendMsg(PLAN, "PLAN_stop_autonomous_message", &stop);
    return TRUE ;
    
}

/* move a BeeSoft Light robot to a relative position */
bool beeApproachRelative(float rel_target_x,            
                         float rel_target_y) 
{
    BASE_goto_relative_type gotor_type ;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
#ifdef USER_debug
        fprintf(stderr, "Approach Relative %f %f\n",
                rel_target_x, rel_target_y);
#endif
    gotor_type.rel_target_x = - rel_target_x ;
    gotor_type.rel_target_y = rel_target_y ;
   
    tcxSendMsg(BASE, "BASE_goto_relative", &gotor_type);
    return TRUE ;
 
}

/* move a BeeSoft Light robot to an absolute position without plan's assistance*/

bool beeApproachAbsolute(float target_x, float target_y) 
{
    BASE_goto_absolute_type goto_absolute_type ;
    float rotation ;
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
        fprintf(stderr, "Approach Absolute %f %f \n",
                target_x, target_y);
#endif
    if(LOCALIZE == FALSE)
    {   
        fprintf(stderr, "LOCALIZE process is not connected\n") ;
        return FALSE ;   
    } 
    if ( known_pos && correctionParametersKnown)
    {
            /* Get the new corrected position of the robot. */
            // compute_backward_correction
            compute_backward_correction(target_x,
   			             target_y,
	  		             0,
			             correction.x,
			             correction.y,
			             correction.rot,
			             correction.type,
			             &goto_absolute_type.abs_target_x,
			             &goto_absolute_type.abs_target_y,
			             &rotation);  

    }
    else
    {
         return FALSE ;    
    } 
    goto_absolute_type.new_target = 1 ;
    approach_des_x = goto_absolute_type.abs_target_x ;
    approach_des_y = goto_absolute_type.abs_target_y ;
    tcxSendMsg(BASE, "BASE_goto_absolute", &goto_absolute_type);
    return TRUE ;
 
}
/* move a BeeSoft Light robot to an absolute position with plan's assistance*/
bool beeGotoRelative(float rel_target_x, float rel_target_y)
{
    PLAN_goal_message_type goal_message_type ;
    float rotation ;
    float temp_x ;
    float temp_y ;
    float temp_rotation ;
    float orientation;
    int exploration; 
 
    if(tcx_initialized == FALSE)
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;
    }

#ifdef USER_debug
        fprintf(stderr, "Goto Relative %f %f \n",
                rel_target_x, rel_target_y);
#endif
    if(LOCALIZE == FALSE)
    {
        fprintf(stderr, "LOCALIZE process is not connected\n") ;
        return FALSE ;
    }
    if(PLAN == FALSE)
    {
        fprintf(stderr, "plan process is not connected\n") ;
        return FALSE ;
    }
    compute_forward_correction(ROBOT_STATE.pos_x,
                               ROBOT_STATE.pos_y,
                               ROBOT_STATE.orientation,
                               correction.x,
                               correction.y,
                               correction.rot,
                               correction.type,
                               &temp_x,
                               &temp_y,
                               &temp_rotation);  
   orientation = ROBOT_STATE.orientation * M_PI / 180.0;
#ifdef USER_debug
    fprintf(stderr, "x = %f y = %f relx = %f rely = %f orientation = %f %f",
      temp_x, temp_y, rel_target_x, rel_target_y, ROBOT_STATE.orientation,
orientation) ;
#endif
    goal_message_type.x = temp_x + rel_target_x * cos(orientation) 
                                 + rel_target_y * sin(orientation) ;
    goal_message_type.y = temp_y - rel_target_x * sin(orientation)
                                 + rel_target_y * cos(orientation) ;
#ifdef USER_debug
    fprintf(stderr, "new location %f %f", goal_message_type.x, goal_message_type.y) ;
#endif
    goal_message_type.name = 0 ;
    goal_message_type.add = 1 ;
    exploration = 0 ; 
    tcxSendMsg(PLAN, "PLAN_goal_message", &goal_message_type);
    tcxSendMsg(PLAN, "PLAN_start_autonomous_message", &exploration);
    return TRUE ;

} 
/* move a BeeSoft Light robot to an absolute position with plan's assistance*/
bool beeGotoAbsolute(float target_x, float target_y) 
{
    PLAN_goal_message_type goal_message_type ;
    float rotation ;
    int exploration;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not connected\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not connected\n") ;
        return FALSE ;   
    }
    
#ifdef USER_debug
        fprintf(stderr, "Goto Absolute %f %f \n",
                target_x, target_y);
#endif
    if(LOCALIZE == FALSE)
    {
        fprintf(stderr, "LOCALIZE process is not connected\n") ;
        return FALSE ;
    }
    if(PLAN == FALSE)
    {
        fprintf(stderr, "plan process is not connected\n") ;
        return FALSE ;
    }
         
    goal_message_type.x = target_x ; 
    goal_message_type.y = target_y ;
    goal_message_type.name = 0 ;
    goal_message_type.add = 1 ;
    exploration = 0 ;
    tcxSendMsg(PLAN, "PLAN_goal_message", &goal_message_type);     
    tcxSendMsg(PLAN, "PLAN_start_autonomous_message", &exploration);  
    return TRUE ;

 
}
/*******************************************************************
*                                                                 *
*                        SENSORS FUNCTIONS                        *
*                                                                 *
*******************************************************************/


typedef struct {
    float pos_x ;
    float pos_y ;
    float orientation ;
} robot_position ;

/* The location of pubRobotPosition is returned to user process every time
 * beeGetCurrentPosition() is invoked. 
 */
robot_position pubRobotPosition ;
typedef struct {
    float trans_speed ;
    float rot_speed ;
} robot_speed ;
robot_speed pubRobotSpeed ;
/* The location of pubRobotSpeed is returned to user process every time
 * beeGetCurrentSpeed() is invoked.
 */
/* provide current position of a BeeSoft Light robot */
robot_position * beeGetCurrentPosition() 
{
    float temp_x ;
    float temp_y ;
    float temp_rotation ;
    if(known_pos)
    {
        mutex_ROBOT_status_flag[1] = 1 ;
        mutex_ROBOT_status_stuck = 0 ;
        while(mutex_ROBOT_status_flag[0]) 
            mutex_ROBOT_status_stuck = 1 ;
        pubRobotPosition.pos_x = ROBOT_STATE.pos_x ;
        pubRobotPosition.pos_y = ROBOT_STATE.pos_y ;
        pubRobotPosition.orientation = ROBOT_STATE.orientation ;
        mutex_ROBOT_status_flag[1] = 0 ;
        return &pubRobotPosition ;
    }
    else
    {
        return NULL ;
    }
}
                                  
/* provide current speed of a BeeSoft Light robot */
robot_speed * beeGetCurrentSpeed() 
{
    if(known_pos)
    {

        mutex_ROBOT_status_flag[1] = 1 ;
        mutex_ROBOT_status_stuck = 0 ;
        while(mutex_ROBOT_status_flag[0])
            mutex_ROBOT_status_stuck = 1 ;
        pubRobotSpeed.trans_speed = ROBOT_STATE.trans_current_speed ;
        pubRobotSpeed.rot_speed = ROBOT_STATE.rot_current_speed ;
        mutex_ROBOT_status_flag[1] = 0 ;
        return &pubRobotSpeed ;
    }
    else
    {
        return NULL ;
    }
}

/* trigger to regularly update sonar data of a BeeSoft Light robot */
bool beeSonarStartRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }
  
#ifdef USER_debug
        fprintf(stderr, "beeSonarStartRegularUpdate");
#endif
    baseUpdate.subscribe_sonar_report  = 1;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    return TRUE ;
 
}

/* stop the regular update operations */
bool beeSonarStopRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeSonarStoptRegularUpdate");
#endif
    baseUpdate.subscribe_sonar_report  = 0;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    known_sonar_val = FALSE ;
    return TRUE ;

}

/* get sonar value */
float * beeGetSonarValue(int * number_of_reading)          // Get sonar value
{  
    int i ;
    if(known_sonar_val)
    {
        *number_of_reading = 24 ;
        mutex_sonar_flag[1] = 1 ;
        mutex_sonar_stuck = 0 ;
        while(mutex_sonar_flag[0]) 
            mutex_sonar_stuck = 1 ;
        for (i=0 ; i< 24 ; i++)
            pub_sonar_values[i] = sonar_values[i] ;
        mutex_sonar_flag[1] = 0 ;
 
        return pub_sonar_values ;
    }
    else
    {
#ifdef USER_debug
        fprintf(stderr, "Sonar value is unknown") ;
#endif 
        *number_of_reading = 0 ; 
        return NULL ;   
    }
}

/* register a callback function to handle sonar data of a BeeSoft Light robot */
bool beeSonarRegisterCallback(void (*callbackHandler)(float*)) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }    

#ifdef USER_debug
        fprintf(stderr, "beeSonarRegisterCallback");
#endif
        
    UserSonarCallbackHandler = callbackHandler ;
    return TRUE ;


}

/* trigger to regularly update infrared data of a BeeSoft Light robot */
bool beeIRStartRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeIRStartRegularUpdate");
#endif
    baseUpdate.subscribe_ir_report  = 1;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    return TRUE ;

}

/* stop the regular update operations */
bool beeIRStopRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeIRStopRegularUpdate");
#endif
    baseUpdate.subscribe_ir_report  = 0;
    known_ir_val = FALSE ;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    return TRUE ;

}

/* get infrared value */
int* beeGetIRValue(int rowno, int * numberofreading)     
{  
    int i ;
    if(known_ir_val)
    {
        mutex_ir_flag[1] = 1 ;
        mutex_ir_stuck = 0 ;
        while(mutex_ir_flag[0])
            mutex_ir_stuck = 1 ;
        switch(rowno)
        {
           case UPPERROW: *numberofreading = 24 ;
                          for (i=0 ; i< 24 ; i++)
                             pub_ir_upperrow_values[i] = ir_upperrow_values[i] ;
                          mutex_ir_flag[1] = 0 ;
                          return pub_ir_upperrow_values ;
           case LOWERROW: *numberofreading = 24 ;
                          for (i=0 ; i< 24 ; i++)
                             pub_ir_lowerrow_values[i] = ir_lowerrow_values[i] ;
                          mutex_ir_flag[1] = 0 ;
                          return pub_ir_lowerrow_values ; 
           case DROW:     *numberofreading = 8 ;
                          for (i=0 ; i< 8 ; i++)
                             pub_ir_drow_values[i] = ir_drow_values[i] ;
                          mutex_ir_flag[1] = 0 ;
                          return pub_ir_drow_values ; 
           default:   *numberofreading = 0 ;
                      return NULL ;
        }
    }
    else
    {
#ifdef USER_debug
        fprintf(stderr, "\nIR value is unknown") ;
#endif
        *numberofreading = 0 ; 
        return NULL ;   
    }
}

/* register a callback function to handle infrared data of a BeeSoft Light robot */
bool beeIRRegisterCallback(void (*callbackHandler)(int*, int*, int*)) 
{

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }    

#ifdef USER_debug
        fprintf(stderr, "beeIRRegisterCallback");
#endif
        
    UserIRCallbackHandler = callbackHandler;
    return TRUE ;

}

/* trigger to regularly update laser data of a BeeSoft Light robot */
bool beeLaserStartRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeLaserStartRegularUpdate");
#endif
    baseUpdate.subscribe_laser_report  = 1;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    return TRUE ;
    
}

/* stop the regular update operations */
bool beeLaserStopRegularUpdate()
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeLaserStopRegularUpdat");
#endif
    baseUpdate.subscribe_laser_report  = 0;
    tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
    return TRUE ;
}

/* get laser value */
int* beeGetLaserValue(int whichone, int * numberofreading)  
{  
    int i ;
    switch(whichone)
    {
        case FRONT_LASER:   
            if(laser_f_reading!= NULL) 
            {
                mutex_laser_flag[1] = 1 ;
                mutex_laser_stuck = 0 ;
                while(mutex_laser_flag[0])
                      mutex_laser_stuck = 1 ;
                *numberofreading = laser_f_numberOfReadings ;
                for (i = 0; i < laser_f_numberOfReadings; i++)
                    pub_laser_f_reading[i] = laser_f_reading[i] ;
                mutex_laser_flag[1] = 0 ;
                return pub_laser_f_reading ;
            }
            else
            {
#ifdef USER_debug
                fprintf(stderr, "Laser value is not known") ;
#endif
                *numberofreading = 0 ;
                return NULL ;   
            }
            break ;
        case REAR_LASER:
             if(laser_r_reading!= NULL)
            {
                mutex_laser_flag[1] = 1 ;
                mutex_laser_stuck = 0 ;
                while(mutex_laser_flag[0])
                      mutex_laser_stuck = 1 ;
                *numberofreading = laser_r_numberOfReadings ;
                for (i = 0; i < laser_r_numberOfReadings; i++)
                    pub_laser_r_reading[i] = laser_r_reading[i] ;
                mutex_laser_flag[1] = 0 ;
                return pub_laser_r_reading ; 
            }
            else
            {
#ifdef USER_debug
                fprintf(stderr, "Laser value is not known") ;
#endif
                *numberofreading = 0 ;
                return NULL ;
            }
            break ; 
    }
}

/* register a callback function to handle laser data of a BeeSoft robot  */
bool beeLaserRegisterCallback(void (*callbackHandler)(int, float, float, int*,  
                                    int, float, float, int*)) 
{
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if(BASE == FALSE)
    {
        fprintf(stderr, "colliBase Server is not running\n") ;
        return FALSE ;   
    }    

#ifdef USER_debug
        fprintf(stderr, "beeSonarRegisterCallback");
#endif
        
  
    UserLaserCallbackHandler = callbackHandler ;  
    return TRUE ;

}

bool beeCameraConnect()
{
    struct timeval this_time;
    float  time_difference;
    if(tcx_initialized == FALSE)
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;
    }

    fprintf(stderr, "Connecting to Camera server...\n");
    CAMERA = tcxConnectModule(TCX_CAMERA_MODULE_NAME);
    fprintf(stderr, "Camera Connected.\n");

    return TRUE;
}


/* trigger to regularly update camera data of a BeeSoft Light robot */
bool beeCameraSubscribe(int numGrabber, int numberOfImage,
                        int image_xmin,  int image_ymin,
                        int image_xsize, int image_ysize,
                        int return_image_xsize, int return_image_ysize)
{
    CAMERA_register_auto_update_type subscribe;
    CAMERA_startstop_type data;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
#ifdef USER_debug
        fprintf(stderr, "beeCameraStartRegularUpdate");
#endif
    if (CAMERA == NULL)
    {
        fprintf(stderr, "Run beeCameraConnect() first \n") ;
        return FALSE ;
    }
    subscribe.numGrabber         = numGrabber;
    subscribe.image              = numberOfImage;
    subscribe.orig_image_xmin    = image_xmin;
    subscribe.orig_image_ymin    = image_ymin;
    subscribe.orig_image_xsize   = image_xsize;
    subscribe.orig_image_ysize   = image_ysize;
    subscribe.return_image_xsize = return_image_xsize;
    subscribe.return_image_ysize = return_image_ysize;

    tcxSendMsg(CAMERA, "CAMERA_register_auto_update", &subscribe);

    data.numGrabber = numGrabber;
    data.action     = 1;
    tcxSendMsg(CAMERA, "CAMERA_startstop", &data);
    return TRUE ;

}

/* stop the regular update operations */
bool beeCameraUnsubscribe(int numGrabber )
{
    CAMERA_register_auto_update_type subscribe;
    CAMERA_startstop_type data;

    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }

#ifdef USER_debug
        fprintf(stderr, "beeCameraStopRegularUpdat");
#endif
    
    if (CAMERA)
    {
        subscribe.numGrabber         = numGrabber;
        subscribe.image              = 0 ;
        tcxSendMsg(CAMERA, "CAMERA_register_auto_update", &subscribe);
        data.numGrabber = numGrabber;
        data.action     = 0;
        tcxSendMsg(CAMERA, "CAMERA_startstop", &data);
        return TRUE ;

    } 
    else 
    {
        fprintf(stderr, "cameraServer is not connected\n");
        return FALSE ;
    }
}


/* register a callback function to handle camera image */
bool beeCameraImageRegisterCallback(void (*callbackHandler)(CameraImageType *) ) 
{
#ifdef USER_debug
        fprintf(stderr, "beeCameraRegisterCallback");
#endif
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;   
    }
    if (CAMERA)
    {
        UserCameraCallbackHandler = callbackHandler ;  
        return TRUE ;
    }
    else
    {
        fprintf(stderr, "cameraServer is not connected\n");
        return FALSE ;
    }

}

CameraImageType* beeCameraRequestImage( int numGrabber, 
		            int orig_image_xmin,
		            int orig_image_ymin,
		            int orig_image_xsize,
		            int orig_image_ysize,
		            int return_image_xsize,
		            int return_image_ysize)
{
    CAMERA_image_query_type specs;
    if(tcx_initialized == FALSE) 
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return NULL ;   
    }
    if (CAMERA == NULL) 
    {
        fprintf(stderr, "cameraServer is not connected\n");
        return NULL ;
    }
    specs.numGrabber = numGrabber;
    specs.orig_image_xmin = orig_image_xmin;
    specs.orig_image_ymin = orig_image_ymin;
    specs.orig_image_xsize = orig_image_xsize;
    specs.orig_image_ysize = orig_image_ysize;
    specs.return_image_xsize = return_image_xsize;
    specs.return_image_ysize = return_image_ysize;
    waitingForCameraImageReply = 1 ;
    tcxSendMsg(CAMERA, "CAMERA_image_query", &specs);
    while(waitingForCameraImageReply)
       sleep(1)  ;
    return &cameraImageReply ;

}


/* return value is the shared memory id of new retrieved image */
int  beeCameraRequestShmId(int numGrabber )
{

    CAMERA_shmid_query_type specs;
    if(tcx_initialized == FALSE)
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return 0 ;
    }

    if (CAMERA)
    {
        specs.numGrabber = numGrabber;
        waitingForCameraShmIdReply = 1 ;
        tcxSendMsg(CAMERA, "CAMERA_shmid_query", &specs);
        while(waitingForCameraShmIdReply) 
        {
          sleep(1) ;
        }
        return cameraReplyShmId ;
    }
    else
    {
        fprintf(stderr, "cameraServer is not connected. \n");
        return 0 ;
    }

}

bool beeCameraSaveFile(int numGrabber, char *filename, int num )
{

    CAMERA_save_type data;
    if(tcx_initialized == FALSE)
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;
    }

    if (CAMERA){

        data.numGrabber = numGrabber;

        sprintf( data.filename, "%s", filename );
        data.frames = num; 

        tcxSendMsg(CAMERA, "CAMERA_save", &data);
        return TRUE ; 
    } 
    else 
    {
        fprintf(stderr, "cameraServer is not connected. \n");
        return FALSE ;
    }

}


bool beeCameraLoadFile(int numGrabber, char *filename ) 
{

    CAMERA_load_type data;
    if(tcx_initialized == FALSE)
    {
        fprintf(stderr, "TCX Server is not running\n") ;
        return FALSE ;
    }
    if (CAMERA)
    {
        data.numGrabber= numGrabber;
        sprintf( data.filename, "%s", filename );
        /* Set waitingForCameraLoadReply to 1. Then the function blocks.
         * Then the reply handler reset it to 0. Then the function returns.
         */
        tcxSendMsg(CAMERA, "CAMERA_load", &data);
        return TRUE ;
    } 
    else 
    {
        fprintf(stderr, "cameraServer is not connected\n");
        return FALSE ;
    }

}

