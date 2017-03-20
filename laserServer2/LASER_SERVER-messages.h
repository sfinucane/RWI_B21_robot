#ifndef LASER_SERVER_messages_defined
#define LASER_SERVER_messages_defined

/* 
  Notice: when including this file, you need to have the flag
  
  TCX_define_variables
  
  be defined exactly once. This will allocate memory for the
  module pointer and the message arrays.
  */


#define TCX_LASER_SERVER_MODULE_NAME "laserServer"
#ifdef TCX_define_variables   /* do this exactly once! */
TCX_MODULE_PTR LASER_SERVER;	      /* needs to be allocate in a user program */
#else
extern TCX_MODULE_PTR LASER_SERVER;  /* otherwise: reference */
#endif

/* -------------------------------------------------- */
/* Messages                                           */
/* -------------------------------------------------- */

typedef struct {
  int sweep0;	/* laser0 - 0=don't send, n>0 send every n-th frame */
  int sweep1;   /* laser1 - 0=don't send, n>0 send every n-th frame */
} LASER_SERVER_register_auto_update_type, *LASER_SERVER_register_auto_update_ptr;

#define LASER_SERVER_register_auto_update_format "{ int, int }"

/* ----------------------------------------------------------------------- */

typedef struct {
  int numLaser;      /* query which laser, num = 0 | 1 */
} LASER_SERVER_sweep_query_type, *LASER_SERVER_sweep_query_ptr;

#define LASER_SERVER_sweep_query_format "{ int }"

typedef struct {
  int numberLasers; /* will be set to NUMBER_LASERS in laserHandlers.c */
  int numLaser;     /* which laser */
  float *value;
  struct timeval timeStamp;
} LASER_SERVER_sweep_reply_type, *LASER_SERVER_sweep_reply_ptr;

#define LASER_SERVER_sweep_reply_format "{ int, int, <float : 1>,  {long, long} }"

/* ----------------------------------------------------------------------- */

#ifdef TCX_define_variables		/* do this exactly once! */
#define LASER_SERVER_messages \
{"LASER_SERVER_register_auto_update", LASER_SERVER_register_auto_update_format},\
{"LASER_SERVER_sweep_reply",          LASER_SERVER_sweep_reply_format},\
{"LASER_SERVER_sweep_query",          LASER_SERVER_sweep_query_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/******* (a) Procedure headers ******/

void LASER_SERVER_sweep_reply_handler( TCX_REF_PTR            ref,
				 LASER_SERVER_sweep_reply_ptr sweep);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE LASER_SERVER_reply_handler_array[] = {

  {"LASER_SERVER_sweep_reply", "LASER_SERVER_sweep_reply_handler",
     (void *) LASER_SERVER_sweep_reply_handler, TCX_RECV_ALL, NULL}

};

#endif /* define reply handlers */

#endif /* messages defined */








