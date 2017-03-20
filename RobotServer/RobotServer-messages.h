#ifndef RobotServer_messages_defined
#define RobotServer_messages_defined

#include "tcx.h"

#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR RobotServer;             /* needs to be alloc. in a user prog.*/

#else

extern TCX_MODULE_PTR RobotServer;      /* otherwise: reference */

#endif


/**** Message-Typen ****/

typedef struct {

  ulong ID;            /* message ID                           */
  uint  type;          /* type of query 0: get list, 1: get into list */

} RobotServer_query_type, *RobotServer_query_ptr;

#define RobotServer_query_format "{long, int}"


typedef struct {
  
  ulong ID;          /* message ID                            */
  uint  counter;     /* amount of robots alive                */
  unsigned char **robot_list;  /* list of active robots */
} RobotServer_reply_type, *RobotServer_reply_ptr;

#define RobotServer_reply_format "{long, int, <{string} : 2>}"
/*                                                        ^This means that
							  in the second
							  variable of this
							  struct the number of
							  elements in the list
							  is given */

typedef struct {

  ulong ID;            /* message ID                              */
  uint  status;	      /* 0=don't send, n>0 send every n-th frame */
  uint  type;          /* type of query                        */


} RobotServer_register_auto_update_type, *RobotServer_register_auto_update_ptr;
#define RobotServer_register_auto_update_format "{long, int, int}"

typedef struct{
  TCX_MODULE_PTR    module;	/* pointer to TCX module */
  int               status;	/* 1=subscribed to regular status updates */
  int               last_status;
  RobotServer_query_ptr  data;
} RobotServer_auto_update_type;



/**** RobotServer commands - these are the commands/queries understood by RobotServer ****/

#ifdef TCX_define_variables		/* do this exactly once! */

#define RobotServer_messages \
  {"RobotServer_register_auto_update",  RobotServer_register_auto_update_format},\
  {"RobotServer_remove_auto_update",  RobotServer_register_auto_update_format},\
  {"RobotServer_query",    RobotServer_query_format}, \
  {"RobotServer_reply",    RobotServer_reply_format}

#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with RobotServer ******/


/******* (a) Procedure headers ******/
void RobotServer_reply_handler(TCX_REF_PTR ref, RobotServer_reply_ptr data);

/******* (b) Handler array ******/
TCX_REG_HND_TYPE RobotServer_reply_handler_array[] = {
  {"RobotServer_reply", "RobotServer_reply_handler",
   (void (*)()) RobotServer_reply_handler, TCX_RECV_ALL, NULL}
};

#endif

#ifdef DEFINE_QUERY_HANDLERS

/***** QUERY handlers -- these handlers must be defined within the
 ***** program that communicates with RobotServer ******/


/******* (a) Procedure headers ******/

void RobotServer_query_handler(TCX_REF_PTR ref, RobotServer_query_ptr data);
     
/******* (b) Handler array ******/
TCX_REG_HND_TYPE RobotServer_query_handler_array[] =
{
  {"RobotServer_query", "RobotServer_query_handler",
   (void (*)()) RobotServer_query_handler, TCX_RECV_ALL, NULL},
  {"RobotServer_register_auto_update", "RobotServer_register_auto_update_handler",
   RobotServer_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"RobotServer_remove_auto_update", "RobotServer_remove_auto_update_handler",
   RobotServer_remove_auto_update_handler, TCX_RECV_ALL, NULL}
};

#endif

#endif







