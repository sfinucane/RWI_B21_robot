

#ifndef MOUTH_messages_defined
#define MOUTH_messages_defined

#include "tcx.h"

#define TCX_MOUTH_MODULE_NAME "MOUTH"

#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR MOUTH;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR MOUTH;	/* otherwise: reference */

#endif

typedef struct {
  int mood;
} MOUTH_show_mood_type, *MOUTH_show_mood_ptr;

#define MOUTH_show_mood_format "{int}"

typedef struct {
  int mood;
} MOUTH_set_mood_type, *MOUTH_set_mood_ptr;

#define MOUTH_set_mood_format "{int}"

typedef struct {
  int angle;
} MOUTH_set_eyes_type, *MOUTH_set_eyes_ptr;

#define MOUTH_set_eyes_format "{int}"

typedef struct {
  int angle;
} MOUTH_set_mouth_type, *MOUTH_set_mouth_ptr;

#define MOUTH_set_mouth_format "{int}"

typedef struct {
  int eyes;
  int mouth;
} MOUTH_set_face_type, *MOUTH_set_face_ptr;

#define MOUTH_set_face_format "{int, int}"


#ifdef TCX_define_variables		/* do this exactly once! */

#define MOUTH_messages \
  {"MOUTH_show_mood",              MOUTH_show_mood_format},\
  {"MOUTH_set_mood",               MOUTH_set_mood_format},\
  {"MOUTH_set_eyes",               MOUTH_set_eyes_format},\
  {"MOUTH_set_mouth",              MOUTH_set_mouth_format},\
  {"MOUTH_set_face",               MOUTH_set_face_format}

#endif




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS



/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with PLAN ******/


/******* (a) Procedure headers ******/

/******* (b) Handler array ******/
TCX_REG_HND_TYPE MOUTH_reply_handler_array[] = {};

#endif

#endif
