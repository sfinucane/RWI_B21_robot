/*
 * $Id: CAMERA-messages.h,v 1.1 2002/09/14 16:37:46 rstone Exp $
 *
 */
#ifndef CAMERA_messages_defined
#define CAMERA_messages_defined

/* 
  Notice: when including this file, you need to have the flag
  
  TCX_define_variables
  
  be defined exactly once. This will allocate memory for the
  module pointer and the message arrays.
  */


#define TCX_CAMERA_MODULE_NAME "cameraServer"
#ifdef TCX_define_variables	/* do this exactly once! */
TCX_MODULE_PTR CAMERA;		/* needs to be allocate in a user program */
#else
extern TCX_MODULE_PTR CAMERA;	/* otherwise: reference */
#endif

/* -------------------------------------------------- */
/* Messages                                           */
/* -------------------------------------------------- */

/* image format type definitions: */
#define UNCOMPRESSED_FORMAT 0
#define JPEG_FORMAT 1

/* A NOTE about frame rates:

   The color quickcam and the matrox meteor are the two image sources
   supported by cameraServer.  The quickcam can grab images at about 1
   every 2 seconds or so.  The meteor can grab 15 images per second or
   so.  The "image" parameter below, telling it to send every n-th
   frame needs some attention when you are using different image
   sources, therefore.

   However, sending the images over the network is slow enough that
   the frame rates end up being about the same order of magnitude.
   */
typedef struct {
  int numGrabber;		/* zero for first grabber */
  int image;			/* 0=don't send, n>0 send every n-th frame */
  int orig_image_xmin;
  int orig_image_ymin;
  int orig_image_xsize;
  int orig_image_ysize;
  int return_image_xsize;
  int return_image_ysize;
  int image_format;      /* image format, defined above. */
} CAMERA_register_auto_update_type, *CAMERA_register_auto_update_ptr;

#define CAMERA_register_auto_update_format "{int, int, int, int, int, int, int, int, int}"

/* ----------------------------------------------------------------------- */

typedef struct {
  int numGrabber;		/* zero for first grabber ... */
  int orig_image_xmin;
  int orig_image_ymin;
  int orig_image_xsize;
  int orig_image_ysize;
  int return_image_xsize;
  int return_image_ysize;
  int image_format;      /* image format defined above. */
} CAMERA_image_query_type, *CAMERA_image_query_ptr;

#define CAMERA_image_query_format "{int, int, int, int, int, int, int, int}"

typedef struct {
  int image_format;  /* image format of this image, defined above. */
  int size;	/* total number of bytes in compressed image data*/
  unsigned char *image; /* compressed image data */
} COMPRESSED_IMAGE_TYPE;

#define COMPRESSED_IMAGE_FMT "{int, int, <char:2>}"

/* Notes about the CAMERA_image_reply_type, as returned by cameraServer:

   If compressedImage.image_format is UNCOMPRESSED_FORMAT, size will
   be xsize * ysize, and red, green, and blue will have valid pointers
   to image data.  Also, compressedImage.size will be 0, and
   compressedImage.image will be NULL.

   If compressedImage.image_format is JPEG_FORMAT, size will be 0 and
   red, green, and blue will be NULL.  Of course, compressedImage.size
   will be the size in bytes of the compressed image data, and
   compressedImage.image will be a valid pointer to the compressed
   image data.
   */
typedef struct {
  int                size;	/* total number of bytes        */
  int                xsize;	/* image size (number of pixels)*/
  int                ysize;	/* image size (number of pixels)*/
  unsigned char     *red;	/* image data                   */
  unsigned char     *green;	/* image data                   */
  unsigned char     *blue;	/* image data                   */
  int                numGrabber; /* which grabber took the picture */
  COMPRESSED_IMAGE_TYPE compressedImage;
} CAMERA_image_reply_type, *CAMERA_image_reply_ptr;

#define CAMERA_image_reply_format "{int, int, int, <char:1>, <char:1>, <char:1>, int" "," COMPRESSED_IMAGE_FMT "}"

/* ----------------------------------------------------------------------- */

/* NOTE: Image compression is not available through the shared memory
   interface. */
typedef struct {
  int numGrabber;		/* zero for the first grabber */
} CAMERA_shmid_query_type, *CAMERA_shmid_query_ptr;

#define CAMERA_shmid_query_format "{int}"

typedef struct {
  int shmid;			/* shmid */
  int numGrabber;		/* zero for the first */
} CAMERA_shmid_reply_type, *CAMERA_shmid_reply_ptr;
#define CAMERA_shmid_reply_format "{int,int}"

/* ----------------------------------------------------------------------- */

typedef struct {
  int numGrabber;		/* zero for the first ...  */
  int frames;			/* # of frames to grab, 0=stop, -1=continuous, */
				/* which means grabbing until stop is received */
  char filename[128];		/* name of the file, that stores the frames    */
} CAMERA_save_type, *CAMERA_save_ptr;
#define CAMERA_save_format "{int, int, [char:128] }"

/* ----------------------------------------------------------------------- */

typedef struct {
  char filename[128];
  int numGrabber;
} CAMERA_load_type, *CAMERA_load_ptr;
#define CAMERA_load_format "{ [char:128], int }"

typedef struct {
  int dummy;                                     /* this message will be sent */
} CAMERA_load_reply_type, *CAMERA_load_reply_ptr;/* once we reach the end of  */
#define CAMERA_load_reply_format "{int}"         /* the file.                 */

/* ----------------------------------------------------------------------- */

typedef struct {
  int numGrabber;		/* zero for first grabber */
  int action;			/* action=1 => start else stop */
} CAMERA_startstop_type, *CAMERA_startstop_ptr;
#define CAMERA_startstop_format "{ int, int }"

/* ----------------------------------------------------------------------- */

#ifdef TCX_define_variables		/* do this exactly once! */
#define CAMERA_messages \
{"CAMERA_register_auto_update", CAMERA_register_auto_update_format},\
{"CAMERA_image_reply",          CAMERA_image_reply_format},\
{"CAMERA_shmid_query",          CAMERA_shmid_query_format},\
{"CAMERA_shmid_reply",          CAMERA_shmid_reply_format},\
{"CAMERA_save",                 CAMERA_save_format},\
{"CAMERA_load",                 CAMERA_load_format},\
{"CAMERA_startstop",            CAMERA_startstop_format},\
{"CAMERA_load_reply",           CAMERA_load_reply_format},\
{"CAMERA_image_query",          CAMERA_image_query_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#ifdef DEFINE_REPLY_HANDLERS

/******* (a) Procedure headers ******/

void CAMERA_image_reply_handler( TCX_REF_PTR            ref,
				 CAMERA_image_reply_ptr image);

void CAMERA_shmid_reply_handler( TCX_REF_PTR            ref,
				 CAMERA_shmid_reply_ptr shmid);

void CAMERA_load_reply_handler( TCX_REF_PTR           ref,
				CAMERA_load_reply_ptr data);


/******* (b) Handler array ******/

TCX_REG_HND_TYPE CAMERA_reply_handler_array[] = {

  {"CAMERA_image_reply", "CAMERA_image_reply_handler",
     (void *) CAMERA_image_reply_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_load_reply", "CAMERA_load_reply_handler",
     (void *) CAMERA_load_reply_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_shmid_reply", "CAMERA_shmid_reply_handler",
     (void *) CAMERA_shmid_reply_handler, TCX_RECV_ALL, NULL}

};

#endif /* define reply handlers */

#endif /* messages defined */

/*
 * $Log: CAMERA-messages.h,v $
 * Revision 1.1  2002/09/14 16:37:46  rstone
 * *** empty log message ***
 *
 * Revision 1.19  1999/05/28 20:30:45  thrun
 * necessary modifications for BeeSoft light
 *
 * Revision 1.18  1999/04/21 22:25:00  fox
 * New support for quickcam. Should be ok.
 *
 * Revision 1.16  1998/08/16 19:50:00  thrun
 * can now genrate arbitrary sub-images and downsample them.
 *
 * Revision 1.15  1998/01/13 00:35:11  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.14  1997/10/04 18:01:05  swa
 * Fixed a bug in CAMERA-messages so that sending images over TCX works again.
 *
 * Revision 1.13  1997/10/04 00:13:04  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.12  1997/08/11 15:52:56  swa
 * Changed the name of the module to cameraServer (looks nicer), removed
 * libcameraClient.h from the Makefile. Added beeps.
 *
 * Revision 1.11  1997/07/24 18:48:00  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.10  1997/07/24 00:54:23  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.9  1997/07/23 22:43:31  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.8  1997/07/22 22:47:15  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.7  1997/06/23 23:48:42  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.6  1997/06/23 02:35:58  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.5  1997/06/21 22:36:21  thrun
 * Improved interface, cleaner
 *
 * Revision 1.4  1997/06/20 01:09:28  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.3  1997/06/19 21:06:57  thrun
 * Added various stuff for faceInfo (callbacks, etc)
 *
 * Revision 1.2  1997/06/18 15:58:13  thrun
 * now with auto-update
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
