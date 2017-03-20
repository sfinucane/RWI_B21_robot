
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/faceServer.old/face_interface.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 21:06:45 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: face_interface.h,v $
 * Revision 1.1  2002/09/14 21:06:45  rstone
 * *** empty log message ***
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*****************************************************************************/

#ifndef FACE_INTERFACE_LOADED
#define FACE_INTERFACE_LOADED

#define FACE_PORT "/dev/ttyS3"

#define FACE_BAUDRATE B9600 

#define SSC_SYNC 255

#define BROW0_ADDR 0
#define BROW1_ADDR 7
#define MOUTH0_ADDR 1
#define MOUTH1_ADDR 6
#define EYE0_ADDR 2
#define EYE1_ADDR 5

#define BROW0_CENTER 127
#define BROW1_CENTER 110
#define MOUTH0_CENTER 127
#define MOUTH1_CENTER 127
#define EYE0_CENTER 127
#define EYE1_CENTER 127

#define MAX_EYE_POS 90
#define MIN_EYE_POS -90
#define MAX_BROW_POS 15
#define MIN_BROW_POS -15
#define MAX_MOUTH_POS 80
#define MIN_MOUTH_POS -80

typedef struct {
  int eye0_pos;
  int eye1_pos;
  int brow0_pos;
  int brow1_pos;
  int mouth0_pos;
  int mouth1_pos;

  FILE *fp;
} face_state;

typedef face_state FACE_status_update_type;
extern face_state state;

/* 
   open port for face,
   port and position state are updated,
   returns 1 on failure.
   */
int FaceOpen(face_state *state);

/* 
   close port for face, 
   port state is updated,
   returns 1 if already closed.
   */
int FaceClose(face_state *state);

/* 
   set eye, brow, or mouth position to pos.
   left-right symmetry is imposed.
   bounds on motion are checked and the position is clamped if necessary. 
   position state is updated.
   returns 1 if port is not open.
   */
int FaceSetEyes(face_state *state, int pos);
int FaceSetBrows(face_state *state, int pos);
int FaceSetMouth(face_state *state, int pos);

/*
  completely update face position using values in state.
  bounds on motion are checked and the position is clamped if necessary. 
  position state is updated if clamping occurs.
  returns 1 if port is not open.  
  */
int FaceSetState(face_state *state);

/* 
   set face position to neutral.
   state is updated.
   returns 1 if port is not open.  
   */
int FaceReset(face_state *state);

/* 
   adjust mood (mouth and eyebrow positions) by val.
   positive quantities adjust mood in happy direction, negative in sad dir.
   returns 1 if port is not open.  
   */
int FaceAdjustMood(face_state *state, int val);

/* 
   adjust eye direction by val.
   positive quantities adjust eyes to (robot's) right negative to left
   returns 1 if port is not open.
   */
int FaceAdjustEyes(face_state *state, int val);

#endif
