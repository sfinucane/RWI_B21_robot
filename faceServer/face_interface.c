
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/faceServer/face_interface.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 21:06:30 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: face_interface.c,v $
 * Revision 1.1  2002/09/14 21:06:30  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1999/12/28 05:51:04  thrun
 * new head control module. by Mike Montemerlo
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "face_interface.h"
/* #include <sys/time.h> */

int FaceWrite(face_state *state, int addr, int val)
{
  if(!state) return 1;
  fprintf(state->fp,"%c%c%c", SSC_SYNC, addr, val);
  fflush(state->fp);
  return 0;
}

int FaceInit(face_state *state)
{
  printf("Testing face.\n");
  if(!state) return 1;
  printf("Init face.\n");
  state->eye0_pos = state->eye1_pos = 0;
  state->brow0_pos = state->brow1_pos = 0;
  state->mouth0_pos = state->mouth1_pos = 0;  
  return 0;
}

int FaceOpen(face_state *state)
{
  if(!state) return 1;

  printf("Opening face on port %s.\n", FACE_PORT);
  state->fp = fopen(FACE_PORT,"w");
  if (!state->fp) 
    perror("");
  FaceInit(state);

  return (state->fp == NULL);
}

int FaceClose(face_state *state)
{
  if(!state || !state->fp) return 1;

  fclose(state->fp);
  state->fp = NULL;
  return 0;
}

int FaceSetEyes(face_state *state, int val)
{
  if(!state || !state->fp) return 1;
  if(val > MAX_EYE_POS) val = MAX_EYE_POS;
  if(val < MIN_EYE_POS) val = MIN_EYE_POS;

  fprintf(stderr, "Setting eyes to %d\n", val);

  FaceWrite(state, EYE0_ADDR, EYE0_CENTER + val);
  FaceWrite(state, EYE1_ADDR, EYE1_CENTER + val);

  state->eye0_pos = state->eye1_pos = val;
  return 0;
}

int FaceSetBrows(face_state *state, int val)
{
  if(!state || !state->fp) return 1;
  if(val > MAX_BROW_POS) val = MAX_BROW_POS;
  if(val < MIN_BROW_POS) val = MIN_BROW_POS;

  fprintf(stderr, "Setting brows to %d\n", val);

  FaceWrite(state, BROW0_ADDR, BROW0_CENTER - val);
  FaceWrite(state, BROW1_ADDR, BROW1_CENTER + val);

  state->brow0_pos = state->brow1_pos = val;
  return 0;
}

int FaceSetMouth(face_state *state, int val)
{
  if(!state || !state->fp) return 1;
  if(val > MAX_MOUTH_POS) val = MAX_MOUTH_POS;
  if(val < MIN_MOUTH_POS) val = MIN_MOUTH_POS;

  fprintf(stderr, "Setting mouth to %d\n", val);

  FaceWrite(state, MOUTH0_ADDR, MOUTH0_CENTER - val);
  FaceWrite(state, MOUTH1_ADDR, MOUTH1_CENTER + val);

  state->mouth0_pos = state->mouth1_pos = val;
  return 0;
}

int FaceSetState(face_state *state)
{
  if(!state || !state->fp) return 1;  
  if(state->eye0_pos > MAX_EYE_POS) state->eye0_pos = MAX_EYE_POS;
  if(state->eye0_pos < MIN_EYE_POS) state->eye0_pos = MIN_EYE_POS;
  if(state->eye1_pos > MAX_EYE_POS) state->eye1_pos = MAX_EYE_POS;
  if(state->eye1_pos < MIN_EYE_POS) state->eye1_pos = MIN_EYE_POS;

  FaceWrite(state, EYE0_ADDR, EYE0_CENTER + state->eye0_pos);
  FaceWrite(state, EYE1_ADDR, EYE1_CENTER + state->eye1_pos);

  if(state->brow0_pos > MAX_BROW_POS) state->brow0_pos = MAX_BROW_POS;
  if(state->brow0_pos < MIN_BROW_POS) state->brow0_pos = MIN_BROW_POS;
  if(state->brow1_pos > MAX_BROW_POS) state->brow1_pos = MAX_BROW_POS;
  if(state->brow1_pos < MIN_BROW_POS) state->brow1_pos = MIN_BROW_POS;

  FaceWrite(state, BROW0_ADDR, BROW0_CENTER - state->brow0_pos);
  FaceWrite(state, BROW1_ADDR, BROW1_CENTER + state->brow1_pos);

  if(state->mouth0_pos > MAX_MOUTH_POS) state->mouth0_pos = MAX_MOUTH_POS;
  if(state->mouth0_pos < MIN_MOUTH_POS) state->mouth0_pos = MIN_MOUTH_POS;
  if(state->mouth1_pos > MAX_MOUTH_POS) state->mouth1_pos = MAX_MOUTH_POS;
  if(state->mouth1_pos < MIN_MOUTH_POS) state->mouth1_pos = MIN_MOUTH_POS;

  FaceWrite(state, MOUTH0_ADDR, MOUTH0_CENTER - state->mouth0_pos);
  FaceWrite(state, MOUTH1_ADDR, MOUTH1_CENTER + state->mouth1_pos);

  return 0;
}

int FaceAdjustMood(face_state *state, int val)
{
  if(!state) return 1;
  FaceSetBrows(state, state->brow0_pos + val);
  FaceSetMouth(state, state->mouth0_pos + (val * 2));
  return 0;
}

int FaceAdjustEyes(face_state *state, int val)
{
  if(!state) return 1;
  FaceSetEyes(state, state->eye0_pos + val);
  return 0;
}

int FaceReset(face_state *state)
{
  if(!state) return 1;
  FaceInit(state);
  FaceSetState(state);
  return 0;
}

