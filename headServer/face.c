#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include "serial.h"
#include "sock.h"
#include "face.h"

int FaceWrite(face_state *state, int addr, int val)
{
  char cmd[10];

  if(!state) return 1;
  sprintf(cmd, "%c%c%c", SSC_SYNC, addr, val);
  writen(state->fd, cmd, 3);
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

  printf("Opening face.\n");
  state->fd = open_serialport(FACE_PORT);
  if(state->fd != -1)
    perror("");
  FaceInit(state);
  return (state->fd < 0);
}

int FaceClose(face_state *state)
{
  if(!state || state->fd < 0) return 1;

  close_serialport(state->fd);
  state->fd = -1;
  return 0;
}

int FaceSetEyes(face_state *state, int val)
{
  if(!state || !state->fd < 0) return 1;
  if(val > MAX_EYE_POS) val = MAX_EYE_POS;
  if(val < MIN_EYE_POS) val = MIN_EYE_POS;

  FaceWrite(state, EYE0_ADDR, EYE0_CENTER + val);
  FaceWrite(state, EYE1_ADDR, EYE1_CENTER + val);

  state->eye0_pos = state->eye1_pos = val;
  return 0;
}

int FaceSetBrows(face_state *state, int val)
{
  if(!state || !state->fd < 0) return 1;
  if(val > MAX_BROW_POS) val = MAX_BROW_POS;
  if(val < MIN_BROW_POS) val = MIN_BROW_POS;

  FaceWrite(state, BROW0_ADDR, BROW0_CENTER - val);
  FaceWrite(state, BROW1_ADDR, BROW1_CENTER + val);

  state->brow0_pos = state->brow1_pos = val;
  return 0;
}

int FaceSetMouth(face_state *state, int val)
{
  if(!state || !state->fd < 0) return 1;
  if(val > MAX_MOUTH_POS) val = MAX_MOUTH_POS;
  if(val < MIN_MOUTH_POS) val = MIN_MOUTH_POS;

  FaceWrite(state, MOUTH0_ADDR, MOUTH0_CENTER - val);
  FaceWrite(state, MOUTH1_ADDR, MOUTH1_CENTER + val);

  state->mouth0_pos = state->mouth1_pos = val;
  return 0;
}

int FaceSetState(face_state *state)
{
  if(!state || !state->fd < 0) return 1;  
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

