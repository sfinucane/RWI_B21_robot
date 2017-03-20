#define FACE_PORT "/dev/ttyS3"

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

  int fd;
} face_state;

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
