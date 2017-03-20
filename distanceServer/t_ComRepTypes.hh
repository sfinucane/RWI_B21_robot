
#ifndef _COMREPTYPES_HH
#define _COMREPTYPES_HH

#define COM_OBJ_TYPE_BOX 1 
#define RTLdefaultShader 0

typedef struct {
  struct {
    double x,y,z;
  } center;
  struct {
    double x,y,z;
  } dim;
  double mat[4][3];
  unsigned short shaderId;  
  char nameLen;
} t_BoxComRep;

typedef struct {
  char cmd;
  char opcode;
  unsigned short len;
} t_ComPacketHdr;

#endif
