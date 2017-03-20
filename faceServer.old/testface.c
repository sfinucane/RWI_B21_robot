#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "FACE-messages.h"
#include "faceClient.h"

void CLIENT_close_handler(char *name, TCX_MODULE_PTR module) {
  fprintf(stderr, "CLIENT: closed connection detected: %s\n", name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else if (!strcmp(name, TCX_FACE_MODULE_NAME)){ /* server shut down */
    FACE = NULL;		/* prevents us from sending messages */
  }
}

main(int argc, char **argv) {
  float eyes, mouth;
  struct timeval tcx_waiting_time;

  /*
   * initialize tcx
   */

  faceRegister();
  initClient("client",CLIENT_close_handler);
  faceConnect(1);

  do{
    scanf("%f %f", &eyes, &mouth);
    tcx_waiting_time.tv_sec = 0;
    tcx_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &tcx_waiting_time);

    /*
     * do stuff
     */
    
    faceSetEyes(eyes);
    faceSetMouth(mouth);
    sleep(1);
  } while (1);
}


