
#include<stdlib.h>
#include<stdio.h>
#include<sys/time.h>
#include"tcx.h"
#include"tcxP.h"
#define DEFINE_REPLY_HANDLERS
#define TCX_define_variables
#include "SIMULATOR-messages.h"

void update_test_close_handler(char *name, TCX_MODULE_PTR module)
{
  if(module == SIMULATOR) {
    fprintf(stderr, "SIMULATOR died, so do I\n");
    exit(0);
  }
}


void 
SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				  char **message)
{;}

void 
SIMULATOR_message_to_baseServer_handler(TCX_REF_PTR   ref,
					char *message)
{;}



void 
SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
					char **message)
{;}


void 
SIMULATOR_message_to_laser_handler(TCX_REF_PTR   ref,
				   SIMULATOR_message_to_laser_ptr	data)
{;}

void 
SIMULATOR_status_update_handler(TCX_REF_PTR ref,
				    SIMULATOR_status_update_type *status)
{
  static int cnt = 0;

  fprintf(stderr, "robot_pos: %f %f %f          \r",
	  status->posX,
	  status->posY,
	  status->posRot);
  tcxFree("SIMULATOR_status_update", status);
  cnt ++;
  if(cnt > 100) {
    tcxSendMsg(SIMULATOR, "SIMULATOR_cancel_auto_update", NULL);
  }
}


int
main(int argc, char **argv)
{
  int skip = 1;
  char *tcxServerHost;
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    SIMULATOR_messages
  };

  tcxServerHost = getenv("TCXHOST");
  if(tcxServerHost == NULL) {
    tcxServerHost = "localhost";
  }
  tcxInitialize(argv[1], tcxServerHost);
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			  / sizeof(TCX_REG_MSG_TYPE));  
  tcxRegisterHandlers(SIMULATOR_reply_handler_array,
			  sizeof(SIMULATOR_reply_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterCloseHnd(update_test_close_handler);

  SIMULATOR = tcxConnectModule("SIMULATOR");
  tcxSendMsg(SIMULATOR, "SIMULATOR_register_auto_update", &skip);
  while(1) {
    struct timeval waiting_time;
    waiting_time.tv_sec = 0;
    waiting_time.tv_usec = 100000;
    tcxRecvLoop(&waiting_time);
  }
}

