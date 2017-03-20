
#ifndef AUTOUPDES_H
#define AUTOUPDES_H

#include "SIMULATOR-messages.h"

#define MAX_N_AUTO_UPDATE_MODULES  100

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int skip;
} auto_update_type;

int add_auto_update_module(TCX_MODULE_PTR module, int skip);
int remove_auto_update_module(TCX_MODULE_PTR module);
void send_automatic_status_update(SIMULATOR_status_update_type *status);

#endif
