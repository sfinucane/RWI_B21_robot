#include <stdio.h>
#include "autoupdates.h"

static int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];

int add_auto_update_module(TCX_MODULE_PTR module, int skip)
{
  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  fprintf(stderr, "Add %s to auto-reply list.\n",
	  tcxModuleName(module));
  auto_update_modules[n_auto_update_modules].module = module; /*new pointer*/
  auto_update_modules[n_auto_update_modules].skip = skip; /*new pointer*/
  n_auto_update_modules++;
  return 1;
}

int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].skip = 
	  auto_update_modules[j+1].skip; /* shift back */
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  return found;
}

static int n_send_automatic_status_update = 0;

void send_automatic_status_update(SIMULATOR_status_update_type *status)
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++){
    if( n_send_automatic_status_update % auto_update_modules[i].skip == 0) {
      tcxSendMsg(auto_update_modules[i].module, 
		 "SIMULATOR_status_update",
		 status);
    }
  }
  n_send_automatic_status_update++;
}
