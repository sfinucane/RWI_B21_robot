// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/options.cc,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project.
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: options.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1999/06/29 08:38:38  schulz
//  Moved handling of command line options into these extra files.
//
//  Revision 1.2  1998/10/05 09:51:43  schulz
//  Added option to include the simulator into the RTLClient
//
//  Revision 1.1  1998/06/19 13:57:53  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.7  1997/10/25 08:30:05  schulz
//  added initialize method to some classes
//  modified matchTest to estimate position and door state seperately
//
//  Revision 1.6  1997/09/29 16:02:58  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.5  1997/09/18 08:16:20  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.4  1997/07/20 10:11:56  schulz
//  Many changes sice last update:
//  -- RTLServer/-Client and -Simulator databases are synchronized
//     over IP/Multicasting now.
//  -- multiple robots are allowed in one scene
//  -- robots are treated like ordinary active obstacles
//  -- added simple 3D visualization for robots
//  -- added a classes netlink and typeInfo, which implement
//  -- a fairly convenient way of transfering object states across the network
//
//  Revision 1.3  1997/03/05 16:29:43  schulz
//  removed obsolete files robot.cc robot.hh
//  added experimental support for MRTVR
//
//  Revision 1.2  1997/03/04 17:15:14  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <strings.h>
#include <stdlib.h>

#include "options.hh"
#ifdef __GNUG__
#include <typeinfo>
#endif
#include <vector>

#define TRUE 1
#define FALSE 0

char mapfile[80];
char inifile[80];
Boolean want_help = FALSE;
char server_mgroup[80];

static char *message_help1 = "This message";
static char *message_help2 = "dito";
static char *message_help3 = "dito";
static char *message_map   = "Load the mapfile <value>";
static char *message_ini   = "Load the inifile <value>";

static char *message_baseServer   = "Use baseServer protocol instead "
                             "of BASE protocol\n";
static char *message_mgroup = "Give a maultcast group i.e. 224.220.10.54/1998";
static char *message_sname = "TCX module name of this simulator";
static char *message_bname = "TCX module name of the colliServer to connect";

vector<option*> options;

static void
usage(char *exec)
{
    int i;
    printf("usage: %s [options]\n\nOptions:\n\n", exec);
    for( i = 0; i < options.size(); i++) {
	switch(options[i]->type) {
	case TOGGLE:
	    printf("%s\t\t%s\n",options[i]->token,options[i]->descr);
	    break;
	case VALUE:
	    printf("%s\t<value>\t%s\n",options[i]->token,options[i]->descr);
	}
    }
    exit(0);
}

void
read_options(int argc, char **argv)
{
  int i,j, found;

  add_option("-h", TOGGLE, &want_help, message_help1);
  add_option("-help", TOGGLE, &want_help, message_help2);
  add_option("--help", TOGGLE, &want_help, message_help3);
  add_option("-map", VALUE, mapfile, message_map);
  add_option("-ini", VALUE, inifile, message_ini);    
  add_option("-mgroup", VALUE, server_mgroup, message_mgroup);
  
  for(j = 0; j < options.size(); j++) 
    switch(options[j]->type) {
    case VALUE:
      options[j]->value.vstring[0] = 0; 
      break;
    case TOGGLE:
      *options[j]->value.vbool = FALSE;
      break;
    }
  for (i = 1; i < argc; i++) {
    found = 0;
    for(j = 0; j < options.size(); j++) {
      if(strcmp(options[j]->token, argv[i]) == 0) {
	found = 1;
	break;
      }
    }
    if(found) {
      switch(options[j]->type) {
      case TOGGLE:
	*options[j]->value.vbool = TRUE;
	break;
      case VALUE:
	if(i < argc-1) {
	  strcpy(options[j]->value.vstring,argv[i+1]);
	  i++;
	}
	else {
	  fprintf(stderr, "Missing value for option %s\n",argv[i]);
	  usage(argv[0]);
	}
      }
    }
    else {
      fprintf(stderr, "Unknown option %s\n", argv[i]);
      usage(argv[0]);
    }
  }
}

option::option(char *tkn, int tp, Boolean *val, char *dsc)
{
  token = tkn;
  type = tp;
  if(tp == TOGGLE)
    value.vbool = val;
  else
    value.vstring = val;
  descr = dsc;    
}

void
add_option(char *tkn, int tp, char *val, char *dsc)
{
  options.push_back( new option(tkn, tp, val, dsc));
}

