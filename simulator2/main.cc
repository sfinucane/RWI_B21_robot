
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        main.cc
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Dirk Schulz, University of Bonn
 *****
 ***** Date of creation:            July 1996
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/main.cc,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: main.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.43  2000/08/31 09:57:19  haehnel
 * insert two "void" declaration
 *
 * Revision 1.42  2000/03/09 09:30:07  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.41  1999/08/26 16:53:53  schulz
 * -- The robot position can now be specified on the command line.
 * -- SIMULATOR send the robot's global position to modules, which
 *    have registered for it (Not tested yet!).
 *
 * Revision 1.40  1999/06/10 15:49:42  rhino
 * *** empty log message ***
 *
 * Revision 1.39  1999/02/19 14:30:35  schulz
 * replaced Tysons block_wait by a private copy without the XSetFocus stuff!!!
 * Removed any dependency on librobot and libezx.
 *
 * Revision 1.38  1998/04/12 15:55:08  wolfram
 * Added option -robot for multi-robot support
 *
 * Revision 1.37  1997/12/11 14:51:36  schulz
 * added missing case in inhalfplane()
 *
 * Revision 1.36  1997/10/24 20:58:46  tyson
 * The previous X changes where munged.
 *
 * Revision 1.35  1997/10/24 19:06:35  tyson
 * fixed problems with X events and abus
 *
 * Revision 1.34  1997/03/14 17:21:47  tyson
 * Added tactile support to simulator
 *
 * Revision 1.33  1997/03/11 17:16:43  tyson
 * added IR simulation and other work
 *
 * Revision 1.32  1997/02/13 12:50:47  tyson
 * minor fixes.  Fixed stdin on COLLI.
 *
 * Revision 1.31  1997/02/12 13:11:22  tyson
 * more parameter utils.  Updated wander.c. Some processes now support -fork=[y|n] instead of [+|-]stdin
 *
 * Revision 1.30  1997/02/02 22:32:43  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.29  1997/01/30 18:05:13  tyson
 * minor cleanups of abus.  PLAN now gets TCXHOST from beeSoft.ini if not set in envronment
 *
 * Revision 1.28  1997/01/30 13:33:06  schulz
 * minor fixes
 *
 * Revision 1.27  1997/01/29 17:48:28  schulz
 * removed comment from bDeamonize
 *
 * Revision 1.26  1997/01/29 17:47:17  schulz
 * fixed a bug fill_ws_ray()
 *
 * Revision 1.25  1997/01/28 20:39:37  tyson
 * daemonize COLLI, other tweaks
 *
 * Revision 1.24  1997/01/27 15:13:29  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.23  1997/01/26 22:26:30  tyson
 * bUtils parameter file improvements
 *
 * Revision 1.22  1997/01/24 21:42:28  tyson
 * fixed serious libmcp bug, added bUtils for parameter files, logging and daemonizing plus misc.
 *
 * Revision 1.21  1996/12/10 13:43:58  schulz
 * -- added -tcx option
 * -- loading of maps now possible
 * -- file requester has bigger width now.
 *    The translation for the return key has still to be changed!
 *
 * Revision 1.20  1996/12/04 15:05:30  schulz
 * Fix NO_OPTIONS to 7 anyone played around with that ?
 *
 * Revision 1.19  1996/12/04 14:25:28  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.18  1996/12/03 15:18:30  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.17  1996/11/28 13:16:40  schulz
 * minor bugfix
 *
 * Revision 1.16  1996/11/25 21:06:32  tyson
 * Simulator ready for baseServer?  ...time to update baseServer.
 *
 * Revision 1.15  1996/11/21 15:33:24  schulz
 * Some enhancements on the scheduler, we use block_wait instead of
 * XtAppTimeOut now -> better TCX update performance
 *
 * Revision 1.14  1996/11/18 14:43:12  ws
 * default ROBOT_RADIUS is taken from rai/B21/Base.h now.
 * Played around with job priorities a little bit.          DS
 *
 * Revision 1.13  1996/11/18 04:34:59  tyson
 * More baseServer<->simulator2 work. Still not done.
 *
 * Revision 1.12  1996/11/15 14:50:04  schulz
 * this version should be fairly stable.             DS
 *
 * Revision 1.11  1996/11/15 12:21:23  ws
 * Changed InsideObstacle to use the obstacle grid.          DS
 *
 * Revision 1.10  1996/11/14 16:26:32  schulz
 * Changed priorities and timings to allow laser only control   DS
 *
 * Revision 1.9  1996/11/14 12:58:03  ws
 * Introduced a scheduler with priorities and aging, to
 * get better performance on heavily loaded systems.    DS
 *
 * Revision 1.8  1996/11/08 09:23:24  tyson
 * Added base support for baseSever<->Simulator2 to Simulator2.  Sonar will be tougher =:-O
 *
 * Revision 1.7  1996/10/29 17:11:44  schulz
 * removed one more busy wait
 *
 * Revision 1.6  1996/10/25 14:27:12  ws
 * some improvements
 * - better options handling
 * - added an initialization file, many parameters are modifiable now
 * - Modified some timings to reduce CPU load
 *   (TCX is polled after every 50 ms now)
 *
 * Revision 1.5  1996/10/15 13:58:53  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.4  1996/10/14 12:19:09  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.3  1996/10/09 13:27:38  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.2  1996/10/04  11:35:23  ws
// some bug fixes in zooming and centering the robot
//
 * Revision 1.1.1.1  1996/09/30 16:17:45  schulz
 * reimport on new source tree
 *
 * Revision 1.1  1996/08/27 15:23:08  schulz
 * Some files forgotten last time
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <sys/time.h>
#include <bUtils.h>

#include "tcx.h"

#include "robot.h"
#include "playground.hh"
#include "simxw.h"
#include "sonar.h"
#include "ir.h"
#include "tactile.h"
#include "laser.h"
#include "surface.hh"
#include "schedule.hh"
#include "store.hh"

//#include "multirobot.h"

extern t_obstgrid *obstore;
extern char editing;

extern "C" {
  void laserReport();
  void robot_HandleMove();
  void multirobotReport(); // AF
}

#define DRAW_PRIO 5
#define LASER_PRIO 2
#define BASE_PRIO 0
#define STATUS_PRIO -1
#define SONAR_PRIO -1

#define MULTIROBOT_PRIO 0

#define SONAR_EVENT 1

#define MAX_ROBOTS 20 //AF 

/*************  timer intervals in ms for different tasks ****************/

int basevar_update_interval = 50;
int laser_update_interval = 200;
int robot_display_interval = 100;
int status_report_interval = 100;
int sonarServer_report_interval = 100;
int irServer_report_interval = 100;
int tactileServer_report_interval = 100;
int tcx_query_interval = 10;
int multirobot_update_interval=50;

int NumberOfRobots=0;


t_scheduler *sim_scheduler;
struct timeval robot_display_duration = {0,1000};
struct timeval laser_update_duration = {0,1000};
struct timeval basevar_update_duration = {0,1000};
struct timeval tcx_query_duration = {0,1000};
struct timeval status_report_duration = {0,1000};
struct timeval sonar_report_duration = {0,1000};
struct timeval sonarServer_report_duration = {0,1000};
struct timeval irServer_report_duration = {0,1000};
struct timeval tactileServer_report_duration = {0,1000};

struct timeval multirobot_update_duration = {0,1000};

struct timeval robot_display_delta = {0, 0};
struct timeval laser_update_delta = {0, 0};
struct timeval basevar_update_delta = {0, 0};
struct timeval tcx_query_delta = {0, 0};
struct timeval status_report_delta = {0, 0};
struct timeval sonar_report_delta = {0, 0};
struct timeval sonarServer_report_delta = {0, 0};
struct timeval irServer_report_delta = {0, 0};
struct timeval tactileServer_report_delta = {0, 0};

struct timeval multirobot_update_delta = {0,0};

char mapfile[80];
char inifile[80];
char robotNameString[80]="";
char *robotName = NULL;  /* name added to TCX module names */
char pos_string[40] ="";
char m_robotName[MAX_ROBOTS][80]; 
char m_robotNameString[80]="";

Boolean just_viewing = FALSE;
Boolean without_laser = FALSE;
Boolean want_help = FALSE;
Boolean no_tcx = FALSE;
Boolean mark_option = FALSE;
Boolean use_baseServer = FALSE;
Boolean use_sonarServer = FALSE;
Boolean multi_robots = FALSE; /*AF: option is set, when names are read out.*/
Boolean displaynames = FALSE;

typedef struct option {
    char *token;
    int type;
    union {
	Boolean *vbool;
	char *vstring;
    } value;
    char *descr;
};

#define TOGGLE 0
#define VALUE 1
#define VALUES 2 // AF

char *message_help1 = "This message";
char *message_help2 = "dito";
char *message_help3 = "dito";
char *message_map   = "Load the mapfile <value>";
char *message_ini   = "Load the inifile <value>";
char *message_view  = 
  "Do not simulate the robot,\n"
  "\tsimply ask BASE for the robots position.\n"
  "\tUsing this option, the simulator is internally renamed to\n"
  "\tVIEWER, so you can connect BASE to another simulator\n";
char *message_mark =
   "Mark to front side of obstacles with a dot.\n"
   "Only cubes are handled yet. The front side is defined to be at\n"
   "the center point minus width/2.\n";
char *message_laser = 
"Disable laser simulation, this is usful, if BASE runs without\n"
"\t\tlaser support. It is not strictly neccessary, but it saves\n"
"\t\tsome CPU time";
char *message_pos =
"Specify the robot's starting position x,y,rot . x,y in cm, rot in deg\n";
char *message_notcx = "Do not connect, use simulator for map editing only";
char *message_baseServer   = "Use baseServer protocol instead "
                             "of BASE protocol\n";
char *message_robot = "Specify the name of the robot\n";
char *message_robots = "Specify the name(s) of the other robot(s)\n"; //AF
char *message_display = "Displays the names of the robots "
"in simulator window\n"; //AF


#define NO_OPTIONS 13 // AF
option options[NO_OPTIONS] =
{
    {"-h", TOGGLE, &want_help, message_help1},
    {"-help", TOGGLE, &want_help, message_help2},
    {"--help", TOGGLE, &want_help, message_help3},
    {"-map", VALUE, mapfile, message_map},
    {"-ini", VALUE, inifile, message_ini},    
    {"-view", TOGGLE, &just_viewing, message_view},
    {"-laser", TOGGLE, &without_laser, message_laser},
    {"-tcx", TOGGLE, &no_tcx, message_notcx},
    {"+mark", TOGGLE, &mark_option, message_mark},
    {"-pos", VALUE, pos_string, message_pos},
    {"-robot", VALUE, robotNameString, message_robot},
    {"-robots", VALUES, m_robotNameString, message_robots}, // AF
    {"-nodisplaynames", TOGGLE, &displaynames, message_display} // AF
}; 

extern "C" {
void  InitBaseVar();  // from base.h which is incompatible with c++
void read_inifile(char *);
}

extern t_surflist *surfaces;

void
usage(char *exec)
{
    int i;
    printf("usage: %s [options]\n\nOptions:\n\n", exec);
    for( i = 0; i < NO_OPTIONS; i++) {
	switch(options[i].type) {
	case TOGGLE:
	    printf("%s\t\t%s\n",options[i].token,options[i].descr);
	    break;
	case VALUE:
	    printf("%s\t<value>\t%s\n",options[i].token,options[i].descr);
	    break;
	case VALUES: //
	    printf("%s\t<val1>,<...>,<valn>\t%s\n",options[i].token,options[i].descr); 
	    break;
	}
    }
    /*    exit(0); */
}

void
read_options(int argc, char **argv)
{
    int i, j, k, l, found;
    for (i = 1; i < argc; i++) {
	found = 0;
	for(j = 0; j < NO_OPTIONS; j++) {
	    if(strcmp(options[j].token, argv[i]) == 0) {
		found = 1;
		break;
	    }
	}
	if(found) {
	    switch(options[j].type) {
	    case TOGGLE:
		*options[j].value.vbool = TRUE;
		break;
	    case VALUE:
		if(i < argc-1) {
		    strcpy(options[j].value.vstring,argv[i+1]);
		    i++;
		}
		else {
		    fprintf(stderr, "Missing value for option %s\n",argv[i]);
		    usage(argv[0]);
		}
		break;
	    case VALUES: //AF
		 l=0;
		 NumberOfRobots++;
		 char *token_robotName;
		if(i < argc-1) {
		  strcpy(options[j].value.vstring,argv[i+1]);
		i++;
		token_robotName=options[j].value.vstring;
		while (*token_robotName !='\0') {
		  if (*token_robotName !=',') {  
		    m_robotName[NumberOfRobots][l++]=*token_robotName;
		    *token_robotName++;
		  }
		      else {
			*token_robotName++;
			m_robotName[NumberOfRobots][l]='\0';
			l=0;
			NumberOfRobots++;
		      }
		}
		m_robotName[NumberOfRobots][l]='\0';
		multi_robots= TRUE;
		  }
		else {
		  fprintf(stderr, "Missing value for option %s\n",argv[i]);
		  usage(argv[0]);
		}
		break;
	    }
	}
	else {
	    fprintf(stderr, "Unknown option %s\n", argv[i]);
	    usage(argv[0]);
	}
    }
    /*        printf("Other Robots:\n"); // AF
     for (i=0; i<=NumberOfRobots; i++)
       printf("%s\n", m_robotName[i]); 
       getchar(); */
}


void
MoveRobotJob()
{
    MoveRobot();
    if(!editing) obstore->Redraw();
    sim_scheduler->schedule(REGULAR, MoveRobotJob,
			    DRAW_PRIO, robot_display_interval*1000,
			    &robot_display_duration, &robot_display_delta);
}
void
LaserReportJob()
{
    laserReport();
    sim_scheduler->schedule(REGULAR, LaserReportJob,
			    LASER_PRIO, laser_update_interval*1000,
			    &laser_update_duration, &laser_update_delta);
}
void
BaseUpdateJob()
{
    robot_HandleMove();
    if(!editing) obstore->Update();    
    sim_scheduler->schedule(REGULAR, BaseUpdateJob,
			    BASE_PRIO, basevar_update_interval*1000,
			    &basevar_update_duration, &basevar_update_delta);    
}


void
MultiRobotJob() // AF
{
  printf("I'm Robot: %s\t\t(red)\n", robotNameString);
  /*  printf("Other Robots connected to system:\n");
  for (int var=1; var <= NumberOfRobots; var ++)
  printf("%i.\t%s\n", var, m_robotName[var]);*/

  multirobotReport();
}

extern "C" {
    void schedule_statusReport(int);
    void schedule_sonarReport(int);    
    void schedule_sonarServerReport(int);    
    void schedule_irServerReport(int);    
    void schedule_tactileServerReport(int);    
    void statusReport();
    void sonarReport();
    int sonarServerReport();
    int irServerReport();
    int tactileServerReport();
}

void the_statusReport()
{
    statusReport();
    sim_scheduler->schedule(REGULAR, the_statusReport,
			    STATUS_PRIO, status_report_interval*1000,
			    &status_report_duration, &status_report_delta);
}
void schedule_statusReport(int delay)
{
    status_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_statusReport,
			    STATUS_PRIO, delay*1000,
			    &status_report_duration,
			    &status_report_delta);
}
void schedule_sonarReport(int delay)
{
    sim_scheduler->schedule(SONAR_EVENT, (void (*)()) sonarReport,
			    SONAR_PRIO, delay*1000,
			    &sonar_report_duration, &sonar_report_delta);
}


void
the_irServerReport()
{
  if (irServerReport()) {
    sim_scheduler->schedule(REGULAR, the_irServerReport,
			    STATUS_PRIO, irServer_report_interval*1000,
			    &irServer_report_duration,
			    &irServer_report_delta);
  }
}

void
schedule_irServerReport(int delay)
{

  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);

    irServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_irServerReport,
			    STATUS_PRIO, delay*1000,
			    &irServer_report_duration,
			    &irServer_report_delta);
}


void
the_tactileServerReport()
{
  if (tactileServerReport()) {
    sim_scheduler->schedule(REGULAR, the_tactileServerReport,
			    STATUS_PRIO, tactileServer_report_interval*1000,
			    &tactileServer_report_duration,
			    &tactileServer_report_delta);
  }
}

void
schedule_tactileServerReport(int delay)
{
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);

    tactileServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_tactileServerReport,
			    STATUS_PRIO, delay*1000,
			    &tactileServer_report_duration,
			    &tactileServer_report_delta);
#if 1
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}


void
the_sonarServerReport()
{
  if (sonarServerReport()) {
    sim_scheduler->schedule(REGULAR, the_sonarServerReport,
			    STATUS_PRIO, sonarServer_report_interval*1000,
			    &sonarServer_report_duration,
			    &sonarServer_report_delta);
  }
}

void
schedule_sonarServerReport(int delay)
{
    sonarServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_sonarServerReport,
			    STATUS_PRIO, delay*1000,
			    &sonarServer_report_duration,
			    &sonarServer_report_delta);
#ifndef UNIBONN
    tactileInit();
#endif
}


main(int argc, char *argv[])
{
  struct bParamList * bParamList = NULL;
  int i;
  float robotX = 0, robotY = 0, robotOri = 0;
  int event_spec;
  struct timeval waiting_time;
  inifile[0] = 0;

  read_options(argc, argv);
  //  exit(0);
  if (strcmp(robotNameString, "") != 0)
    robotName=robotNameString;
  else
    robotName=NULL;

  if(strlen(pos_string) > 0) {
    int cnt = 0;

    fprintf(stderr, "pos_string: %s\n", pos_string);
    cnt = sscanf(pos_string, "%f,%f,%f",
		 &robotX, &robotY, &robotOri);
    if(cnt < 3) {
      fprintf(stderr, "The command line position specification has wrong format!\n");
  //    exit(-1);
    }
  }

  surfaces = new t_surflist();
  read_inifile(inifile);

  
  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList = bParametersAddEntry(bParamList, "", "fork", "yes");
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");

#ifdef UNIBONN
  bParamList = bParametersAddEntry(bParamList, "", "fork", "no");
#endif

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);
  
  *argv=robotNameString ;
  InitWidgets(&argc, argv);
  if(strlen(pos_string) > 0) {
    map = new t_playground(mapfile, robotX, robotY, robotOri);
  }
  else
    map = new t_playground(mapfile);

  InitRobot();
  InitBaseVar();
  InitSonar();
  if(!no_tcx) {

    if (bRobot.fork) {
      bDaemonize("simulator2");
    }
    
    fprintf(stderr, "TCXHOST=[%s]\n", bRobot.TCXHOST);
    
    sim_scheduler = new t_scheduler();
    
    InitTCX(bRobot.TCXHOST);
    

    //    MultiRobotJob();    
    sim_scheduler->schedule(REGULAR, MoveRobotJob,
			    DRAW_PRIO, robot_display_interval*1000,
			    &robot_display_duration, &robot_display_delta);
    
    //      printf("Move the Robot:%p\n", (void *) MoveRobotJob);
    //      printf("LaserReportJob:%p\n", (void *) LaserReportJob);
    if(!without_laser) {
      sim_scheduler->schedule(REGULAR, LaserReportJob,
			      LASER_PRIO, laser_update_interval*1000,
			      &laser_update_duration, &laser_update_delta);
    }
    if(!just_viewing) {
      sim_scheduler->schedule(REGULAR, BaseUpdateJob,
				  BASE_PRIO, basevar_update_interval*1000,
				  &basevar_update_duration, &basevar_update_delta);	    
      }

      /* Multirobot-Scheduling */
      //      printf("MultiRobotJob:%p\n", (void *) MultiRobotJob);
      //      printf("Multi is set: %i", multi_robots);
       
    if (multi_robots){
      sim_scheduler->schedule(REGULAR, MultiRobotJob,
			      MULTIROBOT_PRIO, multirobot_update_interval*1000,
			      &multirobot_update_duration, &multirobot_update_delta); 
    }
		
      while(1) {
	  event_spec = sim_scheduler->triggerNextEvent();
	  switch(event_spec) {
	  case 0:
	  case 1:
	      query_x();
	      break;
	  case 2:
	      waiting_time.tv_sec = 0;
	      waiting_time.tv_usec = 0;		
	      tcxRecvLoop(&waiting_time);
	      break;
	  default:
	      break;
	  }
      }
  }
  else {
      mainloop();
  }
}








