#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "Nclient.h"

#include "robot_arg.h"

#define XY_DIST  1000
#define XY_SPEED 100
#define XY_ACCEL 300

#define PSI_DIST  (8*M_PI*1000)
#define PSI_SPEED 1100
#define PSI_ACCEL 1100

BOOL done;

static void SignalTrap(int signal_num)
{
  done = TRUE;
  return;
}

static void PickXYGoal(struct N_RobotState *rs)
{
  int result;
  long x, y, new_x, new_y;
  double time;
  struct N_AxisSet *as;

  as = &rs->AxisSet;

  if ((result = N_GetIntegratedConfiguration(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "get_integrated_conf error: %d\n", result);
    exit(-1);
  }

  x = rs->Integrator.x;
  y = rs->Integrator.y;
  new_x = ((double)XY_DIST*rand())/(RAND_MAX + 1.0);
  new_y = ((double)XY_DIST*rand())/(RAND_MAX + 1.0);

  printf("Translation: (%ld, %ld)->(%ld, %ld)\n", x, y, new_x, new_y);

  time = hypot(new_x - x, new_y - y)/XY_SPEED;

  as->Axis[N_XTRANSLATION].Update = TRUE;
  as->Axis[N_YTRANSLATION].Update = TRUE;
  as->Axis[N_XTRANSLATION].DesiredPosition = new_x;
  as->Axis[N_XTRANSLATION].DesiredSpeed = fabs((new_x - x)/time);
  as->Axis[N_XTRANSLATION].Acceleration = XY_ACCEL;
  as->Axis[N_YTRANSLATION].DesiredPosition = new_y;
  as->Axis[N_YTRANSLATION].DesiredSpeed = fabs((new_y - y)/time);
  as->Axis[N_YTRANSLATION].Acceleration = XY_ACCEL;

  if ((result = N_SetAxes(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "set_axis error: %d\n", result);
    exit(-1);
  }

  return;
}

static void Rotate(struct N_RobotState *rs)
{
  int result;
  long psi, new_psi;
  struct N_AxisSet *as;

  as = &rs->AxisSet;

  if ((result = N_GetIntegratedConfiguration(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "get_integrated_conf error: %d\n", result);
    exit(-1);
  }

  psi = rs->Integrator.Rotation;

  printf("Rotation:    %ld->%ld\n", psi, new_psi);

  as->Axis[N_ROTATION].Update = TRUE;
  as->Axis[N_ROTATION].DesiredPosition = psi + 90;
  as->Axis[N_ROTATION].DesiredSpeed = PSI_SPEED;
  as->Axis[N_ROTATION].Acceleration = PSI_ACCEL;

  if ((result = N_SetAxes(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "set_axis error: %d\n", result);
    exit(-1);
  }

  return;
}



int main(int argc, char *argv[])
{
  char *sched_host;
  const char *argv0;
  unsigned short sched_port;
  int error;
  long robot_id;
  struct N_RobotState *rs;
  struct N_AxisSet *as;

  argv0 = ARG_GetProgramName(argv[0]);

  /* get the robot ID and scheduler from the command line */
  sched_host = "localhost";
  sched_port = 7073;
  robot_id = 1;
  ARG_GetStandardArgs(&argc, &argv, &sched_host, &sched_port, &robot_id);

  /* initialize the client library and connect to our robot */
  if ((error = N_InitializeClient(sched_host, sched_port)) != N_NO_ERROR ||
      (error = N_ConnectRobot(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error connecting (%d).\n", argv0, error);
    exit(-1);
  }

  /* get a pointer to the state structure and the AxisSet substructure */
  rs = N_GetRobotState(robot_id);
  as = &rs->AxisSet;
  
  /* here, we zero the robot.  before setting the integrated configuration
   * to zero, we must make sure that none of the axes have a non-zero
   * goal position so that the zeroing does not induce a move */
  as->Global = TRUE;
  as->Axis[N_XTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_XTRANSLATION].DesiredPosition = 0;
  as->Axis[N_XTRANSLATION].DesiredSpeed = 0;
  as->Axis[N_XTRANSLATION].Acceleration = 0;
  as->Axis[N_XTRANSLATION].Update = TRUE;
  as->Axis[N_YTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_YTRANSLATION].DesiredPosition = 0;
  as->Axis[N_YTRANSLATION].DesiredSpeed = 0;
  as->Axis[N_YTRANSLATION].Acceleration = 0;
  as->Axis[N_YTRANSLATION].Update = TRUE;
  as->Axis[N_ROTATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_ROTATION].DesiredPosition = 0;
  as->Axis[N_ROTATION].DesiredSpeed = 0;
  as->Axis[N_ROTATION].Acceleration = 0;
  as->Axis[N_ROTATION].Update = TRUE;
  if ((error = N_SetAxes(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error setting axes to zero (%d).\n", argv0, error);
    exit(-1);
  }

  /* zero the integrated configuration */
  rs->Integrator.x = 0;
  rs->Integrator.y = 0;
  rs->Integrator.Rotation = 0;
  if ((error = N_SetIntegratedConfiguration(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error zeroing (%d).\n", argv0, error);
    exit(-1);
  }

  /* set up unchanging parameters */
  as->Global = TRUE;
  as->Axis[N_XTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_YTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_ROTATION].Mode = N_AXIS_POSITION_ABSOLUTE;

  done = FALSE;
  signal(SIGINT, SignalTrap);
  while (!done)
  {
    if ((error = N_GetAxes(robot_id)) != N_NO_ERROR)
    {
      fprintf(stderr, "%s: error retrieving current axis values (%d)\n", argv0,
	      error);
      exit(-1);
    }

    /* if x and y are both stopped, set a new xy goal */
    if (!as->Axis[N_XTRANSLATION].InProgress &&
	!as->Axis[N_YTRANSLATION].InProgress)
    {
      PickXYGoal(rs);
    }

    /* if rotation has stopped, set a new rotation goal */
    if (!as->Axis[N_ROTATION].InProgress)
    {
      PickPsiGoal(rs);
    }
  }

  /* stop the robot */
  rs->AxisSet.Global = TRUE;
  rs->AxisSet.Axis[N_XTRANSLATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_YTRANSLATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_ROTATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_XTRANSLATION].Update = TRUE;
  rs->AxisSet.Axis[N_YTRANSLATION].Update = TRUE;
  rs->AxisSet.Axis[N_ROTATION].Update = TRUE;
  if ((error = N_SetAxes(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error stopping the robot (%d).\n", argv0, error);
    exit(-1);
  }

  N_DisconnectRobot(robot_id);
  return 0;
}
