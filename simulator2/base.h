

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File name:                   base.h
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Peter Wallossek, University of Bonn
 *****
 ***** Date of creation:            Jan 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/base.h,v $
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
 * $Log: base.h,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1997/03/27 11:10:05  schulz
 * - Disabled tactile support for UNIBONN at the moment.
 * The robot does not seem to recover after a hit
 * - one more change in base_coord_{x,y}
 *
 * Revision 1.6  1997/03/25 21:44:46  tyson
 * Many bug fixes.
 *
 * Revision 1.5  1997/01/06 10:51:50  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.4  1996/11/22 20:50:56  tyson
 * minor clean-ups
 *
 * Revision 1.3  1996/11/18 14:43:11  ws
 * default ROBOT_RADIUS is taken from rai/B21/Base.h now.
 * Played around with job priorities a little bit.          DS
 *
 * Revision 1.2  1996/11/08 09:23:24  tyson
 * Added base support for baseSever<->Simulator2 to Simulator2.  Sonar will be tougher =:-O
 *
 * Revision 1.1.1.1  1996/09/30 16:17:43  schulz
 * reimport on new source tree
 *
 * Revision 1.3  1996/08/26 14:38:06  schulz
 * *** empty log message ***
 *
 * Revision 1.2  1996/08/26 10:05:29  schulz
 * added laser interface (without visualization, yet) D.S.
 *
 * Revision 1.9  1994/12/16 13:13:58  wallosse
 * No sin() and cos() changed in mySIN(), myCOS(), taking values out of table
 * from trigfkt.c
 * Added Commands IX, WD to base.c.
 *
 * Revision 1.8  1994/09/30  14:22:53  wallosse
 * Now there is a minimal distance for sonar !
 *
 * Revision 1.6  1994/09/06  11:52:16  fox
 * Now zooming is possible and scheduling works with the routines from block3.c
 *
 * Revision 1.5  1994/07/22  11:57:04  wallosse
 * Fixed a lot of bugs !
 * Vewrsion now runs again under LINUX when the optimize flag is not used when compiling.
 *
 * Revision 1.4  1994/06/22  13:45:25  wallosse
 * Changed a lot ! :	now events are scheduled with routine schedule().
 * 			obstacles can be added or chenged while robot
 * 			is runnung.
 * 			TARGET is not ok !!!
 *
 * Revision 1.2  1994/06/07  15:05:08  wallosse
 * New header files. Don't they look great?
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "tcx.h"
#include "rwibase_interface.h"
    
#define BATTERY_VOLTAGE		16

/* #define COUNTS_IN_360_DEGREES  		1024 */
#define COUNTS_PER_DEG      		(float) COUNTS_PER_DEGREE

/* internal BASE variables */

struct BaseVar {

  float         time;                

  float   	RotateAcceleration;
  float   	RotateWhere;
  float    	RotateWantedVelocity;
  float    	RotateCurrentVelocity;

  int		RotateDirection;
  float   	RotateRelativePos;
  char		RotateRelativeFlag;
  char		RotateHaltFlag;

  float   	TranslateAcceleration;
  float   	TranslateWhere;			
  float    	TranslateWantedVelocity;
  float    	TranslateCurrentVelocity;

  int		TranslateDirection;
  float   	TranslateRelativePos;
  char		TranslateRelativeFlag;
  char		TranslateHaltFlag;

/*
  int           pos_x;               
  int 		pos_y;               
*/

  char		bump;
  int		watch_dog;

  long  	StatusReportPeriod;

};

extern struct BaseVar BaseVariables;

extern int	retvalHandleMove;



extern struct 	timeval		tv_start;		/* tv_base - tv_start  =  running time */
extern struct 	timeval		tv_base;


/**********************************************************************************************************/
/* PROCEDURE :                  base()                                  ***********************************/
/* Parameter :                  command                                 ***********************************/
/*                                                                      ***********************************/
/* Change internal base variables in dependencie if Parameter command.  ***********************************/
/* This parameters are used by routine HandleMove().                    ***********************************/
/* Sets timer for status report update.                                 ***********************************/
/**********************************************************************************************************/

void	base(unsigned char command[]);


/**********************************************************************************************************/
/* PROCEDURE :                  HandleMove()                            ***********************************/
/* Parameter :                  none					***********************************/
/*                                                                      ***********************************/
/* Using internal base variables to change robtot parameters like       ***********************************/
/* acceleration, velocity, position ...                                 ***********************************/
/* Returns 1, when move can be performed, 0 otherwise                   ***********************************/
/**********************************************************************************************************/

void	HandleMove();


/**********************************************************************************************************/
/* PROCEDURE :                  InitBaseVar()                           ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* Initialization of global Base Variables                              ***********************************/
/**********************************************************************************************************/

void    InitBaseVar();


/**********************************************************************************************************/
/* PROCEDURE :                  statusReport()                          ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* sends status report via TCX to MODULE_BASE                           ***********************************/
/**********************************************************************************************************/


void statusReport();

/*** only used for status report ***/

/*** some help routines ***/
#define mySGN(x)  ((x) >= 0 ? (1) : (-1))
#define myABS(x)  ((x) < 0 ? -(x) : (x))
#define mySQR(x)  ((x) * (x))
#define myMAX(x, y)     ((x) > (y) ? (x) : (y))
#define myMIN(x, y)     ((x) < (y) ? (x) : (y))
#define ctoi(x)         ((x) - '0')

/*** tables with trigonometric values -> used for speed up ***/
/* who needs speedup ? */

/* extern float SIN_TAB[]; */
/* #define mySIN(x) sin(x * M_PI / 180) */
/* #define myCOS(x) cos(x * M_PI / 180) */

/* #define mySIN(x) (mySGN((x))*SIN_TAB[(int)(0.5 + myABS((x))) % 360]) */
/* #define myCOS(x) (mySIN((x)+90)) */
/*** atan from table doesn't speed up !?! ***/

void base_sendTarget(float,float);
void InitBaseVar();

float base_coord_x();
float base_coord_y();
float base_coord_rot();

struct mcpPacket {
  unsigned char size;
  unsigned char chksum;
  unsigned char data1[255];
  unsigned char data[255];
};

typedef struct mcpPacket mcpPacketType;

#ifdef __cplusplus
}
#endif
