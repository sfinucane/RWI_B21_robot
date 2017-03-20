/* Time-stamp: <98/02/12 16:04:58 derr>
***********************************************************************
***********************************************************************
***********************************************************************
*****
***** File name:                   constDef.h
*****
***** Part of:                     RHINO SOFTWARE
*****
***** Creator:                     Andreas Derr, University of Bonn
*****
***** Date of creation:            Oct 1997
*****
***** Purpose:                     const definitions (global variables)
*****                              of class rawData
*****
*****
***** $Source: /usr/local/cvs/bee/src/localize/rawData/constDef.h,v $
*****
***** $Revision: 1.1 $
*****
***** $Date: 2002/09/14 17:06:31 $
*****
***** $Author: rstone $
*****
*****
*****
***** Contact derr@informatik.uni-bonn.de
*****
***********************************************************************
***********************************************************************
***********************************************************************
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
***********************************************************************
***********************************************************************
***********************************************************************
**
**
**                  ----- REVISION HISTORY -----
**
** $Log: constDef.h,v $
** Revision 1.1  2002/09/14 17:06:31  rstone
** *** empty log message ***
**
** Revision 1.1  1998/02/12 18:13:59  derr
** Added library for reading laserint- sonarint- and 'new'-scripts.
**
** Revision 1.1  1998/02/12 15:59:48  derr
** New library librawData for reading laserint-, sonarint and 'new'-script.
** Will replace bee/src/localize/script.c.
**
**
**
***********************************************************************
***********************************************************************
**********************************************************************/
#ifndef CONST_DEF_OF_RAW_EXTERN_DATA_INCLUDE
#define CONST_DEF_OF_RAW_EXTERN_DATA_INCLUDE

extern const char *ORIG_MARKERSCRIPTMARK;
extern const char *ORIG_SCRIPTLASERMARK;
extern const char *ORIG_SCRIPTPOSMARK;
extern const char *ORIG_SCRIPTSONARMARK;
extern const char *ORIG_SCRIPTNAMEMARK;
extern const char *ORIG_SCRIPTDESCRIPTIONMARK;
extern const char *ORIG_TIMEMARK;

extern const char *SECOND_SCRIPT_START;
extern const char *SECOND_SCRIPT_NAME_STR;
extern const char *SECOND_SCRIPT_TYPE_STR;
extern const char *SECOND_SCRIPT_END;

extern const char *SECOND_SCRIPT_PAT_START;
extern const char *SECOND_SCRIPT_PAT_NAME;
extern const char *SECOND_SCRIPT_PAT_POSITION;
extern const char *SECOND_SCRIPT_PAT_TIME;
extern const char *SECOND_SCRIPT_PAT_BUTTON;
extern const char *SECOND_SCRIPT_PAT_INFRARED;
extern const char *SECOND_SCRIPT_PAT_LASER;
extern const char *SECOND_SCRIPT_PAT_SONAR;
extern const char *SECOND_SCRIPT_PAT_TACTILE;
extern const char *SECOND_SCRIPT_PAT_MARKER;
extern const char *SECOND_SCRIPT_PAT_END;

extern const char *SECOND_SCRIPT_PAT_TYPE_LASER;
extern const char *SECOND_SCRIPT_PAT_TYPE_POSITION;
extern const char *SECOND_SCRIPT_PAT_TYPE_SONAR;
extern const char *SECOND_SCRIPT_PAT_TYPE_TIME;
extern const char *SECOND_SCRIPT_PAT_TYPE_MARKER;
extern const char *SECOND_SCRIPT_PAT_TYPE_INFRARED;
extern const char *SECOND_SCRIPT_PAT_TYPE_TACTILE;
extern const char *SECOND_SCRIPT_PAT_TYPE_BUTTON;

extern const char *SCRIPTBUMPERMARK;

extern const char  END_OF_LINE;

extern const double DEG_360;

extern const char *STDIN_SCRIPT_NAME;
/* extern const float ROB_RADIUS; */
#endif
/* End of CONST_DEF_OF_RAW_DATA_HEADER */

