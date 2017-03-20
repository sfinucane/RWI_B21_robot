
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        trigofkt.c
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/trigofkt.c,v $
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
 * $Log: trigofkt.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/30 16:17:46  schulz
 * reimport on new source tree
 *
 * Revision 1.2  1996/08/27 08:12:04  schulz
 * Changed Filename suffix to .cc for c++ files
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <math.h>

float sin_tab[720];
float cos_tab[720];
float tan_tab[720];

float _mySIN(float r)
{
  float d = r * 180.0 / M_PI;
  int d2;
  d2 = (int)(2*d);
  return sin_tab[d2];
}

float _myCOS(float r)
{
  float d = r * 180 /M_PI;
  int d2 = (int)(2*d);
  return cos_tab[d2];
}

void initTrigFkt()
{
  float f;
  for(f = 0; f< 360; f += 0.5 ) {
    sin_tab[(int)(f*2)] = sin(f*M_PI/180);
    cos_tab[(int)(f*2)] = cos(f*M_PI/180);
  }
  for(f = -90; f <90; f+=0.5) {
    tan_tab[(int)(2*(270+f))] =
      tan_tab[(int)(2*(90+f))] = tan(f*M_PI/180);
  }
}

float _myTAN(float r)
{
  float d = r * 180/M_PI;
  int d2 = (int)(2*(90+d));
  return tan_tab[d2];
}

