
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        compound.cc
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/compound.cc,v $
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
 * $Log: compound.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1996/12/04 14:25:27  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.2  1996/10/29 16:05:10  ws
 * changed some class names
 *
 * Revision 1.1.1.1  1996/09/30 16:17:44  schulz
 * reimport on new source tree
 *
 * Revision 1.2  1996/09/11 14:03:48  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
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

#include <math.h>
#include "compound.hh"

t_compobst::t_compobst(int n, float x, float y, float iori)
{
    type = 4;
    components = NULL;
    ori = iori;
    number = n;
    cx = x;
    cy = y;
}

Boolean t_compobst::distance(float x, float y,
				 float xe, float ye,
				 float* dist, float* ang)
{
    float tdist, tang;
    Boolean ret = FALSE;
    t_obstlist* trav = components;
    *dist = MAXFLOAT;
    while(trav) {
	ret |= trav->obstacle->distance(x,y,xe,ye,&tdist,&tang);
	if(tdist < *dist) {
	    *dist = tdist;
	    *ang = tang;
	}
	trav = trav->next;
    }
    return ret;
}

float t_compobst::min_distance(float x, float y)
{
    float tdist,dist;
    t_obstlist* trav = components;
    dist = MAXFLOAT;
    while(trav) {
	tdist = trav->obstacle->min_distance(x,y);
	if(tdist < dist)
	    dist = tdist;
	trav = trav->next;
    }
    return dist;
}

void t_compobst::bounds(float* xo1, float* yo1, float* xo2, float* yo2)
{
    float tx1,ty1,tx2,ty2;
    t_obstlist* trav = components;
    *xo1=MAXFLOAT;
    *yo1=MAXFLOAT;
    *xo2=MINFLOAT;
    *yo2=MINFLOAT;
    while(trav) {
	trav->obstacle->bounds(&tx1,&ty1,&tx2,&ty2);
	if(tx1 < *xo1)
	    *xo1 = tx1;
	if(ty1 < *yo1)
	    *yo1 = ty1;
	if(tx2 < *xo2)
	    *xo2 = tx2;
	if(ty2 < *yo2)
	    *yo2 = ty2;
	trav = trav->next;
    }
    return;
}

Boolean t_compobst::inside(float x, float y)
{
    t_obstlist* trav = components;
    while(trav) {
	if(trav->obstacle->inside(x,y))
	    return TRUE;
	trav = trav->next;
    }
    return FALSE;
}

void t_compobst::expose()
{
    t_obstlist* trav = components;
    while(trav) {
	trav->obstacle->expose();
	trav = trav->next;
    }
}
