/* ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/linalg.h,v $
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
//  $Log: linalg.h,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.3  1999/10/28 09:19:27  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.2  1998/10/12 13:31:53  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:47  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.3  1997/03/06 10:44:23  schulz
//  Corrected file header
//
//  Revision 1.2  1997/03/04 17:53:53  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//
//
// --------------------------------------------------------------------------*/


#ifdef __cplusplus
typedef char Boolean;
extern "C" {
#endif
int
cut_line_and_line(float, float, 
		  float, float, 
		  float, float, 
		  float, float, 
		  float*, float*);
int 
cut_circle_and_line(float, float, float,
		    float, float,
		    float, float, 
		    float*, float*);
int
point_on_left_side(float px, float py,
		   float x1, float y1,
		   float x2, float y2);
extern void intersect_circleline(float, float, float, Boolean,
				 float, float, float,
				 Boolean*, float*, float*);
extern void intersect_rectline(float,float, float, float,
			       float, float, float, Boolean,
			       Boolean*, float*, float*);
extern float line_min_distance(float, float, float, float,
			       float, float);
extern void line_min_dist_point(float, float,
				float, float,
				float, float,
				float*, float*);
extern Boolean inhalfplane(float, float,
			float, float,
			float, float);
#ifdef __cplusplus
}
#endif
