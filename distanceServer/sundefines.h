/*
// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/sundefines.h,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: sundefines.h,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1998/10/12 13:32:15  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:58:05  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.3  1997/09/29 16:03:09  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.2  1997/03/04 17:54:07  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//
//
// ----------------------------------------------------------------------------
*/

/* ensure stdlib to be loaded, because we will redefine RAND_MAX */
#include <stdlib.h>

#ifndef RAND_MAX
#define   RAND_MAX        2147483647
#endif

#ifndef MAXFLOAT
#define   MAXFLOAT        ((float)3.40282346638528860e+38)
#endif

