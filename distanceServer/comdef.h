// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/comdef.h,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: comdef.h,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1998/10/12 13:31:45  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:39  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.7  1997/09/25 13:33:39  schulz
//  trying to get obstacle network updates into the database
//
//  Revision 1.6  1997/06/27 08:56:00  hopp
//  picking added
//
//  Revision 1.5  1997/06/26 15:59:01  hopp
//  modify,delete added
//
//  Revision 1.4  1997/06/26 11:07:49  hopp
//  added header
//

/*

test

======================================
*** Message Definition  
======================================


======================================
*** Header
======================================
byte     command
         { 00 NOOP 
           01 CREATE
           02 MODIFY
           03 DELETE
           04 REQUEST
         }
byte     opcode
         { 01 OBJECT
           02 SURFACE 
           03 CAMERA
           00 REQUEST ALL
         }
short    len       (including header)



======================================
*** Create Camera Message
======================================

3dvector EyePint
3dVector lookAt
3dVector upVector
double   hfov
double   vfov



======================================
*** Create OBJECT Messages 
======================================
short    hirarchy 
         { 00 default 
         }
byte     object type
         { 01 box
           02 sphere
         }
short    uniqueObjectId

{ box
3dvec    center
double   halfx
double   halfy
double   halfz
4x3mat   matrix
short    surfaceId
byte     namelen
char     nametag[namelen]
}

{ sphere
3dvec    center
double   radius
short    surfid
byte     namelen
char     nametag[namelen]
}



*/


// COMMANDS
#define COMM_NOOP 0
#define COMM_CREATE 1
#define COMM_MODIFY 2
#define COMM_DELETE 3
#define COMM_REQUEST 4

// OPCODES
#define COMM_OBJECT 1
#define COMM_SURFACE 2
#define COMM_CAMERA 3
#define COMM_PICK 4
#define COMM_NAMEDOBJECT 5
#define COMM_ALL 0
// OBJECT TYPES
#define COMM_OBJECT_BOX 01
#define COMM_OBJECT_SPHERE 02
#define COMM_STATUS_CAMERA 01 

// SURFACE TYPES
#define COMM_SURFACE_PHONG 01

