/*
// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/md5.h,v $
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
//  $Log: md5.h,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1998/10/12 13:31:56  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:51  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.4  1997/09/29 16:02:56  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.3  1997/06/13 09:49:02  schulz
//  rtp stuff -> C++
//
//  Revision 1.2  1997/03/04 17:53:55  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//
//
// ----------------------------------------------------------------------------
*/

#ifndef MD5_H
#define MD5_H

#ifdef __alpha
typedef unsigned int uint32;
#else
typedef unsigned long uint32;
#endif

struct MD5Context {
	uint32 buf[4];
	uint32 bits[2];
	unsigned char in[64];
};

#ifdef __cplusplus
extern "C" {
#endif
  
void MD5Init(struct MD5Context *context);
void MD5Update(struct MD5Context *context, unsigned char const *buf,
	       unsigned len);
void MD5Final(unsigned char digest[16], struct MD5Context *context);
void MD5Transform(uint32 buf[4], uint32 const in[16]);

#ifdef __cplusplus
}
#endif

/*
 * This is needed to make RSAREF happy on some MS-DOS compilers.
 */
typedef struct MD5Context MD5_CTX;

#endif /* !MD5_H */
