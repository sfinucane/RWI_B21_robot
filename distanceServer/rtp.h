// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/rtp.h,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of RFC 1890 plus some changes for little endian
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: rtp.h,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1999/08/06 09:14:56  haehnel
//  minor changes for pc-solaris
//
//  Revision 1.1  1998/10/12 13:32:07  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:59  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.5  1997/06/27 11:26:37  schulz
//  inserted #ifnder _RTP_H wrapper
//
//  Revision 1.4  1997/06/26 15:59:37  hopp
//  MIPS Support
//
//  Revision 1.3  1997/06/13 09:49:03  schulz
//  rtp stuff -> C++
//
//  Revision 1.2  1997/03/04 17:54:01  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//
//
// ----------------------------------------------------------------------------

/*
 * rtp.h  --  RTP header file (RFC 1890)
 */

#ifndef _RTP_H
#define _RTP_H

#include <sys/types.h>

#if !(defined(_LITTLE_ENDIAN) || defined(_BIG_ENDIAN)) 
#if defined ( I386) || (WIN32) || (__linux__)
#define _LITTLE_ENDIAN
#else
#define _BIG_ENDIAN
#endif
#endif

/*
 * The type definitions below are valid for 32-bit architectures and
 * may have to be adjusted for 16- or 64-bit architectures.
 */
typedef unsigned char  u_int8;
typedef unsigned short u_int16;
typedef unsigned int   u_int32;
typedef          short int16;

/*
 * Current protocol version.
 */
#define RTP_VERSION    2

#define RTP_SEQ_MOD (1<<16)
#define RTP_MAX_SDES 255      /* maximum text length for SDES */

typedef enum {
    RTCP_SR   = 200,
    RTCP_RR   = 201,
    RTCP_SDES = 202,
    RTCP_BYE  = 203,
    RTCP_APP  = 204
} rtcp_type_t;

typedef enum {
    RTCP_SDES_END   = 0,
    RTCP_SDES_CNAME = 1,
    RTCP_SDES_NAME  = 2,
    RTCP_SDES_EMAIL = 3,
    RTCP_SDES_PHONE = 4,
    RTCP_SDES_LOC   = 5,
    RTCP_SDES_TOOL  = 6,
    RTCP_SDES_NOTE  = 7,
    RTCP_SDES_PRIV  = 8
} rtcp_sdes_type_t;

/*
 * RTP data header
 */
typedef struct {
#ifdef _BIG_ENDIAN
    unsigned int version:2;   /* protocol version */
    unsigned int p:1;         /* padding flag */
    unsigned int x:1;         /* header extension flag */
    unsigned int cc:4;        /* CSRC count */
    unsigned int m:1;         /* marker bit */
    unsigned int pt:7;        /* payload type */
#endif
#ifdef _LITTLE_ENDIAN
    unsigned int cc:4;        /* CSRC count */
    unsigned int x:1;         /* header extension flag */
    unsigned int p:1;         /* padding flag */
    unsigned int version:2;   /* protocol version */
    unsigned int pt:7;        /* payload type */
    unsigned int m:1;         /* marker bit */
#endif
    u_int16 seq;              /* sequence number */
    u_int32 ts;               /* timestamp */
    u_int32 ssrc;             /* synchronization source */
    u_int32 csrc[1];          /* optional CSRC list */
} rtp_hdr_t;

/*
 * RTCP common header word
 */
typedef struct {
#ifdef _BIG_ENDIAN
    unsigned int version:2;   /* protocol version */
    unsigned int p:1;         /* padding flag */
    unsigned int count:5;     /* varies by packet type */
    unsigned int pt:8;        /* RTCP packet type */
    u_int16 length;           /* pkt len in words, w/o this word */
#endif
#ifdef _LITTLE_ENDIAN
    unsigned int count:5;     /* varies by packet type */
    unsigned int p:1;         /* padding flag */
    unsigned int version:2;   /* protocol version */
    unsigned int pt:8;        /* RTCP packet type */
    u_int16 length;           /* pkt len in words, w/o this word */
#endif
} rtcp_common_t;

/*
 * Big-endian mask for version, padding bit and packet type pair
 */
#ifdef _BIG_ENDIAN
#define RTCP_VALID_MASK (0xc000 | 0x2000 | 0xfe)
#define RTCP_VALID_VALUE ((RTP_VERSION << 14) | RTCP_SR)
#endif
#ifdef _LITTLE_ENDIAN
#define RTCP_VALID_MASK (0x00c0 | 0x0020 | 0xfe00)
#define RTCP_VALID_VALUE ((RTP_VERSION << 6) | (RTCP_SR<<8))
#endif
/*
 * Reception report block
 */
typedef struct {
    u_int32 ssrc;             /* data source being reported */
#ifdef _BIG_ENDIAN    
    unsigned int fraction:8;  /* fraction lost since last SR/RR */
    int lost:24;              /* cumul. no. pkts lost (signed!) */
#endif
#ifdef _LITTLE_ENDIAN
    int lost:24;              /* cumul. no. pkts lost (signed!) */
    unsigned int fraction:8;  /* fraction lost since last SR/RR */
#endif
    u_int32 last_seq;         /* extended last seq. no. received */
    u_int32 jitter;           /* interarrival jitter */
    u_int32 lsr;              /* last SR packet from this source */
    u_int32 dlsr;             /* delay since last SR packet */
} rtcp_rr_t;

/*
 * SDES item
 */
typedef struct {
    u_int8 type;              /* type of item (rtcp_sdes_type_t) */
    u_int8 length;            /* length of item (in octets) */
    char data[1];             /* text, not null-terminated */
} rtcp_sdes_item_t;


typedef struct {
  u_int32 src;      /* first SSRC/CSRC */
  rtcp_sdes_item_t item[1]; /* list of SDES items */
} rtcp_sdes_t;


/*
 * One RTCP packet
 */
typedef struct {
    rtcp_common_t common;     /* common header */
    union {
        /* sender report (SR) */
        struct {
            u_int32 ssrc;     /* sender generating this report */
            u_int32 ntp_sec;  /* NTP timestamp */
            u_int32 ntp_frac;
            u_int32 rtp_ts;   /* RTP timestamp */
            u_int32 psent;    /* packets sent */
            u_int32 osent;    /* octets sent */ 
            rtcp_rr_t rr[1];  /* variable-length list */
        } sr;

        /* reception report (RR) */
        struct {
            u_int32 ssrc;     /* receiver generating this report */
            rtcp_rr_t rr[1];  /* variable-length list */
        } rr;

        /* source description (SDES) */
        rtcp_sdes_t sdes;

        /* BYE */
        struct {
            u_int32 src[1];   /* list of sources */
            /* can't express trailing text for reason */
        } bye;
    } r;
} rtcp_t;

#endif
