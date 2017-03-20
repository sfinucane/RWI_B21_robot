// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/simrtp.hh,v $
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
//  $Log: simrtp.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1998/10/12 13:32:10  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/06/29 13:43:21  schulz
//  added lookForRobots sensor
//
//  Revision 1.1  1998/06/19 13:58:02  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.9  1997/09/18 15:11:10  schulz
//  changed ttl to 30
//
//  Revision 1.8  1997/09/09 14:06:20  schulz
//  minor changes
//
//  Revision 1.7  1997/09/09 12:57:32  schulz
//  added multiple camera support
//  hopefully fixed destroyed simrtp.hh and sim_rtp.cc again
//
//  Revision 1.6  1997/09/05 10:05:27  hopp
//  .entered SUN2_SUN compatibility
//  .
//
//  Revision 1.3  1997/06/19 09:12:51  schulz
//  major filelist cleanup
//
//  Revision 1.2  1997/06/17 12:34:19  schulz
//  minor enhancements and bug fixes
//
//  Revision 1.1  1997/06/13 09:53:54  schulz
//  rtp stuff -> C++
//
//  Revision 1.3  1997/03/06 16:06:26  schulz
//  debugging RTP stuff some printfs etc.
//
//  Revision 1.2  1997/03/04 17:54:02  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//
//
// ----------------------------------------------------------------------------

#ifndef SIMRTP_HH
#define SIMRTP_HH

#include <sys/socket.h>  /* struct sockaddr */
#include <netinet/in.h>  /* struct sockaddr_in */
#include "rtp.h"
#include <sys/types.h>
 #define MAX_PARTICIPANTS          20
 #define RTP_PACKET_BUFFER_SIZE 40000
 #define rtcp_bandwidth (double)  (8000.0 * 0.05)

/*
 * code taken from RFC 1889
 * Per-source state information
*/

typedef struct {
    u_int16 max_seq;        /* highest seq. number seen */
    u_int32 cycles;         /* shifted count of seq. number cycles */
    u_int32 base_seq;       /* base seq number */
    u_int32 bad_seq;        /* last 'bad' seq number + 1 */
    u_int32 probation;      /* sequ. packets till source is valid */
    u_int32 received;       /* packets received */
    u_int32 expected_prior; /* packet expected at last interval */
    u_int32 received_prior; /* packet received at last interval */
    u_int32 transit;        /* relative trans time for prev pkt */
    u_int32 jitter;         /* estimated jitter */
    u_int32 lsr;            /* rtp time of last SR */
    /* ... */
} source;


typedef struct {
    int rtp_sock_in;
    int rtp_sock_out;    
    int rtcp_sock_in;
    int rtcp_sock_out;    
    u_int32 ssrc;
    source src;
    int is_sender;
    char *cname;
    char *name;
    char *email;
    char *phone;
    char *location;
    char *tool;
    char *note;
    char *priv;
} rtp_session_member;

#define NO_INPUT 0
#define RTP_INPUT 1
#define RTCP_INPUT 2

class t_RTPChannel {
public:
  t_RTPChannel(char *name,
	       char *email,
	       char *phone,
	       char *location,
	       char *tool,
	       char *note,
	       char *priv,
	       char *multicast_address,
	       unsigned short port,
	       char ttl);
  void send_rtp_packet(char*, int);
  int send_rtcp_packet();

  int receive_rtp_packet(char *packet, int* len);
  int receive_rtp_packet(char *packet, int* len, ulong* sendId);
  int receive_rtcp_packet();

  u_int32      mySSRC();
  unsigned int octets_send();
  unsigned int octets_received();

  enum SDESItems {
    SDES_CNAME, SDES_NAME, SDES_EMAIL, SDES_PHONE, SDES_LOCATION,
    SDES_TOOL, SDES_NOTE, SDES_PRIV
  };

  // return an SDES-Item of the last sender
  char *sdes_item(SDESItems sdesItem);
  // return an SDES-Item of the sender with SSRC = ssrc
  char *sdes_item(SDESItems sdesItem, unsigned long src);  
  int rtp_socket();
  int rtcp_socket();

  float fraction_lost();
  int lost_packages();
  int session_members();
  void print_participants_cname();

  // returns NO_INPUT on failure and some of {RTP_INPUT, RTCP_INPUT}
  // otherwise
  int check_for_input();

  // debugging API will disapear later 
  void test_print_rtcp_packet(rtcp_common_t* p);
  void test_print_rtp_packet(rtp_hdr_t* p);
  void test_print_participants_cname();

protected:
 
  struct sockaddr_in rtp_sin;
  struct sockaddr_in rtcp_sin;

  rtp_session_member rtp_me;
  rtp_session_member participant[MAX_PARTICIPANTS];

  char rtp_packet_buffer[RTP_PACKET_BUFFER_SIZE];

  /* number of members curr. in the session */
  int  participant_cnt;

  /* who sent the last packet received */
  int rtp_last_sender;

  /* sequence number of the last RTP packet sent */
  u_int16 rtp_seq_number;                        

  /* number of packets sent sofar */
  u_int32 rtp_packet_cnt;

  /* dito for octets */  
  u_int32 rtp_octet_cnt;

  /* number of octets received sofar */
  u_int32 rtp_octets_received_cnt;           


  int rtcp_avg_packet_size;
  int rtcp_initial_report;

  // request a new entry for a participant
  int rtp_new_participant();

  // remove participant with index p (not SSRC!) 
  void rtp_remove_participant(int p);

  // lookup an existing participant
  int find_participant(u_int32 src);

  void rtp_initialize_participant(int p, int p_ssrc);
  int store_participant_sdes(int p, int type, char* data, int len);
  //int rtp_open_participant_sockets(int i, char* host, short rtp_port);


  void rtp_read_note(int p);
  int rtp_update_sdes_cname(int p, char *cname);
  u_int32 rtp_sender_cnt();
  u_int32 rtp_receiver_cnt();

  // the following functions compose RTP/RTCP packets

  void rtp_host_sdes_items(rtcp_t *r);
  void rtp_rtcp_rr_content(rtcp_rr_t* rr, int i);
  void rtp_rtp(rtp_hdr_t* h);
  int rtp_rtcp_sr(rtcp_t* r);
  int rtp_rtcp_rr(rtcp_t* r);
  int rtp_rtcp_sdes(rtcp_t* r, char* name, char* cname, char* note);
  void rtp_rtcp_bye(rtcp_t* r);

  // the following functions read RTP/RTCP packets

  int rtp_handle_rtp_packet(u_int32 len);
  void rtp_handle_rtcp_subpackets(u_int32 plen);
  int rtp_handle_rtcp_packet(u_int32 plen);

  // low level connection stuff
  
  int rtp_open_my_sockets(char* multicast_address, short rtcp_port, char ttl);
  int openrsock(int addr, unsigned short port, const struct sockaddr_in& local);
  int openssock(int addr, unsigned short port, int ttl);
};

#endif
