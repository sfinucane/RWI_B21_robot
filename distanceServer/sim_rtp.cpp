/* ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/sim_rtp.cpp,v $
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
//  $Log: sim_rtp.cpp,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1999/05/20 13:34:00  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:32:09  schulz
//  This is a complete new version
//
//  Revision 1.3  1998/08/08 23:05:36  schulz
//  Fixed some NULL Pointer problems
//
//  Revision 1.2  1998/06/29 13:43:21  schulz
//  added lookForRobots sensor
//
//  Revision 1.1  1998/06/19 13:58:02  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.9  1997/10/21 12:51:45  schulz
//  still cleaningup for first release
//
//  Revision 1.8  1997/09/29 16:03:05  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.7  1997/09/18 15:11:08  schulz
//  changed ttl to 30
//
//  Revision 1.6  1997/09/09 12:57:32  schulz
//  added multiple camera support
//  hopefully fixed destroyed simrtp.hh and sim_rtp.cc again
//
//  Revision 1.5  1997/09/05 10:05:59  hopp
//  changed defines for SUN2_SUN compatibility
//  .
//
//  Revision 1.7  1997/06/19 09:12:49  schulz
//  major filelist cleanup
//
//  Revision 1.6  1997/06/18 15:13:23  schulz
//  fixed composition of sdes packets
//
//  Revision 1.5  1997/06/18 12:51:46  schulz
//  debugging stuff
//
//  Revision 1.4  1997/06/18 12:30:49  schulz
//  fixed a bug in t_RTPChannel constructor
//
//  Revision 1.3  1997/06/18 12:00:55  schulz
//  fixed some nasty endian related bugs in sim_rtp.cc
//
//  Revision 1.2  1997/06/17 12:34:18  schulz
//  minor enhancements and bug fixes
//
//  Revision 1.1  1997/06/13 09:53:53  schulz
//  rtp stuff -> C++
//
//  Revision 1.3  1997/03/06 16:06:24  schulz
//  debugging RTP stuff some printfs etc.
//
//  Revision 1.2  1997/03/04 17:40:00  schulz
//  Added file header to *.c files
//  Removed some old files
//
//
//
// --------------------------------------------------------------------------*/

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>       /* stderr, printf() */
#include <sys/time.h>    /* select and co. */
#include <sys/types.h>
#include <sys/utsname.h>
#include <netdb.h>
#include <unistd.h>
#include <pwd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#ifndef LINUX 
#include <sys/systeminfo.h>
#endif

#include "md5.h"
#include "simrtp.hh"


int const MAX_DROPOUT = 3000; 
int const MIN_SEQUENTIAL = 1;
int const MAX_MISORDER = 100;


static u_int32 rtp_current_time();

/******************************* some code taken from RFC 1889 *****************************************/

/* The RFC suggests the use of MD5 to generate random SSRC values */
/* The code is copied here                                        */

#define MD_CTX MD5_CTX
#define MDInit MD5Init
#define MDUpdate MD5Update
#define MDFinal MD5Final

static u_long
md_32(char *string, int length)
{
  MD_CTX context;
  union {
    char   c[16];
    u_long x[4];
  } digest;
  u_long r;
  int i;
  
  MDInit (&context);
  MDUpdate (&context, (unsigned char*)string, length);
  MDFinal ((unsigned char *)&digest, &context);
  r = 0;
  for (i = 0; i < 3; i++) {
    r ^= digest.x[i];
  }
  return r;
}                               /* md_32 */


/*
* Return random unsigned 32-bit quantity. Use 'type' argument if you
* need to generate several different values in close succession.
*/

static u_int32
random32(int type)
{
  struct {
    int     type;
    struct  timeval tv;
    clock_t cpu;
    pid_t   pid;
    u_long  hid;
    uid_t   uid;
    gid_t   gid;
    struct  utsname name;
  } s;
  
  gettimeofday(&s.tv, 0);
  uname(&s.name);
  s.type = type;
  s.cpu  = clock();
  s.pid  = getpid();
#ifndef LINUX 
  char buf[128];
  if (sysinfo(SI_HW_SERIAL,buf,128) == -1) fprintf(stderr,"ERROR  random32\n"); 
     s.hid = strtoul(buf,NULL,0);
#else
  s.hid  = gethostid();
#endif
  s.uid  = getuid();
  s.gid  = getgid();
  
  return md_32((char *)&s, sizeof(s));
}                               /* random32 */


/* sequence initialisation and update for a given packet source */

static void
init_seq(source *s, u_int16 seq)
{
    s->base_seq = seq - 1;
    s->max_seq = seq;
    s->bad_seq = RTP_SEQ_MOD + 1;
    s->cycles = 0;
    s->received = 0;
    s->received_prior = 0;
    s->expected_prior = 0;
    /* other initialization */
}

static int
update_seq(source *s, u_int16 seq)
{
    u_int16 udelta = seq - s->max_seq;
    
    /*
     * Source is not valid until MIN_SEQUENTIAL packets with
     * sequential sequence numbers have been received.
     */
    if (s->probation) {
	/* packet is in sequence */
	if (seq == s->max_seq + 1) {
	    s->probation--;
	    s->max_seq = seq;
	    if (s->probation == 0) {
		init_seq(s, seq);
		s->received++;
		return 1;
	    }
	} else {
	    s->probation = MIN_SEQUENTIAL - 1;
	    s->max_seq = seq;
	}
	return 0;
    } else if (udelta < MAX_DROPOUT) {
	/* in order, with permissible gap */
	if (seq < s->max_seq) {
	    /*
	     * Sequence number wrapped - count another 64K cycle.
	     */
	    s->cycles += RTP_SEQ_MOD;
	}
	s->max_seq = seq;
    } else if (udelta <= RTP_SEQ_MOD - MAX_MISORDER) {
	/* the sequence number made a very large jump */
	if (seq == s->bad_seq) {
	    /*
	     * Two sequential packets -- assume that the other side
	     * restarted without telling us so just re-sync
	     * (i.e., pretend this was the first packet).
	     */
	    init_seq(s, seq);
	}
	else {
	    s->bad_seq = (seq + 1) & (RTP_SEQ_MOD-1);
	    return 0;
	}
    } else {
	/* duplicate or reordered packet */
    }
    s->received++;
    return 1;
}

static double
rtcp_interval(int members,
		     int senders,
		     double rtcp_bw,
		     int we_sent,
		     int packet_size,
		     int *avg_rtcp_size,
		     int initial)
{
    /*
     * Minimum time between RTCP packets from this site (in seconds).
     * This time prevents the reports from `clumping' when sessions
     * are small and the law of large numbers isn't helping to smooth
     * out the traffic.  It also keeps the report interval from
     * becoming ridiculously small during transient outages like a
     * network partition.
     */
    double const RTCP_MIN_TIME = 5.;
    /*
     * Fraction of the RTCP bandwidth to be shared among active
     * senders.  (This fraction was chosen so that in a typical
     * session with one or two active senders, the computed report
     * time would be roughly equal to the minimum report time so that
     * we don't unnecessarily slow down receiver reports.) The
     * receiver fraction must be 1 - the sender fraction.
     */
    double const RTCP_SENDER_BW_FRACTION = 0.25;
    double const RTCP_RCVR_BW_FRACTION = (1-RTCP_SENDER_BW_FRACTION);

    /*
     * Gain (smoothing constant) for the low-pass filter that
     * estimates the average RTCP packet size (see Cadzow reference).
     */
    double const RTCP_SIZE_GAIN = (1./16.);
    
    double t;                   /* interval */
    double rtcp_min_time = RTCP_MIN_TIME;
    int n;                      /* no. of members for computation */


    /*
     * Very first call at application start-up uses half the min
     * delay for quicker notification while still allowing some time
     * before reporting for randomization and to learn about other
     * sources so the report interval will converge to the correct
     * interval more quickly.  The average RTCP size is initialized
     * to 128 octets which is conservative (it assumes everyone else
     * is generating SRs instead of RRs: 20 IP + 8 UDP + 52 SR + 48
     * SDES CNAME).
     */
    if (initial) {
	rtcp_min_time /= 2;
	*avg_rtcp_size = 128;
    }
    
    /*
     * If there were active senders, give them at least a minimum
     * share of the RTCP bandwidth.  Otherwise all participants share
     * the RTCP bandwidth equally.
     */
    n = members;
    if (senders > 0 && senders < members * RTCP_SENDER_BW_FRACTION) {
	if (we_sent) {
	    rtcp_bw *= RTCP_SENDER_BW_FRACTION;
	    n = senders;
	} else {
	    rtcp_bw *= RTCP_RCVR_BW_FRACTION;
	    n -= senders;
	}
    }
    
    /*
     * Update the average size estimate by the size of the report
     * packet we just sent.
     */
    *avg_rtcp_size += (int)((packet_size - *avg_rtcp_size)*RTCP_SIZE_GAIN);
    
    /*
     * The effective number of sites times the average packet size is
     * the total number of octets sent when each site sends a report.
     * Dividing this by the effective bandwidth gives the time
     * interval over which those packets must be sent in order to
     * meet the bandwidth target, with a minimum enforced.  In that
     * time interval we send one report so this time is also our
     * average time between reports.
     */
    t = (*avg_rtcp_size) * n / rtcp_bw;
    if (t < rtcp_min_time) t = rtcp_min_time;

    /*
     * To avoid traffic bursts from unintended synchronization with
     * other sites, we then pick our actual next report interval as a
     * random number uniformly distributed between 0.5*t and 1.5*t.
     */
    return t * (drand48() + 0.5);
}

static void
rtp_estimate_jitter(source* s, rtp_hdr_t* r)
{
    int transit = rtp_current_time() - r->ts;
    int d = transit - s->transit;
    s->transit = transit;
    if(d < 0) d = -d;
    s->jitter += (unsigned int)((1./16.) * ((double)d - s->jitter));

}

static u_int8  
rtp_calculate_loss(source* s, u_int32* lost)
{
    u_int32 extended_max = s->cycles + s->max_seq;
    u_int32 expected = extended_max - s->base_seq +1;
    u_int32 expected_interval;
    u_int32 received_interval;
    u_int32 lost_interval;
    u_int8 fraction;
    *lost = expected - s->received;
    if(*lost > (u_int32)0xffffff) *lost = 0xffffff;
    expected_interval = expected - s->expected_prior;
    s->expected_prior = expected;
    received_interval = s->received - s->received_prior;
    lost_interval = expected_interval - received_interval;
    if(expected_interval == 0 || lost_interval <= 0) fraction = 0;
    else fraction = (lost_interval << 8) / expected_interval;
    return fraction;
}


/* read only version of the above function, used for statistics */

static u_int8  
rtp_calculate_loss_ro(source* s, u_int32* lost)
{
    u_int32 extended_max = s->cycles + s->max_seq;
    u_int32 expected = extended_max - s->base_seq +1;
    u_int32 expected_interval;
    u_int32 received_interval;
    u_int32 lost_interval;
    u_int8 fraction;
    *lost = expected - s->received;
    if(*lost > (u_int32)0xffffff) *lost = 0xffffff;
    expected_interval = expected - s->expected_prior;
    received_interval = s->received - s->received_prior;
    lost_interval = expected_interval - received_interval;
    if(expected_interval == 0 || lost_interval <= 0) fraction = 0;
    else fraction = (lost_interval << 8) / expected_interval;
    return fraction;
}


/* end of RFC sample code */
/* -----------------------------------------------------------------------------------------*/


/* RFC 1889 wants time to be given in a reduced NTP format */
/* The following three functions convert between struct timeval and the special format */ 
      
static void
gettimeofday_ntp(u_int32 *t)
{
  struct timeval tv;

  gettimeofday(&tv, 0);
  t[0] = tv.tv_sec  + 2208988800U;
  t[1] = (unsigned int)(tv.tv_usec * 4294.967296);
} /* gettimeofday_ntp */


static u_int32
timeval_ntp32(struct timeval *tv)
{
  return ((tv->tv_sec  + 2208988800U) << 16) | 
         ((u_long)(tv->tv_usec * 4294.967296) >> 16);
} /* timeval_ntp32 */

static u_int32
rtp_current_time()
{
  struct timeval now;
  gettimeofday(&now, NULL);
  return ((now.tv_sec & 0x0000ffff) * 8000) + (now.tv_usec / 125);
  //  return timeval_ntp32(&now);
}

/* next thing to do is conversion between network and host data formats for all kinds of packages */
/* however some of these conversion routines have severe side effects !!! */
static void
network_long(u_int32* p)
{
  *p = htonl(*p); 
}

static void
host_long(u_int32* p)
{
    *p = ntohl(*p);  
}

static void
host_short(u_int16* p)
{
    *p = ntohs(*p);  
}

/* rtp_host_sdes_items(rtcp_t *r) converts the RTCP sdes packet pointed to by r into host format.
 * It stores the sdes items found into the participants rtp_session_member struct.
 * If the sender is not yet known, a participant entry will be allocated.
 * It will print a warning if the packet is corrupted.
 */

void
t_RTPChannel::rtp_host_sdes_items(rtcp_t *r)
{
  int count = r->common.count;
  rtcp_sdes_t *sd = (rtcp_sdes_t*) &(r->r.sdes);
  rtcp_sdes_item_t *rsp, *rspn;
  rtcp_sdes_item_t *end = (rtcp_sdes_item_t *)
    ((u_int32 *)r + r->common.length + 1);
  int s;

  while (--count >= 0) {
    rsp = &sd->item[0];
    if (rsp >= end) break;
    rtp_last_sender = s = find_participant(sd->src);
    
    if(s < 0) {
      s = rtp_new_participant();
      if(s < 0) {
	fprintf(stderr, "Too many participants, ignoring new one\n");
	return;
	}
      else {
	  rtp_initialize_participant(s, sd->src);
      }
    }
    for (; rsp->type; rsp = rspn ) {
      rspn = (rtcp_sdes_item_t *)((char*)rsp+rsp->length+2);
      if (rspn >= end) {
	rsp = rspn;
	break;
      }
      store_participant_sdes(s, rsp->type, rsp->data, rsp->length);
    }
    sd = (rtcp_sdes_t *)
      ((u_int32 *)sd + (((char *)rsp - (char *)sd) >> 2)+1);
  }
  if (count >= 0) {
    fprintf(stderr, "Got Corrupted RTCP_SDES packet\n");  
  }
}

/* convert RTP packet header into host format
 * note the bit array fields of the header need not be converted,
 * because their order is switched in the rtp.h header file to suit
 * the actual machines endian type.
 */

static void
rtp_host_rtp(rtp_hdr_t* h)
{
  host_short(&(h->seq));
  host_long(&(h->ts));
  host_long(&(h->ssrc));
  host_long(&(h->csrc[0]));    
}

/* convert RTCP receiver report content into host format
 * note the bit array fields of the packet need not be converted,
 * because their order is switched in the rtp.h header file to suit
 * the actual machines endian type.
 */

static void
rtp_host_rtcp_rr_content(rtcp_rr_t* rr)
{
  host_long(&rr->ssrc);
  host_long(&rr->last_seq);
  host_long(&rr->jitter);
  host_long(&rr->lsr);
  host_long(&rr->dlsr);      
}


/* convert RTCP sender report into host format */

static int
rtp_host_rtcp_sr(rtcp_t* r)
{
  int i,k;
  k = (int)r->common.length*4 - sizeof(r->r.sr);
  if( (k < 0) || (k % sizeof(rtcp_rr_t) != 0) ) {
    fprintf(stderr, "got malformed RTCP_SR packet!\n");
    return 0;
  }
  k = (r->common.length*4 - sizeof(r->r.sr))/sizeof(rtcp_rr_t); 
  host_long(&r->r.sr.ssrc);
  host_long(&r->r.sr.ntp_sec);
  host_long(&r->r.sr.ntp_frac);
  host_long(&r->r.sr.rtp_ts);
  host_long(&r->r.sr.psent);
  host_long(&r->r.sr.osent);
  for(i = 0; i < k; i++)
    rtp_host_rtcp_rr_content(&r->r.sr.rr[i]);
  return 1;
}

/* convert a RTCP receiver report into host format 
 * a receiver report is sent instead of a sender report, if the participant
 * did not send anything since the last RTCP packet.
 */

static void
rtp_host_rtcp_rr(rtcp_t* r)
{
  int i,k;
  k = (r->common.length*4)/sizeof(rtcp_rr_t); 
  host_long(&r->r.rr.ssrc);
  for(i = 0; i < k; i++)
    rtp_host_rtcp_rr_content(&r->r.rr.rr[i]);
}

/* convert a RTCP sdes header into host format */

static void
rtp_host_rtcp_sdes(rtcp_t* r)
{
  host_long(&r->r.sdes.src);
}

/* convert a RTCP bye packet into host format */

static void
rtp_host_rtcp_bye(rtcp_t* r)
{
  host_long(&r->r.bye.src[0]);
}

/* convert the common RTCP header into host format 
 * note the bit array fields of the header need not be converted,
 * because their order is switched in the rtp.h header file to suit
 * the actual machines endian type.
 */
   
static void
rtp_host_rtcp_common(rtcp_t* r)
{
  
  host_short(&r->common.length);
}

/* --------------------------------------------------------------------------------------- */


/* rtp_new_participant() returns the next free entry number of the participant array */
/* It returns -1 if we are full */

int
t_RTPChannel::rtp_new_participant()
{
  int i = -1;
  if(participant_cnt < MAX_PARTICIPANTS) {
    i = participant_cnt;
    participant_cnt++;
  }
  else 
    fprintf(stderr,
	    "Number of participants exceeded ignoring new ones!\n");
  return i;
}

/* remove a participant from the session */

void
t_RTPChannel::rtp_remove_participant(int p)
{
    int i;
    if( p < 0) return;
    if(p >= participant_cnt) return;
    for(i = p; i < participant_cnt-1; i++) {
	participant[i] = participant[i+1];
    }
    participant_cnt--;
}


/* find_participant(u_int32 src) returns the participant index of the participant with SSRC src.
 * It returns -1 if such a participant does not exist
 */

int
t_RTPChannel::find_participant(u_int32 src)
{
  int i, found = 0;
  for(i = 0; i < participant_cnt; i++) {
    if(participant[i].ssrc == src) {
      return i;
    }
  }
  return -1;
}



/* rtp_cname() returns the programs own CNAME.
 * The CNAME is composed of <users loginname>@<full hostname>
 * This can be overridden by specifying the RTPCNAME environment variable.
 */

static char*
rtp_cname()
{
  static char rtp_my_cname[256];
  char *p_name;
  if((p_name = getenv("RTPCNAME")) == NULL) {
    struct utsname uts;
    struct hostent *myent;
    struct passwd *mypwd_ent;
    uid_t myid;
    uname(&uts);
    myid = getuid();
    mypwd_ent = getpwuid(myid);
    myent = gethostbyname(uts.nodename);
    sprintf(rtp_my_cname, "%s@%s",mypwd_ent->pw_name,myent->h_name);
    return rtp_my_cname;
  }
  return p_name;
}


/* initialize rtp_session_meber with index p and SSRC p_ssrc with default values */

void
t_RTPChannel::rtp_initialize_participant(int p, int p_ssrc)
{
    participant[p].ssrc = p_ssrc;
    participant[p].is_sender = 0;
    participant[p].cname = NULL;
    participant[p].name = NULL;
    participant[p].email = NULL;
    participant[p].phone = NULL;
    participant[p].location = NULL;
    participant[p].tool = NULL;        
    participant[p].note = NULL;
    participant[p].priv = NULL;                    
}


/* RFC 1889 says session members are uniquely determined by their CNAME 
 * The SSRC of a session member may change from time to time
 * (for example due to a client crash and reinitialization). So we have to ensure,
 * that one session member is not treated as multiple participants.
 * This done by rtp_update_sdes_cname, every time a SDES cname item is received.
 */

int
t_RTPChannel::rtp_update_sdes_cname(int p, char *cname)
{
    int i;
    if(participant[p].cname) {               /* test if old information */
	if(strcmp(participant[p].cname,cname) == 0) {
	    free(participant[p].cname);
	    participant[p].cname = cname;
	    return 1;
	}
    }
                   /* check if cname is in use by somebody else */
    for(i = 0; i < participant_cnt; i++) {
	if(!participant[i].cname) continue; /* not yet known for example i==p*/
	if(strcmp(participant[i].cname, cname) == 0) {   /* found duplicate entry */
	    participant[i].ssrc = participant[p].ssrc;
	    rtp_remove_participant(p);                   /* remove one of them */
	    return 0;
	}
    }
                   /* cname is free for participant p */
    if(participant[p].cname)
	free(participant[p].cname);
    participant[p].cname = cname;
    return 1;
}


int
t_RTPChannel::store_participant_sdes(int p, int type, char* data, int len)
{
  char *item;
  item = new char[len+1];
  memcpy(item, data, len);
  item[len] = 0;
  switch(type) {
  case RTCP_SDES_CNAME:
      if(!rtp_update_sdes_cname(p, item)) {
	  free(item);
	  return 0;
      }
      break;
  case RTCP_SDES_NAME:
    if(participant[p].name) free(participant[p].name);
    participant[p].name = item;
    break;
  case RTCP_SDES_EMAIL:
    if(participant[p].email) free(participant[p].email);
    participant[p].email = item;
    break;
  case RTCP_SDES_PHONE:
    if(participant[p].phone) free(participant[p].phone);
    participant[p].phone = item;
    break;
  case RTCP_SDES_LOC:
    if(participant[p].location) free(participant[p].location);
    participant[p].location = item;
    break;
  case RTCP_SDES_TOOL:
    if(participant[p].tool) free(participant[p].tool);
    participant[p].tool = item;
    break;
  case RTCP_SDES_NOTE:
    if(participant[p].note) free(participant[p].note);
    participant[p].note = item;
    break;
  case RTCP_SDES_PRIV:
    if(participant[p].priv) free(participant[p].priv);
    participant[p].priv = item;
    break;
  case RTCP_SDES_END:
  default: break;
  }
  return 1;
}

/* count the number of active senders in the session */
/* needed for RTCP interval calculation */

u_int32
t_RTPChannel::rtp_sender_cnt()
{
    int i;
    u_int32 j = 0;
    for(i = 0; i < participant_cnt; i++)
	if(participant[i].is_sender) j++;
    return j;
}

/* dito for receivers only */

u_int32
t_RTPChannel::rtp_receiver_cnt()
{
    int i;
    u_int32 j = 0;
    for(i = 0; i < participant_cnt; i++)
	if(!participant[i].is_sender) j++;
    return j;
}

/* ------------------------ composition of RTP / RTCP packets ------------------------- */

/*
* Generate RTP data packet.
*/

void
t_RTPChannel::rtp_rtp(rtp_hdr_t* h)
{
    h->version = RTP_VERSION;
    h->p = 0;
    h->x = 0;
    h->cc = 0;
    h->m = 0;
    h->pt = 0;
    h->seq = htons(rtp_seq_number);
    h->ts = htonl(rtp_current_time());
    h->ssrc = htonl(rtp_me.ssrc);
    h->csrc[0] = htonl(0);
}

void
t_RTPChannel::rtp_rtcp_rr_content(rtcp_rr_t* rr, int i)
{
    u_int32 lost;
    u_int8 fraction = rtp_calculate_loss(&participant[i].src, &lost);
    rr->ssrc = htonl(participant[i].ssrc);
    rr->fraction = fraction;
    rr->lost = lost;
    rr->last_seq = htonl(participant[i].src.cycles +
			 participant[i].src.max_seq);
    rr->jitter = htonl(participant[i].src.jitter);
    rr->lsr = htonl(participant[i].src.lsr);
    rr->dlsr = htonl(rtp_current_time()-participant[i].src.lsr);
}

int
t_RTPChannel::rtp_rtcp_sr(rtcp_t* r)
{
  int i;
  int len;
  u_int32 sec, frac;
  struct { u_int32 sec, frac;} now;
  gettimeofday_ntp((u_int32*)&now);
  r->common.length  = 0;
  r->common.count   = participant_cnt;
  r->common.version = RTP_VERSION;
  r->common.p       = 0;
  r->common.pt      = RTCP_SR;
  len = 
      sizeof(r->common)
      + sizeof(r->r.sr)
      + participant_cnt * sizeof(rtcp_rr_t);
  if(len%4) len = (len/4 + 1) * 4;
  r->common.length  = htons(len/4 - 1);

  r->r.sr.ssrc = htonl(rtp_me.ssrc);
  r->r.sr.ntp_sec = htonl(now.sec);
  r->r.sr.ntp_frac = htonl(now.frac);    
  r->r.sr.rtp_ts = htonl(rtp_current_time());
  r->r.sr.psent = htonl(rtp_packet_cnt);
  r->r.sr.osent = htonl(rtp_octet_cnt);
  for(i = 0; i < participant_cnt; i++) {
      participant[i].is_sender = 0;
      rtp_rtcp_rr_content(&(r->r.sr.rr[i]),i);
  }
  return len;
}

int
t_RTPChannel::rtp_rtcp_rr(rtcp_t* r)
{
  int i;
  int len;
  
  r->common.length  = 0;
  r->common.count   = participant_cnt;
  r->common.version = RTP_VERSION;
  r->common.p       = 0;
  r->common.pt      = RTCP_RR;
  len = 
    sizeof(r->common) +
      sizeof(r->r.rr) +
	participant_cnt * sizeof(rtcp_rr_t);
  if(len%4) len = (len/4 + 1) * 4;
  r->common.length  = htons(len/4 - 1);

  r->r.rr.ssrc = htonl(rtp_me.ssrc);
  for(i = 0; i < participant_cnt; i++) {
      rtp_rtcp_rr_content(&(r->r.rr.rr[i]),i);
  }
  return len;
}


int
t_RTPChannel::rtp_rtcp_sdes(rtcp_t* r, char* name, char* text, char* note)
{
  int len;
  rtcp_sdes_item_t *item;
  r->common.length  = 0;
  r->common.count   = 1;
  r->common.version = RTP_VERSION;
  r->common.p       = 0;
  r->common.pt      = RTCP_SDES;
  len = 
    sizeof(r->common)
      + sizeof(r->r.sdes) 
      + sizeof(rtcp_sdes_item_t) + strlen(text)
      + sizeof(rtcp_sdes_item_t) + strlen(note)
      + sizeof(rtcp_sdes_item_t) + strlen(name)
      + 1;
  if(len%4) len = (len/4+1)*4;
  r->common.length  = htons(len/4 - 1);
  
  r->r.sdes.src = htonl(rtp_me.ssrc);
  item = &(r->r.sdes.item[0]);
  item->type = RTCP_SDES_CNAME;
  item->length = strlen(text);
  strcpy(item->data, text);
  item = (rtcp_sdes_item_t*)(item->data + item->length);
  item->type = RTCP_SDES_NOTE;
  item->length = strlen(note);  
  strcpy(item->data, note);
  item = (rtcp_sdes_item_t*)(item->data + item->length);
  item->type = RTCP_SDES_NAME;
  item->length = strlen(name);  
  strcpy(item->data, name);
  item = (rtcp_sdes_item_t*)(item->data + item->length);
  char *c = (char*) item;
  do {
    *c++ = 0;
  }  while(((int) c) % 4); 
  return len;
}


void
t_RTPChannel::rtp_rtcp_bye(rtcp_t* r)
{
  int len;
  r->common.length  = 0;
  r->common.count   = 0;
  r->common.version = RTP_VERSION;
  r->common.p       = 0;
  r->common.pt      = RTCP_BYE;
  len = 
    sizeof(r->common)
      + sizeof(r->r.bye);
  if(len%4) len = (len/4+1)*4;
  r->common.length  = htons(len/4 - 1);
  r->r.bye.src[0] = htonl(rtp_me.ssrc);
}


/* ----------------------------- functions for receiving RTP /RTCP packets --------------------- */ 

int
t_RTPChannel::rtp_handle_rtp_packet(u_int32 len)
{
    int i, found = 0;
    rtp_hdr_t* p = (rtp_hdr_t*) rtp_packet_buffer;

    if(len < sizeof(rtp_hdr_t)) {
	fprintf(stderr, "RTP packet too short! < %d\n", sizeof(rtp_hdr_t));
    }
    rtp_host_rtp(p);

    /* Checking header consistency */
    
    if(p->version != 2) {
	fprintf(stderr, "RTP packet has wrong version!\n");
	return 0;
    }
    if(p->x ==  1) {
	fprintf(stderr, "Header has extension, not supported\n");
	return 0;
    }
    if(p->cc != 0) {
	fprintf(stderr, "Oops, CC != 0 should not happen!\n");
	return 0;
    }
    if(p->m != 0 || p->pt != 0) {
	fprintf(stderr, "Oops, do not use paylod type yet, but found one\n");
	return 0;
    }

    for(i = 0; i < participant_cnt; i++) {
	if(p->ssrc == participant[i].ssrc) {
	    found = 1;
	    break;
	}
    }
    if(!found) {
      i = rtp_new_participant();
      if(i >= 0) {
	  rtp_initialize_participant(i, p->ssrc);
	  participant[i].is_sender = 1;
	  init_seq(&participant[i].src, p->seq);
	  participant[i].src.probation = MIN_SEQUENTIAL;
      }
    }
    if(update_seq(&participant[i].src, p->seq)) {
	rtp_estimate_jitter(&participant[i].src, p);
	return 1;
    }
    else {
	fprintf(stderr, "RTP sequence missmatch, ignoring packet\n");
	return 0;
    }
}


void
t_RTPChannel::rtp_handle_rtcp_subpackets(u_int32 plen)
{
  u_int32 len;        /* length of compound RTCP packet in words */
  rtcp_t *r;          /* RTCP header */
  rtcp_t *end;        /* end of compound RTCP packet */

  len = plen / 4;
  if(plen % sizeof(u_int32)) len += 1;
  r = (rtcp_t*) rtp_packet_buffer;
  end = (rtcp_t *)((u_int32 *)r + len);

  do {
    switch(r->common.pt) {
    case RTCP_SR:
      rtp_host_rtcp_sr(r);
      break;
    case RTCP_RR:
      rtp_host_rtcp_rr(r);
      break;
    case RTCP_SDES:
      rtp_host_rtcp_sdes(r);
      rtp_host_sdes_items(r);
      break;
    case RTCP_BYE:
      rtp_host_rtcp_bye(r);
      break;
    case RTCP_APP:
    default:
      break;
    }
    r = (rtcp_t *)((u_int32 *)r + r->common.length + 1);
  }
  while (r < end && r->common.version == 2);
}

int
t_RTPChannel::rtp_handle_rtcp_packet(u_int32 plen) 
{
    
       u_int32 len;        /* length of compound RTCP packet in words */
       rtcp_t *r;          /* RTCP header */
       rtcp_t *end;        /* end of compound RTCP packet */
       rtcp_common_t test_header; 
       len = plen / 4;
       if(plen % sizeof(u_int32)) len += 1;
       r = (rtcp_t*) rtp_packet_buffer;

       test_header = r->common;
       test_print_rtcp_packet(&r->common);

       if ((*(u_int16 *)&test_header & RTCP_VALID_MASK) != RTCP_VALID_VALUE) {
	   fprintf(stderr, "RTCP header corrupted ignoring packet!\n");
	   return 0;
       }

       end = (rtcp_t *)((u_int32 *)r + len);

       do {
	   rtp_host_rtcp_common(r);
	   r = (rtcp_t *)((u_int32 *)r + r->common.length + 1);
       }
       while (r < end && r->common.version == 2);
       
       if (r != end) {
	   fprintf(stderr, "RTCP packet corrupted ignoring!\n");	   
	   return 0;
       }
       rtp_handle_rtcp_subpackets(plen);
       return 1;
}

/* ----------------------------------- socket initilization stuff ------------------------------- */

int
t_RTPChannel::openrsock(int addr, unsigned short port, const struct sockaddr_in& local)
{
  int fd;
  struct sockaddr_in sin;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    perror("socket");
    exit(1);
  }
  //  nonblock(fd);
  int on = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *)&on,
		 sizeof(on)) < 0) {
    perror("SO_REUSEADDR");
  }
#ifdef SO_REUSEPORT
  on = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, (char *)&on,
		 sizeof(on)) < 0) {
    perror("SO_REUSEPORT");
    exit(1);
  }
#endif
  memset((char *)&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = htons(port);
#ifdef IP_ADD_MEMBERSHIP
  if (IN_CLASSD(ntohl(addr))) {
    /*
     * Try to bind the multicast address as the socket
     * dest address.  On many systems this won't work
     * so fall back to a destination of INADDR_ANY if
     * the first bind fails.
     */
    
    sin.sin_addr.s_addr = addr;
    if (bind(fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
      sin.sin_addr.s_addr = INADDR_ANY;
      if (bind(fd, (struct sockaddr*)&sin, sizeof(sin)) < 0) {
	perror("bind");
	exit(1);
      }
    }
    /* 
     * XXX This is bogus multicast setup that really
     * shouldn't have to be done (group membership should be
     * implicit in the IP class D address, route should contain
     * ttl & no loopback flag, etc.).  Steve Deering has promised
     * to fix this for the 4.4bsd release.  We're all waiting
     * with bated breath.
     */

    struct ip_mreq mr;

    mr.imr_multiaddr.s_addr = addr;
    mr.imr_interface.s_addr = INADDR_ANY;
    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, 
		   (char *)&mr, sizeof(mr)) < 0) {
      perror("IP_ADD_MEMBERSHIP");
      exit(1);
    }

  } else
#endif
    {
      /*
       * bind the local host's address to this socket.  If that
       * fails, another vic probably has the addresses bound so
       * just exit.
       */
      sin.sin_addr.s_addr = local.sin_addr.s_addr;
      if (bind(fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
	perror("bind");
	exit(1);
      }
      /*
       * Despite several attempts on our part to get this fixed,
       * Microsoft Windows isn't complient with the Internet Host
       * Requirements standard (RFC-1122) and won't let us include
       * the source address in the receive socket demux state.
       * (The consequence of this is that all conversations have
       * to be assigned a unique local port so the vat 'side
       * conversation' (middle click on site name) function is
       * essentially useless under windows.)
       */
#ifndef WIN32
      /*
       * (try to) connect the foreign host's address to this socket.
       */
      sin.sin_port = 0;
      sin.sin_addr.s_addr = addr;
      connect(fd, (struct sockaddr *)&sin, sizeof(sin));
#endif
    }
  /*
   * XXX don't need this for the session socket.
   */     
  int bufsize = 80 * 1024;
  if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsize,
		 sizeof(bufsize)) < 0) {
    bufsize = 32 * 1024;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsize,
		   sizeof(bufsize)) < 0)
      perror("SO_RCVBUF");
  }
  return (fd);
}

int
t_RTPChannel::openssock(int addr, unsigned short port, int ttl)
{
  int fd;
  struct sockaddr_in sin;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    perror("socket");
    exit(1);
  }
  //  nonblock(fd);

#ifdef WIN32
  memset((char *)&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = 0;
  sin.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
    perror("bind");
    exit(1);
  }
#endif
  memset((char *)&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = htons(port);
  sin.sin_addr.s_addr = addr;
  if (connect(fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
    perror("connect");
    exit(1);
  }
  if (IN_CLASSD(ntohl(addr))) {
#ifdef IP_ADD_MEMBERSHIP
    char c;

    /* turn off loopback */
    // c = 0;
    // if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &c, 1) < 0) {
      /*
       * If we cannot turn off loopback (Like on the
       * Microsoft TCP/IP stack), then declare this
       * option broken so that our packets can be
       * filtered on the recv path.
       */
      //if (c == 0)
      //	noloopback_broken_ = 1;
    // }
    /* set the multicast TTL */
#ifdef WIN32
    u_int t;
#else
    u_char t;
#endif
    t = (ttl > 255) ? 255 : (ttl < 0) ? 0 : ttl;
    if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL,
		   (char*)&t, sizeof(t)) < 0) {
      perror("IP_MULTICAST_TTL");
      exit(1);
    }
#else
    fprintf(stderr, "\
not compiled with support for IP multicast\n\
you must specify a unicast destination\n");
    exit(1);
#endif
  }
  /*
   * XXX don't need this for the session socket.
   */
  int bufsize = 80 * 1024;
  if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, (char *)&bufsize,
		 sizeof(bufsize)) < 0) {
    bufsize = 48 * 1024;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, (char *)&bufsize,
		   sizeof(bufsize)) < 0)
      perror("SO_SNDBUF");
  }
  return (fd);
}

int
t_RTPChannel::rtp_open_my_sockets(char* multicast_address, short rtp_port, char ttl)
{
  rtp_me.rtp_sock_out = openssock(inet_addr(multicast_address),
				  rtp_port,
				  ttl);
  if(!rtp_me.rtp_sock_out) return 0;
  int len = sizeof(rtp_sin);
  if (getsockname(rtp_me.rtp_sock_out, (struct sockaddr *)&rtp_sin, &len) < 0) {
    perror("getsockname");
    rtp_sin.sin_addr.s_addr = 0;
    rtp_sin.sin_port = 0;
  }
  rtp_me.rtp_sock_in = openrsock(inet_addr(multicast_address),
				 rtp_port,
				 rtp_sin);
  if(!rtp_me.rtp_sock_in) return 0;  
  // same for RTCP
  
  rtp_me.rtcp_sock_out = openssock(inet_addr(multicast_address),
				  rtp_port+1,
				  ttl);
  if(!rtp_me.rtcp_sock_out) return 0;

  rtcp_sin.sin_addr.s_addr = 0;
  rtcp_sin.sin_port = 0;
  if (getsockname(rtp_me.rtcp_sock_out, (struct sockaddr *)&rtcp_sin, &len) < 0) {
    perror("getsockname");
  }
  rtp_me.rtcp_sock_in = openrsock(inet_addr(multicast_address),
				 rtp_port+1,
				 rtcp_sin);
  if(!rtp_me.rtcp_sock_in) return 0;
  return 1;
}

/*************** API functions !!!! *************************************/

int
t_RTPChannel::receive_rtp_packet(char *packet, int* len)
{
    int sin_len,seq;
    struct sockaddr_in from_sin;
    sin_len = sizeof(from_sin);
    *len = recvfrom(rtp_me.rtp_sock_in,
		   rtp_packet_buffer,
		   sizeof(rtp_packet_buffer),
		   0,
		   (struct sockaddr*)&from_sin, &sin_len);
    // I will ignore my own pakets
    if(from_sin.sin_addr.s_addr == rtp_sin.sin_addr.s_addr &&
       from_sin.sin_port == rtp_sin.sin_port)
      return 0;
    rtp_octets_received_cnt += *len;
    if( rtp_handle_rtp_packet(*len) ) {
      *len -= sizeof(rtp_hdr_t); 
      memcpy(packet,
	     rtp_packet_buffer+sizeof(rtp_hdr_t),
	     *len);
      return 1;
    }
    else
      return 0;
}

int
t_RTPChannel::receive_rtp_packet(char *packet, int* len, ulong* senderId)
{
  if(receive_rtp_packet(packet, len)) {
    *senderId = ((rtp_hdr_t*)rtp_packet_buffer)->ssrc;
    return 1;
  }
  else
    return 0;
}


void
t_RTPChannel::send_rtp_packet(char *data, int len)
{
  int i;
  rtp_seq_number++;
  rtp_rtp((rtp_hdr_t*)rtp_packet_buffer);
  memcpy(rtp_packet_buffer+sizeof(rtp_hdr_t), data, len);
/*  test_print_rtp_packet((rtp_hdr_t*)rtp_packet_buffer); */
  i = send(rtp_me.rtp_sock_out,
       rtp_packet_buffer,
       len+sizeof(rtp_hdr_t),
       0);
  //  fprintf(stderr,"Bytes sent: %d\n", i); 
  rtp_octet_cnt += len+sizeof(rtp_hdr_t);
  rtp_me.is_sender = 1; 
}


int
t_RTPChannel::send_rtcp_packet()
{
    int i,k;
    int next;
    double interval;
    rtcp_t* rtcp_p = (rtcp_t*) rtp_packet_buffer;
    char *p_start = rtp_packet_buffer;
    if(rtp_me.is_sender) {
      k = rtp_rtcp_sr((rtcp_t*) p_start);
      p_start += k;
    }
    else { // if( participant_cnt > 0) {
      k = rtp_rtcp_rr((rtcp_t*) p_start);
      p_start += k;
    }
    k = rtp_rtcp_sdes((rtcp_t*) p_start, rtp_me.name, rtp_me.cname, rtp_me.note);
    p_start += k;

    send(rtp_me.rtcp_sock_out,
	 rtp_packet_buffer,
	 (int)p_start-(int)rtp_packet_buffer,
	 0);

    rtp_octet_cnt += (int)p_start-(int)rtp_packet_buffer; 
    interval = rtcp_interval(participant_cnt+1,
			     rtp_sender_cnt(),
			     rtcp_bandwidth,
			     rtp_me.is_sender,
			     (int)p_start-(int)rtp_packet_buffer,
			     &rtcp_avg_packet_size,
			     rtcp_initial_report);
    if(interval > 10.0) interval = 10.0;
    rtcp_initial_report = 0;
    return (int)(1000000*interval);
}

unsigned int
t_RTPChannel::octets_send()
{
    return rtp_octet_cnt;
}

unsigned int
t_RTPChannel::octets_received()
{
    return rtp_octets_received_cnt;
}

int
t_RTPChannel::receive_rtcp_packet()
{
    int len;
    int sin_len;
    struct sockaddr_in from_sin;
    sin_len = sizeof(from_sin);
    len = recvfrom(rtp_me.rtcp_sock_in,
		   rtp_packet_buffer,
		   sizeof(rtp_packet_buffer),
		   0,
		   (struct sockaddr*)&from_sin, &sin_len);
    // I will ignore my own pakets
    if(from_sin.sin_addr.s_addr == rtcp_sin.sin_addr.s_addr &&
       from_sin.sin_port == rtcp_sin.sin_port)
      return 0;
    rtp_octets_received_cnt += len;
    return rtp_handle_rtcp_packet(len);			
}

char*
t_RTPChannel::sdes_item(SDESItems sdesItem)
{
  switch(sdesItem) {
  case SDES_CNAME:
    return participant[rtp_last_sender].cname;
  case SDES_NAME:
    return participant[rtp_last_sender].name;
  case SDES_EMAIL:
    return participant[rtp_last_sender].email;
  case SDES_PHONE:
    return participant[rtp_last_sender].phone;
  case SDES_LOCATION:
    return participant[rtp_last_sender].location;
  case SDES_TOOL:
    return participant[rtp_last_sender].tool;
  case SDES_NOTE:
    return participant[rtp_last_sender].note;
  case SDES_PRIV:                    
    return participant[rtp_last_sender].priv;
  default:
    return NULL;
  }
}

char*
t_RTPChannel::sdes_item(SDESItems sdesItem, unsigned long ssrc)
{
  int i;
  for( i = 0; i < participant_cnt; i++) {
    
    if(participant[i].ssrc == ssrc)
      break;
  }
  if(i >= participant_cnt) return NULL;
  switch(sdesItem) {
  case SDES_CNAME:
    return participant[i].cname;
  case SDES_NAME:
    return participant[i].name;
  case SDES_EMAIL:
    return participant[i].email;
  case SDES_PHONE:
    return participant[i].phone;
  case SDES_LOCATION:
    return participant[i].location;
  case SDES_TOOL:
    return participant[i].tool;
  case SDES_NOTE:
    return participant[i].note;
  case SDES_PRIV:                    
    return participant[i].priv;
  default:
    return NULL;
  }
}

u_int32
t_RTPChannel::mySSRC()
{
  return rtp_me.ssrc;
}

int
t_RTPChannel::rtp_socket()
{
  return rtp_me.rtp_sock_in;
}

int
t_RTPChannel::rtcp_socket()
{
  return rtp_me.rtcp_sock_in;
}


t_RTPChannel::t_RTPChannel(char *name,
	       char *email,
	       char *phone,
	       char *location,
	       char *tool,
	       char *note,
	       char *priv,
	       char *multicast_address,
	       unsigned short port,
	       char ttl)
{
  participant_cnt = 0;
  rtp_octet_cnt = 0;
  rtp_octets_received_cnt = 0;
  rtcp_avg_packet_size = 0;
  rtcp_initial_report = 1;
  rtcp_avg_packet_size = 0;
  rtcp_initial_report = 1;
  rtp_me.ssrc = random32(137);
  rtp_me.is_sender = 0;
  if(name) rtp_me.name = name;
  else rtp_me.name = "";
  if(email) rtp_me.email = email;
  else rtp_me.email = "";
  if(phone) rtp_me.phone = phone;
  else rtp_me.phone = "";
  if(location) rtp_me.location = location;
  else rtp_me.location = "";
  if(tool) rtp_me.tool = tool;
  else rtp_me.tool = "";
  if(note) rtp_me.note = note;
  else rtp_me.note = "";
  if(priv) rtp_me.priv = priv;
  else rtp_me.priv = "";

  rtp_me.cname = rtp_cname();

  if(!rtp_open_my_sockets(multicast_address, port, ttl)) {
    fprintf(stderr, "Could not open sockets\n");
    exit(0);
  }
}

/*
 * we open the server as participant 0, SSRC will be automatically
 * set to the correct value when the servers first RTCP SDES packet
 * is received
 */


float
t_RTPChannel::fraction_lost()
{
  u_int32 lost;
  float fraction = 0.0;
  int i;
  for(i = 0; i < participant_cnt; i++) {
    fraction += (float) rtp_calculate_loss_ro(&participant[i].src, &lost) / 256.0;
  }
  return fraction / participant_cnt;
}

int
t_RTPChannel::lost_packages()
{
  u_int32 lost;
  u_int32 total = 0;
  u_int8 fraction;
  int i;
  for(i = 0; i < participant_cnt; i++) {
    fraction = rtp_calculate_loss_ro(&participant[i].src, &lost);
    total += lost;
  }
  return total;
}

int
t_RTPChannel::session_members()
{
  return participant_cnt+1;
}


int
t_RTPChannel::check_for_input()
{
  fd_set fds;
  int retval = 0;
  struct timeval timeout; // = (struct timeval) {0,0};
  timeout.tv_sec=0;
  timeout.tv_usec=0;
  FD_ZERO(&fds);
  FD_SET(rtp_me.rtp_sock_in,&fds);
  FD_SET(rtp_me.rtcp_sock_in,&fds);
  if(select(FD_SETSIZE, &fds, NULL, NULL, &timeout) == 0)
    return 0;
  if(FD_ISSET(rtp_me.rtp_sock_in, &fds)) retval |= RTP_INPUT;
  if(FD_ISSET(rtp_me.rtcp_sock_in, &fds)) retval |= RTCP_INPUT;   
  return retval;
}

/* --------------------------------- functions for debugging ---------------------- */ 

void
t_RTPChannel::test_print_rtcp_packet(rtcp_common_t* p)
{
    printf("participants: %d\n",participant_cnt);
    printf("new RTCP packet:\n");
    printf("bitarray= %x\n", *(u_int16*)p);
    printf("version = %d\n",(u_int32)p->version);
    printf("p       = %d\n",(u_int32)p->p);
    printf("pt      = %d\n",(u_int32)p->pt);
    printf("count   = %d\n",(u_int32)p->count);    
    printf("length  = %d\n",(u_int32)p->length);
    printf("\n");
}

void
t_RTPChannel::test_print_rtp_packet(rtp_hdr_t* p)
{
    printf("new RTP packet:\n");
    printf("version = %d\n",(u_int32)p->version);
    printf("p       = %d\n",(u_int32)p->p);
    printf("x       = %d\n",(u_int32)p->x);
    printf("cc      = %d\n",(u_int32)p->cc);
    printf("m       = %d\n",(u_int32)p->m);
    printf("pt      = %d\n",(u_int32)p->pt);
    printf("seq     = %d\n",(u_int32)p->seq);
    printf("ts      = %08x\n",(u_int32)p->ts);
    printf("ssrc    = %08x\n",(u_int32)p->ssrc);
    printf("csrc    = %08x\n",(u_int32)p->csrc[0]);                    
    printf("\n");
}

void
t_RTPChannel::test_print_participants_cname()
{
  int i;
  fprintf(stderr, "current participants:\n");
  for(i = 0; i < participant_cnt; i++)
    if(participant[i].cname) {
      fprintf(stderr, "%s\n",
	      participant[i].cname);
    }
}
