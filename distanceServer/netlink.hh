#ifndef _NETLINK_HH
#define _NETLINK_HH

// pure obstacles will have a network interface !

#include <sys/types.h>
#include "simrtp.hh"
#include "typeInfo.hh"

class t_NetLink :
  public t_RTPChannel
{
public:
  t_NetLink(char* name, char *email, char *phone, char *location, char *tool,
	    char *note, char *priv, char *multicast_address, u_short port, char ttl);

  /* transmit create or update msg of an obstacle to the
     interfaces RTPChannel */
  u_short write(t_AttribArray attrs);

  /* receive some attributes from the RTP input buffer */
  u_short read(t_AttribArray attrs);

  void readPacketHeader(char *cmd, char *spec, u_short *len);

  char* writePacketHeader(char cmd, char spec);

  void netWrite();
  int netRead(int *len, u_long *ssrc);
protected:
  static char netBuffer[];
  static char *cNetBufPos;
  void reset();
};

#endif
