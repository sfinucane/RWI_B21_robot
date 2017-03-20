
#include <stdio.h>
#include <strings.h>
#include "netlink.hh"

char
t_NetLink::netBuffer[4096];
char*
t_NetLink::cNetBufPos;

u_short
t_NetLink::write(t_AttribArray attrs)
{
  int i = 0;
  t_AttribArray traverse = attrs;
  while(traverse->type != t_Attrib::TI_END) {
    t_Attrib attr(traverse);

#ifdef DEBUG
    fprintf(stderr, "%s: ", traverse->name);
    switch(traverse->type) {
    case t_Attrib::TI_FLOAT:
      fprintf(stderr, "%f\n", *((float*)traverse->value));
      break;
    case t_Attrib::TI_LONG:
      fprintf(stderr, "%d\n", *((long*)traverse->value));
      break;
    case t_Attrib::TI_CHAR:
      fprintf(stderr, "%d\n", (int) *((char*)traverse->value));
      break;
    case t_Attrib::TI_SHORT:
      fprintf(stderr, "%d\n", (int) *((short*)traverse->value));
      break;
    case t_Attrib::TI_STRING:
      fprintf(stderr, "%s\n", *((char**)traverse->value));
      break;
    default:
      fprintf(stderr, "?\n");
    }
#endif
    attr.transmit(&cNetBufPos);
    traverse++;
  }
  return htons((u_short) (cNetBufPos - netBuffer));
}

u_short
t_NetLink::read(t_AttribArray attrs)
{
  int i = 0;
  t_AttribArray traverse = attrs;
  while(traverse->type != t_Attrib::TI_END) {
    t_Attrib attr(traverse);
    attr.receive(&cNetBufPos);

#ifdef DEBUG
    fprintf(stderr, "%s: ", traverse->name);
    switch(traverse->type) {
    case t_Attrib::TI_FLOAT:
      fprintf(stderr, "%f\n", *((float*)traverse->value));
      break;
    case t_Attrib::TI_LONG:
      fprintf(stderr, "%d\n", *((long*)traverse->value));
      break;
    case t_Attrib::TI_CHAR:
      fprintf(stderr, "%d\n", (int) *((char*)traverse->value));
      break;
    case t_Attrib::TI_SHORT:
      fprintf(stderr, "%d\n", (int) *((short*)traverse->value));
      break;
    case t_Attrib::TI_STRING:
      fprintf(stderr, "%s\n", *((char**)traverse->value));
      break;
    default:
      fprintf(stderr, "?\n");
    }
#endif
    traverse++;
  }
  return (u_short) (cNetBufPos - netBuffer);
}

void
t_NetLink::readPacketHeader(char *cmd, char *spec, u_short *len)
{
  reset();
  *cmd = *cNetBufPos++;
  *spec = *cNetBufPos++;
  bcopy(cNetBufPos, len, sizeof(u_short));
  cNetBufPos += sizeof(u_short);
  *len = ntohs(*len);
}

char *
t_NetLink::writePacketHeader(char cmd, char spec)
{
  reset();
  *cNetBufPos++ = cmd;
  *cNetBufPos++ = spec;
  char *c = cNetBufPos;
  cNetBufPos += sizeof(u_short);
  return c;
}

void
t_NetLink::netWrite()
{
  send_rtp_packet(netBuffer, (int) (cNetBufPos-netBuffer));
  reset();
}

void
t_NetLink::reset()
{
  cNetBufPos = netBuffer;
}

int
t_NetLink::netRead(int *len, u_long *ssrc)
{
  reset();
  return receive_rtp_packet(netBuffer, len, ssrc);
}

t_NetLink::t_NetLink(char* name, char *email, char *phone,
		     char *location, char *tool,
		     char *note, char *priv,
		     char *multicast_address, u_short port, char ttl)
    : t_RTPChannel(name, email, phone, location, tool, note, priv,
		   multicast_address, port, ttl)
{
  reset();
}
