
#include <stdlib.h>
#include <strings.h>
#include <netinet/in.h>
#include <stdio.h>

#include "typeInfo.hh"

void
t_Attrib::receive(char **buf)
{
  unsigned char len;
  char *string;
  
  switch(tp->type) {
  case TI_CHAR:
    *((char*)tp->value) = **buf;
    *buf += sizeof(char);
    break;
  case TI_SHORT:
    bcopy(*buf, tp->value, sizeof(short));
    *buf += sizeof(short);
    *((short*)tp->value) = ntohs(*((short*)tp->value));
    break;
  case TI_LONG:
  case TI_FLOAT:
    bcopy(*buf, tp->value, sizeof(long));
    *buf += sizeof(long);
    *((long*)tp->value) = ntohl(*((long*)tp->value));
    break;
  case TI_DOUBLE:
    bcopy(*buf, tp->value, sizeof(double));    
    *buf += sizeof(double);
#if (defined(_LITTLE_ENDIAN) && !defined(sun)) 
    long *l = (long*) tp->value;
    long t = l[0];
    l[0] = ntohl(l[1]);
    l[1] = ntohl(t);
#endif
    break;
  case TI_STRING:
    len = **buf;
    *buf += sizeof(char);
    string = new char[len+1];
    bcopy(*buf, string,len);
    *buf += len;
    string[len] = '\0';
    *((char**)tp->value) = string; 
    break;
  default:
    fprintf(stderr, "typeInfo::receive: Found Unknown Type\n");
    exit(-1);
  }
}

void
t_Attrib::transmit(char **buf)
{
  unsigned char len;
  switch(tp->type) {
  case TI_CHAR:
    **buf = *((char*)tp->value);
    *buf += sizeof(char);
    break;
  case TI_SHORT: {
    short sval = htons(*((short*)tp->value));
    bcopy(&sval, *buf, sizeof(short));
    *buf += sizeof(short);
    break;
  }
  case TI_LONG:
  case TI_FLOAT: {
    long lval = htonl(*((long*)tp->value));
    bcopy(&lval, *buf, sizeof(long));
    *buf += sizeof(long);
    break;
  }
  case TI_DOUBLE: {
    double dval = *((double*)tp->value);
#ifdef _LITTLE_ENDIAN
    long *l = (long*) &dval;
    long t = l[0];
    l[0] = htonl(l[1]);
    l[1] = htonl(t);
#endif
    bcopy(&dval, *buf, sizeof(double));    
    *buf += sizeof(double);
    break;
  }
  case TI_STRING:
    len = strlen(*((char**)tp->value));
    **buf = len;
    *buf += sizeof(char);
    bcopy(*((char**)tp->value), *buf, len);
    *buf += len;
    break;
  default:
    fprintf(stderr, "typeInfo::transmit: Found Unknown Type\n");
    exit(-1);
  }
}
