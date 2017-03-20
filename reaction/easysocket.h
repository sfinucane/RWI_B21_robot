/* Dan Furlani, May 2000
   
   With enhancements by Brian Rudy, May 2001

   These are ``wrapper'' functions for the basic UNIX Socket functions,
   written with the goal of making socket programming less painful in C.
   They may bear some resemblence to the Java socket model, since I
   wrote them while trying to translate some Java socket code into C.

   Two very useful references I used are:
     GNU C Library Documentation:
       http://www.gnu.org/manual/glibc-2.0.6/
     Michael Lemmon's Course pages on Advanced Unix Programming:
       http://www.nd.edu/~lemmon/courses/UNIX/
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>


#define DEBUG FALSE        /* display debugging info */

#ifndef TRUE
  #define TRUE 1
#endif
#ifndef FALSE
  #define FALSE 0
#endif

/* use of this macro requires care -- it suffers from the dangling
 * else problem (I think) but, it is an easy way to throw debugging
 * info into code, and if DEBUG is set to false, an optimizing 
 * compiler will (I think) recognize the statement as if(0) and never
 * include it in compiled code.    
 */
#define __say if (DEBUG) printf


/* This enum is used to distinguish the two versions of make_server_socket
 * and make_client_connection.  The implementations of these functions
 * depends on whether TCP or UDP "connections" (obviously UDP is
 * connectionless) are desired.
 */
enum protocol {TCP, UDP};


/* Sequence number information data structure so reliable functions can
 * keep track of sequencing.
 */
typedef struct sequence_data {
  int seq;
  int dup;
  int lost;
  int timeout;
  int (*nextSeqNum)(int);  /* pointer to a function taking int to int */
} sequence_data;


/* a sample function for nextSeqNum:
 *     int nextSequenceNumber(int sn) { return sn + 1; }
 * this produces nice debugging output, but something as simple as
 * switching between 1 and 0 is sufficient.  it is *very* important
 * that 2 hosts use the same function to communicate, or it won't work!
 */


/* Prepare a sockaddr_in structure (holds the IP and port) to use when
 * opening a socket.  This function is used by make_client_connection.
 */
void init_sockaddr( struct sockaddr_in *name,
                    const char *hostname,
                    unsigned short int port ) {
  struct hostent *hostinfo;
  name->sin_family = AF_INET;
  name->sin_port = htons(port);
  hostinfo = gethostbyname(hostname);
  if (hostinfo == NULL) {
    fprintf(stderr, "Unknown host %s.\n", hostname);
    exit (EXIT_FAILURE);
  }
  name->sin_addr = *(struct in_addr *) hostinfo->h_addr;
}

/* Create a server socket on the local host with specified port */
int make_server_socket( unsigned short int port, enum protocol pr ) {
  int sock;
  int sock_type;
  struct sockaddr_in name;

  if      (pr == TCP) sock_type = SOCK_STREAM;
  else if (pr == UDP) sock_type = SOCK_DGRAM;
  else {
    fprintf(stderr, "Bad protocol argument to make_client_connection");
    exit(EXIT_FAILURE);
  }
  if ((sock = socket(PF_INET, sock_type, 0)) < 0) {
    perror("socket (server)");
    exit(EXIT_FAILURE);
  }
  name.sin_family = AF_INET;
  name.sin_port = htons(port);
  name.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind( sock, (struct sockaddr *) &name, sizeof(name) ) < 0) {
    perror("bind (server)");
    exit(EXIT_FAILURE);
  }
  if (pr == TCP)                 /* only TCP uses listen! */
    if (listen (sock, 5) < 0) {
      perror("listen (server)");
      exit(EXIT_FAILURE);
    }
  return sock;
}

/* Create a client socket connection to the given hostname & port. */
int make_client_connection( const char *hostname,
                            unsigned short int port,
                            enum protocol pr ) {
  int sock;
  int sock_type;
  struct sockaddr_in server, me;
  int retry = 0;

  if      (pr == TCP) sock_type = SOCK_STREAM;
  else if (pr == UDP) sock_type = SOCK_DGRAM;
  else {
    fprintf(stderr, "Bad protocol argument to make_client_connection");
    exit(EXIT_FAILURE);
  }

  if ((sock = socket(PF_INET, sock_type, 0)) < 0) {
    perror("socket (client)");
    exit(EXIT_FAILURE);
  }

  /* UDP binds a port locally so it can receive ack's from the server */
  /* it's sending to.  Thus it's almost like 2-way communication.     */
  if (pr == UDP) {
    me.sin_family = AF_INET;
    me.sin_port = htons(0);
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind( sock, (struct sockaddr *) &me, sizeof(me) ) < 0) {
      perror("bind (client)");
      exit(EXIT_FAILURE);
    }
  }
  /* connect to server (for UDP, this creates the default connection) */
  init_sockaddr(&server, hostname, port);
  
    if (connect(sock, (struct sockaddr *) &server, sizeof(server)) < 0) {
       perror("connect (client) failed, retrying...");    
       /* exit(EXIT_FAILURE); */
       sleep(2);
       /* This is recursion, be carefull */
       make_client_connection(hostname, port, pr);
     } 
  return sock;
}

/* Close the sucker down -BR */
void close_connection(int sock) {
  int shtdwn_response;
  if (sock >= 0) {
    shtdwn_response = shutdown(sock, 2); 
    /* 0 Stop receiving data for this socket. If further data arrives, 
         reject it. 
       1 Stop trying to transmit data from this socket. Discard any data 
         waiting to be sent. Stop looking for acknowledgement of data 
         already sent; don't retransmit it if it is lost. 
       2 Stop both reception and transmission. 
     */
    if (shtdwn_response < 0) {
       perror("Socket shutdown failed!");
    }    
  }
  else {
     perror("Socket invalid, cannot close.");
  }
}



/* Block until a connection is made to this socket.  TCP only! */
int wait_for_connection(int sock) {
  /* Assume the server doesn't care who connects.  Otherwise clientname    */
  /* needs to be defined by the calling process & passed to this function. */
  int new_sock;

  if ((new_sock = accept(sock, NULL, NULL)) < 0) {
    perror("accept");
    exit(EXIT_FAILURE);
  }
  return new_sock;
}

/************************* TCP send & receive **************************/

/* Write `length' bytes from `buf' into `sock', return # bytes written.
 * TCP only!                                                           
 */
int write_to_socket(int sock, char *buf, int length) {
  int bytes_out, bytes_left;

  for (bytes_left = length;
       bytes_left > 0 && (bytes_out = write(sock, buf, bytes_left)) > 0;
       bytes_left -= bytes_out, buf += bytes_out)
    __say("%d + ", bytes_out);

  if (bytes_out < 0) {         /* error from write()! */
    perror("write");
    exit(EXIT_FAILURE);
  }
  if (bytes_out == 0) {        /* this will not happen if all data was sent. */
    __say("EOF on write\n");   /* it must be for some other (bad) reason.    */
    return 0;                  /* so return 0 as an error code               */
  }

  __say(" = %d/%d bytes written\n", length - bytes_left, length);

  return (length - bytes_left);  /* = number of bytes sent */
}

/* Read a maximum of `length' bytes into `buf' from `sock'.
 * Return actual number of bytes read, or -1 for EOF.  TCP only!
 */
int read_from_socket(int sock, char *buf, int length) {
  int bytes_in, bytes_left;

  for (bytes_left = length;
       bytes_left > 0 && (bytes_in = read(sock, buf, bytes_left)) > 0;
       bytes_left -= bytes_in, buf += bytes_in)
    __say("%d + ", bytes_in);

  if (bytes_in < 0) {
    perror("read");
    exit(EXIT_FAILURE);
  }
  if (bytes_in == 0) __say("EOF on read\n");

  __say(" = %d/%d bytes read\n", length - bytes_left, length);

  return (length - bytes_left); /* = total bytes read */
}

/************************* UDP send & receive **************************/

/* Write datagram out on sock.  UDP only! */
int write_datagram(int sock, char *buf, int length) {
  return write(sock, buf, length);
}

/* Block until a datagram is sent to sock.  After the datagram is
 * received, connect sock to the sender so that an Ack may be sent.
 * This establishes (more or less) a 2-way communications channel.
 * UDP only!
 */
int read_datagram(int sock, char *buf, int length) {
  struct sockaddr_in cli;
  int cli_size;
  int retval;

  cli_size = sizeof(cli);
  retval = recvfrom(sock, buf, length, 0,(struct sockaddr *) &cli, &cli_size);
  if (connect(sock, (struct sockaddr *) &cli, cli_size) < 0) {
    perror("connect (read)");
    exit(EXIT_FAILURE);
  }
  return retval;
}

/********************* Reliable UDP send & receive *********************/

/* Send len bytes of data reliably to the socket previously established
 * using make_client_connection.  sd is a structure containing the info
 * needed for this function to maintain accurate sequencing data.
 */
void reliableSend(int sock, char *data, int len, sequence_data *sd) {
  int sendFailed, gotWrongAck;         /* boolean */
  int ready;
  fd_set r_watch;
  struct timeval tv;
  char ack;

  data[len] = (char) sd->seq;          /* add seq # to data stream */

  /* Try sending until we get the right ACK back. */
  do { /* SEND datagram */
    write_datagram(sock, data, len + 1);
    __say("sent%d ", sd->seq);

    do { /* WAIT for ack */
      /* set timeout info */
      FD_ZERO(&r_watch);               /* clear list of read file desc's */
      FD_SET(sock, &r_watch);          /*   but watch for this one       */
      tv.tv_sec = 0;                   /* set the timer to    */
      tv.tv_usec = sd->timeout;        /*   timeout microsecs */

      /* block until something comes in, or we time out */
      ready = select(FD_SETSIZE, &r_watch, NULL, NULL, &tv);
      if (ready < 0) {                 /* error with select */
        perror ("select");
        exit (EXIT_FAILURE);
      }
      else if (ready) {                /* we got something! */
        /* check that sock is the descriptor that was ready.  this is */
        /* pointless, since it's the only descriptor in the group,    */
        /* but it's probably a Good Idea.                             */
        if (FD_ISSET(sock, &r_watch)) {
          read_datagram(sock, &ack, 1);
          __say("gotAck%d ", (int) ack);
          if (ack != (char) sd->seq) gotWrongAck = TRUE;
          else {                       /* got correct ack     */
            gotWrongAck = FALSE;       /*   so make sure we   */
            sendFailed = FALSE;        /*   drop out of loops */
            sd->seq = (* sd->nextSeqNum)(sd->seq);  /* update seq num */
            if (sd->timeout > 1000) sd->timeout -= 1000;
          }
        }
        else {       /* not our file descriptor? how'd that happen? */
          __say("\nThis should never happen\n");
          /* this isn't a perfect fix, since our timer gets reset */
          gotWrongAck = TRUE;
        }
      }
      else {                           /* oops, we timed out */
        __say("(TIMEOUT=%d)\n", sd->timeout);
        gotWrongAck = FALSE;           /* we didn't get the WRONG ack, */
                                       /*   we got NO ack.  So we need */
        sendFailed = TRUE;             /*   to break out of some loops */
        sd->lost++;                    /* chalk up a lost packet */
        sd->timeout *= 2;              /* make timeout longer */
      }
    }
    while (gotWrongAck);
  }
  while (sendFailed);
  __say("\n");

} /* end of reliableSend() */


/* Read a maximum of len bytes of data reliably from the socket previously
 * established using make_server_socket.  sd is a structure containing the
 * info needed for this function to maintain accurate sequencing data.
 */
int reliableRecv(int sock, char *data, int len, sequence_data *sd) {
  int recvFailed;
  int bytes_read;
  char ack;

  do {
    /* READ datagram */
    __say("wait%d ", sd->seq);
    bytes_read = read_datagram(sock, data, len);
    __say("recd ");

    /* ACK it */
    ack = data[bytes_read - 1];
    write_datagram(sock, &ack, 1);
    __say("ack%d ", (int) ack);

    recvFailed = FALSE;
    if (sd->seq == -1) {               /* first recv? then we don't */
      sd->seq = (int) ack;             /*   know the seq # yet, so  */
    }                                  /*   take their word for it. */
    if (ack != (char) sd->seq) {
      recvFailed = TRUE;
      sd->dup++;
      __say("(WRONG PACKET)\n");
    }

  } /* do */
  while (recvFailed);
  sd->seq = (* sd->nextSeqNum)(sd->seq);  /* update the sequence number */
  __say("\n");
  return bytes_read - 1;

} /* end of reliableRecv() */

/***********************************************************************/
