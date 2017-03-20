#!/usr/bin/perl

# Send the data to zaza2
sub sendit {
   my ($sendtext) = @_;
   use strict;
   use Socket;
   my ($remote, $port, $iaddr, $paddr, $proto, $line);
   $remote  = 'zaza1.exhibits.thetech.org';
   $port    = 80;  # speech port
   if ($port =~ /\D/) { $port = getservbyname($port, 'tcp') }
   die "No port" unless $port;
   $iaddr   = inet_aton($remote) || die "no host: $remote";
   $paddr   = sockaddr_in($port, $iaddr);
   $proto   = getprotobyname('tcp');
   socket(SOCK, PF_INET, SOCK_STREAM, $proto)  || die "socket: $!";
   connect(SOCK, $paddr)    || die "connect: $!";
   print "Sent data: $sendtext";
   print SOCK "$sendtext\r\n";
   sleep 1;
   close (SOCK)  || die "close: $!";
}

&sendit("GET /cgi-bin/zaza-control?op_mode=ShutDown HTTP/1.0\r\n");
#&sendit("GET /cgi-bin/zaza-control?op_mode=Phase1 HTTP/1.0\r\n");

