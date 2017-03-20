#!/usr/bin/perl -w
#

# Ensures proper ACCESS.bus functionality by resetting the bus if it is 
# hung before starting up the robots servers. This script removes the 
# need to hit the A.bus reset switch when powering up the robot.



$port = "/dev/ttyR4"; # Port 4 on the Rocketport 8j card
open(WATCHDOG,"+<$port") || die "opening $port for input: $!";

#
# This is the normal config from 'stty -F /dev/ttyS2 -a':
# 
# speed 9600 baud; rows 0; columns 0; line = 0;
# intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^D; eol = <undef>;
# eol2 = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R; werase = ^W;
# lnext = ^V; flush = ^O; min = 1; time = 0;
# -parenb -parodd cs8 hupcl -cstopb cread clocal -crtscts
# -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr icrnl ixon 
# -ixoff -iuclc -ixany -imaxbel
# opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 
# ff0 isig icanon iexten echo echoe echok -echonl -noflsh -xcase -tostop 
# -echoprt echoctl echoke

system("stty 9600 clocal cread -crtscts -echo -parodd -parenb cs8 -cstopb raw < $port");
#system("stty 9600 -echo -parodd -parenb cs8 -cstopb raw -F $port");


# Flush the output buffer, and reset the stamp
#print "Flushing input\n";
sendMsg("i");

#print "Waiting for reply\n";
my $reply = getMsg();    	      # Get the command help list
getReady(); 		 	      # Get the Ready prompt

my $health = checkHeartBeat();
while ($health eq "Hung") {
  resetBus();  # reset the bus
  sleep 5;     # sleep for 5 seconds and let everything settle 
  $health = checkHeartBeat(); # Check the SCL heartbeat    
}

print "Bus heartbeat ok. Interval time = $health\n";
close(WATCHDOG);
#print "Exiting\n";


# Reset the bus (MSPs and MCPs)
sub resetBus {
  print "Resetting the bus\n";
  sendMsg("r");
  #sleep 4; 		# Wait for the bus to reset
  my $reply = getMsg(3); # Wait for the bus to reset (3 sec)
  if (!($reply =~ m/reset/)) {
    print "Bus reset failed: $reply";
  }
  getReady(); # get the ready prompt
}


sub checkHeartBeat {
  # Check BOTH the clock and data lines for activity

  sendMsg("c");  # request the SCL line status
  my $clock_reply = getMsg();
  getReady();      # get the ready prompt

  sendMsg("d");  # request the SDA line status
  my $data_reply = getMsg();
  getReady();      # get the ready prompt

  if (($clock_reply =~ m/Hung\!/) || ($data_reply =~ m/Hung\!/)) {
    print "ACCESS.bus is hung!\n";
    return "Hung";
  }
  elsif ($clock_reply =~ m/=(\d+)/) {
    return $1;
  }
  else {
    print "Bus heath request failure: $reply";
  }
}


# Get the Watchdog ready: prompt
sub getReady {
  my $reply = getMsg();
  #while (!($reply =~ m/ready:/)) {
  if (!($reply =~ m/ready:/)) {
    print "Oops, we got:" . $reply . " not the Watchdog ready: prompt!\n";
    #$reply = getMsg();
  }
}


# Read characters on the serial port until we get a CR/LF
# If specified, the given timeout value is used to wait for a response
sub getMsg {
  my $timeout = shift || 0.3;  # set it to something sensible if not specified
  #print "Timeout = $timeout\n";
  my $inbuff = "";
  my $char;
  my $rin = "";
  vec($rin,fileno(WATCHDOG),1) = 1;
  while ( (select($rout=$rin, undef, undef, $timeout)) && (!($inbuff =~ m/.+\n/))) {
    $char = '';
    sysread WATCHDOG,$char,1;
    #print "Got:\'" . $char . "\'\n";
    $inbuff .= $char;
  }
  if ($inbuff eq "") {
    print "Error! Timed out waiting for input.\n";
  }
  else {
    #print "Got line:" . $inbuff . "";
  }
  return $inbuff;
}


# send some characters.
sub sendMsg {
  my ($msg) = @_;
  syswrite WATCHDOG, $msg, length($msg);
}
