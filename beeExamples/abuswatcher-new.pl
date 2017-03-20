#!/usr/bin/perl -w
#

# abuswatcher-new.pl
#
# Ensures proper ACCESS.bus functionality by monitoring the bus and 
# baseServer output during normal operation. If a fault is discovered,
# The bus will be reset (if needed) and the robot services restarted.

# 0.02 6-18-2004
# Minor bugfix for MSP fault reporting message.
#
# 0.01 6-11-2004
# First working version. Only supports Phase 2 mode auto-restart.




$port = "/dev/ttyR4"; # Port 4 on the Rocketport 8j card
open(WATCHDOG,"+<$port") || die "opening $port for input: $!";

system("stty 9600 clocal cread -crtscts -echo -parodd -parenb cs8 -cstopb raw < $port");


# Flush the output buffer, and reset the stamp
#print "Flushing input\n";
sendMsg("i");
my $reply = getMsg();    	      # Get the command help list
getReady(); 		 	      # Get the Ready prompt


# main loop
while (1) {
  # Check if baseServer is running
  open BSSTAT, "/sbin/pidof baseServer |";
  my $baseServer_runnning = 0;
  while (<BSSTAT>) {
    if ($_ =~ m/\d+/) {
      $baseServer_runnning = 1;
    }
  }
  close BSSTAT;

  if ($baseServer_runnning) {
    print "baseServer is running\n";
    # Check the last runstate
    open LASTSTATE, "cat /usr/tmp/laststate |";
    my $laststate = <LASTSTATE>;
    unless ($laststate =~ m/ShutDown/) {
      # We are running, so make sure the MSPs are online and the bus is active
      if (checkMSPs()) {
        # One or more MSPs didn't initialize or the bus hung, shut down Phase2 and restart
        if ($laststate =~ m/Phase2/) {
          safeRestart();
        }
        else {
          # We don't have a way of dealing with A.bus problems in other operating modes yet
          system("/home/bee/speakit2.pl MSP failure detected, please restart"); 
        }
      }
      else {
        # All the MSPs are online (or were), check the A.bus heartbeat
         if (checkHeartBeat() eq "Hung") {
           if ($laststate =~ m/Phase2/) {
             safeRestart();
           }
           else {
             # We don't have a way of dealing with A.bus problems in other operating modes yet
             system("/home/bee/speakit2.pl Access bus communications failure detected, please restart");
           }
         }
         else {
           print "ACCESS.bus heartbeat ok\n";
         }
      }
    }
    close LASTSTATE;
  }
  else {
    print "baseServer is not running\n";
  }
  # sleep for a bit, and check again
  sleep 10;  
}


close(WATCHDOG);


# Safely shutdown and restart in Phase2 mode
sub safeRestart {
  sendState("ShutDown");
  system("/home/bee/speakit2.pl fault detected, restarting Phase 2 demo");
  sleep 2;  
  checkAndReset();  # Check the A.bus and reset if need be
  system("/home/bee/speakit2.pl Restarting in 10");
  sleep 1;
  system("/home/bee/speakit2.pl 9");
  sleep 1;
  system("/home/bee/speakit2.pl 8");
  sleep 1;
  system("/home/bee/speakit2.pl 7");
  sleep 1;
  system("/home/bee/speakit2.pl 6");
  sleep 1;
  system("/home/bee/speakit2.pl 5");
  sleep 1;
  system("/home/bee/speakit2.pl 4");
  sleep 1;
  system("/home/bee/speakit2.pl 3");
  sleep 1;
  system("/home/bee/speakit2.pl 2");
  sleep 1;
  system("/home/bee/speakit2.pl 1");
  sleep 1;
  sendState("Phase2"); # restart Phase2
}


# Check the A.bus heartbeat, and reset if it is hung. If its ok, just return
sub checkAndReset {
  my $health = checkHeartBeat();
  while ($health eq "Hung") {
    resetBus();  # reset the bus
    sleep 5;     # sleep for 5 seconds and let everything settle 
    $health = checkHeartBeat(); # Check the heartbeat    
  }
}


# Request a service state change
sub sendState {
  my ($state) = @_;
  # this blocks until the web server closes the connection, so it may take a while
  system("lynx -dump -error_file=/home/httpd/html/logs/sendState-errors.log http://localhost/cgi-bin/zaza-control-simple?op_mode=" . $state . " >/home/httpd/html/logs/sendState.log");
}


# Check the baseServer log for MSP activity
sub checkMSPs {
  my %mspstat = ( '20'    => 0,
                  '21'    => 0,
                  '22'    => 0,
                  '30'    => 0,
                  '31'    => 0,
                  '32'    => 0,
                  '33'    => 0,
                );
  open MSPLOG, "grep 'en_ap()' /home/httpd/html/logs/baseServer.log |";
  while (<MSPLOG>) {
    $_ =~ m/\{000000(\d+)\}\s+/; 
    $mspstat{$1} = 1;
  }
  close MSPLOG;
  for $i (keys %mspstat) {
    if ($mspstat{$i} == 0) {
      print "MSP initialization failure:\n";
      for $j (keys %mspstat) {
        # print a table of the MSP detection states.
        print $j . "=>" . $mspstat{$j} . "\n";
      }
      return 1;
    } 
  }
  print "All MSPs are online\n";
  return 0;
}

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
