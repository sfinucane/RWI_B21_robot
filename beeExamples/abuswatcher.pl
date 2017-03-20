#!/usr/bin/perl
#

# abuswatcher.pl
# Keep track of Zaza's ACCESS bus at baseServer startup, and warn if there
# is trouble.

# 0.1  12-19-2001
# First working version

my $msptwenty = 0;
my $msptwentyone = 0;
my $msptwentytwo = 0;
my $mspthirty = 0;
my $mspthirtyone = 0;
my $mspthirtytwo = 0;
my $mspthirtythree = 0;

sub parselog {
 # system("grep 'en_ap()' /usr/local/apache/htdocs/logs/baseServer.log > /var/tmp/abusstatus");
  system("grep 'en_ap()' /home/httpd/html/logs/baseServer.log > /var/tmp/abusstatus");
  open (ABUSSTATUS, "/var/tmp/abusstatus");

  while (<ABUSSTATUS>) {
    $_ =~ m/\{000000(\d+)\}\s+/; 
    if ($1 eq "20") {
      $msptwenty = 1;
      print "20\n";
    }
    if ($1 eq "21") {
      $msptwentyone = 1;
      print "21\n";
    }
    if ($1 eq "22") {
      $msptwentytwo = 1;
      print "22\n";
    }
    if ($1 eq "30") {
      $mspthirty = 1;
      print "30\n";
    }
    if ($1 eq "31") {
      $mspthirtyone = 1;
      print "31\n";
    }
    if ($1 eq "32") {
      $mspthirtytwo = 1;
      print "32\n";
    }
    if ($1 eq "33") {
      $mspthirtythree = 1;
      print "33\n";
    }
  }
  close ABUSSTATUS;
}


#main {
  parselog();

  my $mspstat = $msptwenty && $msptwentyone && $msptwentytwo && $mspthirty && $mspthirtyone && $mspthirtytwo && $mspthirtythree;
  if (!$mspstat) {
    my $speakstring = "Error, M S P ";
    if (!$msptwenty) {
      $speakstring .= "20,";
    }
    if (!$msptwentyone) {
      $speakstring .= " 21,";
    }
    if (!$msptwentytwo) {
      $speakstring .= " 22,";
    }
    if (!$mspthirty) {
      $speakstring .= " 30,";
    }
    if (!$mspthirtyone) {
      $speakstring .= " 31,";
    }
    if (!$mspthirtytwo) {
      $speakstring .= " 32,";
    }
    if (!$mspthirtythree) {
      $speakstring .= " 33,";
    }

    $speakstring .= " failure, Please restart."; 

    for $i ( 0 .. 3) {
      system("/home/bee/speakit2.pl $speakstring");
      sleep 5;
    }
  }
#  else {
#    # Everything is fine
#    system("/home/bee/speakit2.pl Normal startup.");
#  }


#}
