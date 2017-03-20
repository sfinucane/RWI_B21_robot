#!/usr/bin/perl
#

# battwatcher.pl
# Keep track of Zaza's battery, and warn if there is trouble.
#
# per a 1998 RWI memo, the batteries should be recharged when 
# they reach 48V (this seems a little high)

# 0.1  11-9-2001
# First working version

my $battVoltage = 0.0;

sub getvoltage {

  system("tail -20 /home/httpd/html/logs/buttoncall.log | grep 'BASE_batteryVoltage' >/var/tmp/battvoltage2");

  # Battery monitoring stuff
  open (BVOLTAGE, "/var/tmp/battvoltage2");

  while (<BVOLTAGE>) {
    $_ =~ m/BASE_batteryVoltage =\s+(\d+).(\d+)/; 
    $battVoltage = "$1.$2";
  }
  close BVOLTAGE;
}



main {
  getvoltage();

  while (1) {
    sleep 60;

    getvoltage();

    if (($battVoltage <= 48.5) && ($battVoltage >= 48.0)) {
      # I need to take a nap soon
      system("/home/bee/speakit2.pl I need to take a nap soon.");
    }
    if (($battVoltage <= 48.0) && ($battVoltage >= 47.5)) {
      # I need to take a nap now
      system("/home/bee/speakit2.pl I need to take a nap now.");
    }
    if ($battVoltage < 47.5) {
      # We are now damaging the batteries
      system("/home/bee/speakit2.pl Danger, battery failure eminent, Please recharge me now!");
    }

  }

}
