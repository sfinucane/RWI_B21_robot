#!/usr/bin/perl
#

# Set this to 1 for testing
$test = 0;


print "Starting wander demo...\n";

system("killall -HUP wander");

#/home/bee-new/src/beeExamples/wander -safetyMargin=30 -exploreRange=1000 -transSpeed=20 -transAccel=60 &
#/home/bee-new/src/beeExamples/wander-laser -usePantilt=n -safetyMargin=20 -exploreRange=1000 -transSpeed=20 -transAccel=60 &
#/home/bee-new/src/beeExamples/reaction3 -zap=Y -Dynamic=Y &


if ($test) {
# This is handy for testing
  system("cd /home/bee; src/beeExamples/wander-laser -limp=Y -usePantilt=n -safetyMargin=20 -exploreRange=1000 -transSpeed=20 -transAccel=60 & >/home/httpd/html/logs/wander-laser.log 2>&1 &") == 0
         or print "wander-laser startup failed: $?";
  system("/home/bee/speakit2.pl I am now in test mode.");
  system("/home/bee/src/beeExamples/battwatcher.pl &");
}
else {
  system("/home/bee/speakit2.pl Starting wander demo.");
  system("cd /home/bee; src/beeExamples/wander-laser -usePantilt=n -safetyMargin=20 -exploreRange=1000 -transSpeed=20 -transAccel=60 & >/home/httpd/html/logs/wander-laser.log 2>&1 &") == 0
         or print "wander-laser startup failed: $?";
  system("/home/bee/src/beeExamples/battwatcher.pl &");
}

#system("cd /home/bee-new; src/beeExamples/reaction3 -zap=Y -Dynamic=Y >/usr/local/apache/htdocs/logs/reaction.log 2>&1 &") == 0
#         or print "reaction startup failed: $?";
system("cd /home/bee; src/reaction/reaction -zap=Y -Dynamic=Y >/home/httpd/html/logs/reaction.log 2>&1 &") == 0
         or print "reaction startup failed: $?";

