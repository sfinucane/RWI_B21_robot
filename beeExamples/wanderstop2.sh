#!/bin/sh
#

echo "Stopping wander demo..."
echo "Stopping wander demo." | /home/bee/speakit.pl &

killall -HUP wander-laser
killall -HUP wanderstart2
#killall -HUP reaction3
killall -HUP reaction
killall -HUP battwatcher.pl

