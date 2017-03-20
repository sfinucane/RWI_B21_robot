#!/bin/sh
#

echo "Starting wander demo..."

killall -HUP wander

#/home/bee-new/src/beeExamples/wander -safetyMargin=30 -exploreRange=1000 -transSpeed=20 -transAccel=60 &
/home/bee/src/beeExamples/wander-laser -safetyMargin=20 -exploreRange=1000 -transSpeed=20 -transAccel=60 &

