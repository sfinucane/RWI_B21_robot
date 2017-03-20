#!/bin/sh
#

echo "Stopping wander demo..."

killall -HUP wander-laser
killall -HUP wanderstart

