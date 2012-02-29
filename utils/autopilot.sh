#!/bin/sh
AUTOPILOT=/usr/local/bin/autopilot
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
WD=/var/log/autopilot

cd $WD
$AUTOPILOT &
