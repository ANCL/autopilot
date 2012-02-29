#!/bin/sh
# This script detects which ethernet port is connected and 
# loads the appropriate settings and software

# Define some variables
IOPKT=io-pkt-v4-hc
VIP_DRIVER=/lib/dll/devn-i82544.so
LAB_DRIVER=/lib/dll/devn-speedo.so
IP="192.168.1.21"
NETMASK="255.255.255.0"
EXIT_SUCCESS=0

# function to test if cable is connected
cable_connected () {
if ! ifconfig > /dev/zero
then 
    return 1
fi
STATUS=`ifconfig | grep status | sed -e "s/[ \t]*//g" | cut -d':' -f2`
if [ "$STATUS" = "active" ]
then 
    return 0
else 
    return 1
fi
}

# kill any running network drivers
kill_net_drivers () {
PIDS=`ps -A | grep io-pkt-v4-hc | sed -e 's/^[ \t]*//' | cut -d' ' -f1 | tr "\n" " "`
for PID in $PIDS
do
    kill $PID
done
}

restart_sshd () {
PIDS=`ps -A | grep sshd | sed -e 's/^[ \t]*//' | cut -d' ' -f1 | tr "\n" " "`
for PID in $PIDS
do
    kill $PID
done
/usr/sbin/sshd
}

# load settings for field
load_field_config () {
ifconfig en0 $IP netmask $NETMASK
restart_sshd
/etc/autopilot/autopilot.sh
}

# load settings for lab
load_lab_config () {
ifconfig en0 $IP netmask $NETMASK
qconn
restart_sshd
}

SUCCESS=false
while [ "$SUCCESS" = "false" ]
do
    if ! cable_connected
    then 
#echo "cable disconnected"

	kill_net_drivers
	$IOPKT -d $LAB_DRIVER
	sleep 10
	if cable_connected
 	then
	    echo "Loaded driver for LAB"
	    load_lab_config
	    SUCCESS=true
	else
	    kill_net_drivers
# Attempt to connect to VIP Modem
	    $IOPKT -d $VIP_DRIVER
	    sleep 10
	    if cable_connected
	    then
		echo "Loaded driver for VIP Modem"
		load_field_config
		SUCCESS=true
	    fi
	fi
    else
#	echo "cable connected"
	SUCCESS=true
    fi

done