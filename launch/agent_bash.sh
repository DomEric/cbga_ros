#!/bin/bash

num_agents=$1

echo "Launching $num_agents Agent nodes..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}



i=0
head -$num_agents|
while read line; do
	linearray=($line)
	ROS_NAMESPACE="Agent_$i" rosrun cbga agent_test.py _address:="${linearray[0]}"  &
	((i++))
done
echo "DONE"

cat