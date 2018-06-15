#!/usr/bin/env bash

#connect FC:58:FA:2F:59:55
info=$(bluetoothctl << EOF
info FC:58:FA:2F:59:55
EOF
)

is_connected=$(echo $info | egrep "Connected: yes" | wc -l)

#echo $is_connected
if [ "$is_connected" == 0 ]; then
    echo "trying to connect.."
bluetoothctl << EOF
 connect FC:58:FA:2F:59:55
EOF
else
    echo "already connected"
fi
