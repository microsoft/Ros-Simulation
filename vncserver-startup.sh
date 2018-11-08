#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

VNC_PORT="590"${DISPLAY:1}

export USER="root"

##change vnc password
echo ${VNC_PW} | vncpasswd -f > /root/.vnc/passwd
chown -R root:root /root/.vnc/passwd
chmod 0600 /root/.vnc/passwd

##start vncserver
tightvncserver -kill :1 && rm -rfv /tmp/.X* ; echo "Remove old vnc locks to be a reattachable container"

tightvncserver $DISPLAY -p /root/.vnc/passwd

sleep 1
##log connect options
echo -e "\n------------------ VNC environment started ------------------"
echo -e "\nVNCSERVER started on PORT= $VNC_PORT"

for i in "$@"
do
case $i in
    # if option `-t` or `--tail-log` block the execution and tail the VNC log
    -t|--tail-log)
    echo -e "\n------------------ /root/.vnc/*$DISPLAY.log ------------------"
    tail -f /root/.vnc/*$DISPLAY.log
    ;;
    *)
    # unknown option ==> call command
    exec $i
    ;;
esac
done