#!/bin/bash
set -e

export DISPLAY=:99

mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix
rm -f /tmp/.X11-unix/X99 || true

Xvfb :99 -screen 0 1280x800x16 &
sleep 2

fluxbox &

x11vnc -display :99 -forever -nopw -shared &
sleep 2

websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
echo "noVNC running at: http://localhost:6080/vnc.html"

tail -f /dev/null
