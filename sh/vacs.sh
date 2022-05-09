#!/usr/bin/env bash
roslaunch rosbridge_server rosbridge_websocket.launch &

sleep 10s
cd /home/Documents/vacs
npm start &
#npx serve -s build &
chromium-browser http://localhost:3000 --kiosk --enable-features=OverlayScrollbar,OverlayScrollbarFlashAfterAnyScrollUpdate,OverlayScrollbarFlashWhenMouseEnter &
