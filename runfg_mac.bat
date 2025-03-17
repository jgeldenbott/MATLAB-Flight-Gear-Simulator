#!/bin/zsh

cd /Applications/FlightGear.app/Contents/Resources

export FG_ROOT=/Applications/FlightGear.app

./../MacOS/fgfs --fdm=null --native-fdm=socket,in,30,localhost,5502,udp \
  --prop:/sim/rendering/shaders/quality-level=0 \
  --fg-aircraft="/Users/jaspergeldenbott/Library/Application Support/FlightGear/Aircraft/org.flightgear.fgaddon.stable_2020/Aircraft" \
  --aircraft=757-200-PW2040 \
  --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 \
  --disable-sound --in-air --airport=KSEA --runway=16L --altitude=7224 \
  --heading=113 --offset-distance=4.72 --offset-azimuth=0 &
