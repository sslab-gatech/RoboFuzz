#!/usr/bin/env bash

i=0
for word in $(aspell -d en dump master | aspell -l en expand | head -n 5); do
  echo "{data: \"${word}\"}" > "/tmp/samples/sample-${i}"
  i=$((i+1))
done

pgrep listener || exit 0

while true; do
  STR=$(/usr/bin/radamsa /tmp/samples/sample-*)
  echo "$STR"
  (ros2 topic pub --once /chatter \
    std_msgs/String "${STR}" 2>&1) > /dev/null
  test $? -gt 127 && break # break on segfaults
  pgrep listener || break
done
echo "SEGV"
echo "$STR" > msg
