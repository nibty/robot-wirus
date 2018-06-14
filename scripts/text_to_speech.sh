#!/usr/bin/env bash

set -e

(
  flock -n 200

  echo "$1" | festival --tts

  # alt 1
  #espeak -v mb-us2 -s 140 "$1" --stdout | aplay -D bluealsa:HCI=hci0,DEV=FC:58:FA:2F:59:55,PROFILE=a2dp

  # alt 2
  #echo "$1" |
  #text2wave -scale 3 -eval "(voice_us3_mbrola)" -o /tmp/tts.wav &&
  #cat /tmp/tts.wav |
  #aplay -D bluealsa:HCI=hci0,DEV=FC:58:FA:2F:59:55,PROFILE=a2dp &&
  #rm /tmp/tts.wav
) 200>/var/lock/tts.lock
