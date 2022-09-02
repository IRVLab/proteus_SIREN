#!/bin/bash
# echo "Initializing everything first"
# rosservice call /loco/proteus/siren/tts/Affirmative 
# rosservice call /loco/proteus/siren/synth/Affirmative
# rosservice call /loco/proteus/siren/tts/Affirmative 
# rosservice call /loco/proteus/siren/synth/Affirmative 

# echo "Starting sonemes in 30 seconds"
# sleep 30

# sleep 5
# echo "Starting tts sonemes"
# rosservice call /loco/proteus/siren/tts/Affirmative 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Affirmative 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Attention
# sleep 1
# rosservice call /loco/proteus/siren/tts/Come_To_Me
# sleep 1
# rosservice call /loco/proteus/siren/tts/Danger
# sleep 1
# rosservice call /loco/proteus/siren/tts/Follow_Me
# sleep 1
# rosservice call /loco/proteus/siren/tts/Follow_You
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: 1.0
#     z: 0.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: -1.0
#     z: 0.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: 0.0
#     z: 1.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: 0.0
#     z: -1.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: 1.0
#     z: 1.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: 1.0
#     z: -1.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: -1.0
#     z: 1.0
#     w: 1.0
# mode: ''" 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Go_To_Direction "transform:
#   translation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   rotation:
#     x: 0.0
#     y: -1.0
#     z: -1.0
#     w: 1.0
# mode: ''" 
# rosservice call /loco/proteus/siren/tts/Malfunction
# sleep 1
# rosservice call /loco/proteus/siren/tts/Stay
# sleep 1
# rosservice call /loco/proteus/siren/tts/Negative
# sleep 1
# rosservice call /loco/proteus/siren/tts/Remaining_Battery " quantity: 1.0"
# sleep 1
# rosservice call /loco/proteus/siren/tts/Remaining_Battery " quantity: 0.75"
# sleep 1
# rosservice call /loco/proteus/siren/tts/Remaining_Battery " quantity: 0.5"
# sleep 1
# rosservice call /loco/proteus/siren/tts/Remaining_Battery " quantity: 0.25"
# sleep 1
# rosservice call /loco/proteus/siren/tts/Remaining_Battery " quantity: 0.01"
# sleep 1
# rosservice call /loco/proteus/siren/tts/Wait_For_Command 
# sleep 1
# rosservice call /loco/proteus/siren/tts/Which_Way

sleep 5
echo "Starting tonal sonemes"
rosservice call /loco/proteus/siren/synth/Affirmative 
sleep 1
rosservice call /loco/proteus/siren/synth/Affirmative 
sleep 1
rosservice call /loco/proteus/siren/synth/Attention
sleep 1
rosservice call /loco/proteus/siren/synth/Come_To_Me
sleep 1
rosservice call /loco/proteus/siren/synth/Danger
sleep 1
rosservice call /loco/proteus/siren/synth/Follow_Me
sleep 1
rosservice call /loco/proteus/siren/synth/Follow_You
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: -1.0
    z: 0.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: -1.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 1.0
    z: 1.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 1.0
    z: -1.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: -1.0
    z: 1.0
    w: 1.0
mode: ''" 
sleep 1
rosservice call /loco/proteus/siren/synth/Go_To_Direction "transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: -1.0
    z: -1.0
    w: 1.0
mode: ''" 
rosservice call /loco/proteus/siren/synth/Malfunction
sleep 1
rosservice call /loco/proteus/siren/synth/Stay
sleep 1
rosservice call /loco/proteus/siren/synth/Negative
sleep 1
rosservice call /loco/proteus/siren/synth/Remaining_Battery " quantity: 1.0"
sleep 1
rosservice call /loco/proteus/siren/synth/Remaining_Battery " quantity: 0.75"
sleep 1
rosservice call /loco/proteus/siren/synth/Remaining_Battery " quantity: 0.5"
sleep 1
rosservice call /loco/proteus/siren/synth/Remaining_Battery " quantity: 0.25"
sleep 1
rosservice call /loco/proteus/siren/synth/Remaining_Battery " quantity: 0.01"
sleep 1
rosservice call /loco/proteus/siren/synth/Wait_For_Command 
sleep 1
rosservice call /loco/proteus/siren/synth/Which_Way