# Open Challenge - driving demonstration

Video: https://www.youtube.com/watch?v=Lr_X0RzbjXM

The car completes the open round on the STM32 alone - the Raspberry Pi is physically unplugged.
It holds heading with the IMU on the straights, detects each corner from the floor colour line
plus the wall, turns a gyro-terminated 90 degrees, re-centres in the lane, and stops in the
start section after three laps.

Firmware: ../src/open_round/OpenRound.cpp
