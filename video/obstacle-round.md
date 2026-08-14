# Obstacle Challenge - driving demonstration

Video: https://www.youtube.com/watch?v=ZjexvNc6L4U

The car completes the obstacle round with the Raspberry Pi and fisheye camera added. Same
navigation core as the open round, and when the camera reports a pillar it runs the offset-based
avoidance: red pillars on the right, green on the left, swerving to a distance set by the
pillar position, holding until the colour clears, and returning to centre. The front-proximity
failsafe handles near-misses.

Firmware: ../src/obstacle_round/ObstacleRound.cpp  Â·  Vision: ../src/vision/main.py
