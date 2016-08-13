SCARA Firmware
==============

I started off just modifying misan's
[dcservo](https://github.com/misan/dcservo) code, then extended this with the
forward/inverse kinematics required by
[scarabot](https://github.com/lerouxb/scarabot) and a bunch of other things. It
was just about to start interpreting gcode when I realised that my mechanics
wasn't going to cut it ;)

It uses the excellent [Arduino Makefile](https://github.com/sudar/Arduino-Makefile).
