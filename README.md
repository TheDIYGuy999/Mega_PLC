# This is an Arduino based, modular Industrial PLC

## I have designed this PLC back in 2016. It is currently in use in an industrial screw tightening robot:
- 4 stepper motors for X, Y, Z axes and for screw tightening
- Matrix keyboard
- I2C LCD
- Digital I/O in use for pneumatic screw feeding system, buttons and reference switches

## Features:
- Works wit an Arduino MEGA or DUE
- DO board with 16 24V, 500mA PNP digital outputs, optoinsulated, short circuit protected @ about 900mA
- DI board with 16 24V PNP digital inputs, optoinsulated
- Stepper board for 4 stepper motors, torque adjustable via PWM
- Slot for communication board (I2C, CAN, SPI, 2x Serial)
- Slot for analog board (Analog in, PWM, DAC, I2C)

## This repo contains:
- Eagle schematics
- Eagle board files
- PDF schematics
- The analog and communication boards were made out of perfboard and are NOT included. Maybe someone other is interested in adding them.

## Pictures:
![](https://github.com/TheDIYGuy999/Mega_PLC/blob/master/bottom%20view.jpg)

![](https://github.com/TheDIYGuy999/Mega_PLC/blob/master/top%20view.jpg)

![](https://github.com/TheDIYGuy999/Mega_PLC/blob/master/wired.jpg)

## Videos:
- I/O boards: https://www.youtube.com/watch?v=W2pYYh6aUXc
- First tepper tests: https://www.youtube.com/watch?v=gJDdZBuH5Kc

(c) 2016  TheDIYGuy999
