Testing program for the motor and encoder:

Tasks:
- currently reads encoder using timer 2 and converts to the angle of output shaft
- uses timer 3 to get dt for the control loop
- calculates angular velocity using dt as well as angle data
- need to implement a filter 
- need to find a way to set an origin point

Simulation & Testing:
- currently outputs angle position and velocity through UART2 -> ST-Link -> COM3
- able to read COM3 through Putty and can plot values over time using MATLAB

Wiring:
red -> CW signal
black -> microcontroller ground
yellow -> PA1
green -> PA0
blue -> microcontroller 5V
white -> CCW signal

