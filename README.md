# DE10_Standard_Interrupt_Handler

The project has been carried out by developing a kernel module that is inserted into the Linux operating system supported by the DE10-Standard Computer System board. In other words, it has made use of the hardware of the board by developing modules that control it directly, without using a high-level application that makes use of the various modules.  

The main goal of the project is to control a system consisting of two servomotors by means of a 2-axis accelerometer. In addition, an alternative control method has been developed which allows the servomotors to be controlled using two push-buttons. In this way, the servomotors can be placed in a specific desired position.

The control mode can be determined by the user using the switches. Depending on their configuration, which ones are on and which ones are off, one control mode or another will be activated. The control mode assignment becomes effective only when the user presses a particular push-button. Therefore, changing the position of the swtiches without pressing that button does not generate any change in the system.

In the first mode, the servomotors are controlled using two of the push-buttons available on the board. One of the push buttons moves the servomotors clockwise and the other counterclockwise.

In the second mode, the two degrees of tilt detected by the accelerometer are used to control the servomotors. Tilt in the X-axis, the longitudinal axis of the accelerometer, controls one servomotor and tilt in the Y-axis, the transverse axis, controls the other. In both cases the servomotors are controlled in the same way: by PWM signals. This type of signal is also the way the accelerometer encodes the information it collects. 

In addition to this, the time progress is continuously shown on the 7-segment display, showing the minutes, seconds and tenths of a second elapsed since the start-up.
 
