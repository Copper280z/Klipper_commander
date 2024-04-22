THIS IMPLEMENTATION IS INCOMPLETE. IT DOES NOT CURRENTLY WORK. 

It may work at some point in the future, that time period may be
far away, it may also be brief.


This library is intended to implement the minimal set of functions
required to enumerate a Klipper MCU with a single "Stepper" object 
and "Endstop" object. 

The goal of this is to be able to control an MCU running SimpleFOC 
or any other motor control firmware over the standard Klipper protocol. 
This is advantageous over a step-dir interface because the Klipper 
"queue step" command encodes enough information to calculate a velocity
and acceleration feed forward term, in addition to the position vs clock.

