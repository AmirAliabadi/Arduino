# Arduino
Arduino related projects and libraries. 

quadcopter, dmp 6050, PID tuning

frame + esc + motor = 900g
each esc+motor+prop = 100g
large battery = 410g
small battery = 175g
flight controller = 50g

AUW with the wooden frame:
~1400g with large battery
~1200g with small battery

PID values that worked on the single boom:
Stable: 3.075, 0.0, 0.0
Rate: 0.475, 0.0, 0.004
Alpha: 0.92

Stable: 3.11, 0.0, 0.00
Rate: 0.515, 0.0, 0.08
Alpha: 0.80



-----
For the threaded brushless motors:
The direction you want the motor to spin is the direction that causes the nut to get tighter if you hold the nut still with your fingers.


ESC motor pairs
attach sync noise not in sync
001 paired with 001
xxx paired with 002

attach sync noise in sync 
001 paired with 003
xxx paired with 002