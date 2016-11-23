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


-----
motor thrust test
Spyder ZTW 30AMP with the Amazon 1000kv 2212 non-name motors
VA @ 4.20v		VC @ 4.20v
1210 -  60g		1210 -  68g
1300 - 180g		1300 - 168g
1400 - 270g		1400 - 308g
1500 - 430g		1500 - 410g
1600 - 600g		1600 - 600g