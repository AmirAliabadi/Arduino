/*
 * 
 * 
 * 
http://www.thorlabs.com/tutorials.cfm?tabID=5dfca308-d07e-46c9-baa0-4defc5c40c3e 

While manual tuning can be very effective at setting a PID circuit for your specific system, it does require some amount of 
experience and understanding of PID circuits and response. The Ziegler-Nichols method for PID tuning offers a bit more 
structured guide to setting PID values. Again, you’ll want to set the integral and derivative gain to zero. Increase the 
proportional gain until the circuit starts to oscillate. We will call this gain level Ku. The oscillation will have a 
period of Pu. Gains are for various control circuits are then given below in the chart.


Control Type  Kp        Ki              Kd
P             0.50 Ku   -               -
PI            0.45 Ku   1.2 * (Kp/Pu)   -
PID           0.60 Ku   2.0 * (Kp/Pu)   Kp * (Pu/8)


Ku = P gain that starts system to oscillate
Pu = frequency of oscellation


Ku of 0.45 yields Pu of 6.69 [ ( 87(s)/13 waves ) ]

Kp = 0.60 * .45         =   0.27
Ki = 2.0 * (.24/6.69)   =   0.0717
Kd = .24 * (6.69/8)     =   0.2007
--------------------------------------------------------------------------------------------


Rule Name  Tuning Parameters
Classic Ziegler-Nichols   Kp = 0.6 Ku     Ti = 0.5 Tu     Td = 0.125 Tu
Pessen Integral Rule      Kp = 0.7 Ku     Ti = 0.4 Tu     Td = 0.15 Tu
Some Overshoot            Kp = 0.33 Ku    Ti = 0.5 Tu     Td = 0.33 Tu
No Overshoot              Kp = 0.2 Ku     Ti = 0.5 Tu     Td = 0.33 Tu

Ku = .45
period of osicallation Tu = 87/13 = 6.69
Kp = .6 * .45     = 0.270
Ki = .5 * 6.69    = 3.345
Kd = .125 * 6.69  = 0.83

/////////////////////////////////////////////
/// GOOD values found by trial and error ////
/// 0.5, 0.055, 0.201
/////////////////////////////////////////////

/////////////////////////////////////////////
// PID Sample time of 10
These pids tunings are even better, now that I have upped the PID compute cycle
0.7000  0.0550  0.2010

0.6660  0.0000  0.1000 slugish but very little overshoot
0.8000  0.0000  0.2000

// these one seems good
// very little over should but still not super resonsive when setpoint changes
0.7770  0.0001  0.3000


 * This one seems to work pretty good.
 * 0.980 0.390 0.490
 * 
 * this is what I'm going to start with:
 * .8 .4 .3


/////////////////////////////////////
// bad values:
0.9000  0.0001  0.5000 -- will hold but push it and it will oscillate like crazy

1.5, .02, .25 -- this was good, but batteries might have been a little low so need to double check


Integral term seems to help with getting closer to desired setpoint

with the full quad!
* the weight of the quad is a factor (duh!)
* pid values that seem to work ok:
*** 4.2 / 0.0 / 1.8 
* have not introduced the I term yet
* working on the YAW pid values, these will be different than the pitch/roll values

* also saw 1.3/0/4.7 but this was with a big YAW PID setup

/////////////////////////////////////////////
*/


void init_pid()
{
  Serial.println("#initializing pid...");
  
  //turn the PID on
  yw_pid.SetOutputLimits(-200, 200);
  ac_pid.SetOutputLimits(-100, 100);
  bd_pid.SetOutputLimits(-100, 100);

  yw_pid.SetSampleTime(10);
  ac_pid.SetSampleTime(10);
  bd_pid.SetSampleTime(10);

  output_ypr[YW] = 0;
  output_ypr[AC] = 0;
  output_ypr[BD] = 0;

  ac_pid.SetTunings(pid_xx_kp[0], pid_xx_ki[0], pid_xx_kd[0]);
  bd_pid.SetTunings(pid_xx_kp[0], pid_xx_ki[0], pid_xx_kd[0]);
  yw_pid.SetTunings(pid_yw_kp[0], pid_yw_ki[0], pid_yw_kd[0]);    

  yw_pid.SetMode(AUTOMATIC);
  ac_pid.SetMode(AUTOMATIC);
  bd_pid.SetMode(AUTOMATIC);

  system_check |= INIT_PID_ON ;
}

void pid_off()
{
  output_ypr[AC] = 0.0;
  output_ypr[BD] = 0.0;
  output_ypr[YW] = 0.0;

  //turn the PID on
  yw_pid.SetMode(MANUAL);
  ac_pid.SetMode(MANUAL);
  bd_pid.SetMode(MANUAL);

  system_check &= ~INIT_PID_ON ;
}

void update_pid_settings()
{
/*
    ////////////////////////////////////////////////////
    // Reset of PID when setpoint changes
    if( setpoint_changed & SETPOINT_CHANGED_AC ) {ac_pid.Reset();}
    if( setpoint_changed & SETPOINT_CHANGED_BD ) {bd_pid.Reset();}
    if( setpoint_changed & SETPOINT_CHANGED_YW ) {yw_pid.Reset();}
    
    setpoint_changed = SETPOINT_UNCHANGED;
    //
    ////////////////////////////////////////////////////
*/
    //////////////////////////////////////////////////
    // adaptive PID settings
    int i = 0;
    //if( abs(setpoint[AC] - input_ypr[AC]) > 10 ) i = 1;
    ac_pid.SetTunings(pid_xx_kp[i], pid_xx_ki[i], pid_xx_kd[i]);

    i = 0;
    //if( abs(setpoint[BD] - input_ypr[BD]) > 10 ) i = 1;
    bd_pid.SetTunings(pid_xx_kp[i], pid_xx_ki[i], pid_xx_kd[i]);

    i = 0;
    if( abs(setpoint[YW] - input_ypr[YW]) > 10 ) i = 1;
    yw_pid.SetTunings(pid_yw_kp[i], pid_yw_ki[i], pid_yw_kd[i]);      
    //
    /////////////////////////////////////////////////
}

