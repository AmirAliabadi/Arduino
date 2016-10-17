/*
 * 
 * 
 * 
http://www.thorlabs.com/tutorials.cfm?tabID=5dfca308-d07e-46c9-baa0-4defc5c40c3e 

While manual tuning can be very effective at setting a PID circuit for your specific system, it does require some amount of 
experience and understanding of PID circuits and response. The Ziegler-Nichols method for PID tuning offers a bit more 
structured guide to setting PID values. Again, youâ€™ll want to set the integral and derivative gain to zero. Increase the 
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

// stable + rate pid
// -45 - 45 and -200 to 200 / p=1.2 , i=.44



commit ce763f1b34255f81bee1bf61d10230a3393655f2
Author: AmirAliabadi <amiraliabadi@gmail.com>
Date:   Tue May 31 22:07:55 2016 -0700

    added a comment with old pid setting where I first observed the d-term chatter
/////////////////////////////////////////////
*/


void init_pid()
{
  Serial.println("#init pid");

  update_pid_settings();

  set_pid_refresh_rate();
  
  for(byte i=YW; i<=AC; i++ ) {  
    pid_stable[i].SetOutputLimits(-300, 300);
    pid_stable[i].SetMode(AUTOMATIC);

#ifdef CASCADE_PIDS    
    pid_rate[i].SetOutputLimits(-300, 300);    
    pid_rate[i].SetMode(AUTOMATIC);
#endif
      
  }
  
  system_check |= INIT_PID_ON ;
}

void set_pid_refresh_rate()
{
  for(byte i=YW; i<=AC; i++ ) {  
    pid_stable[i].SetSampleTime(pid_refresh_rate);
#ifdef CASCADE_PIDS    
    pid_rate[i].SetSampleTime(pid_refresh_rate);      
#endif
  }
}

void pid_reset() 
{
  for(byte i=YW; i<=AC; i++ ) {
    pid_stable[i].Reset();
#ifdef CASCADE_PIDS    
    pid_rate[i].Reset();  
#endif
  }
}

void update_pid_settings()
{
    ////////////////////////////////////////////////////
    // Reset of PID when setpoint changes
    //if( setpoint_changed & SETPOINT_CHANGED_AC ) {pid_ac_stable.Reset(); ac_rat.Reset(); }
    //if( setpoint_changed & SETPOINT_CHANGED_BD ) {pid_bd_stable.Reset(); bd_rat.Reset(); }
    //if( setpoint_changed & SETPOINT_CHANGED_YW ) {pid_yw_stable.Reset(); }
    setpoint_changed = SETPOINT_UNCHANGED;
    //
    ///////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    // 
    pid_stable[YW].SetTunings(INPUT_YAW_PID_P, INPUT_YAW_PID_I, INPUT_YAW_PID_D);    
    pid_stable[BD].SetTunings(INPUT_STB_PID_P, INPUT_STB_PID_I, INPUT_STB_PID_D);
    pid_stable[AC].SetTunings(INPUT_STB_PID_P, INPUT_STB_PID_I, INPUT_STB_PID_D);

#ifdef CASCADE_PIDS
    pid_rate[YW].SetTunings(INPUT_YAW_RATE_PID_P, INPUT_YAW_RATE_PID_I, INPUT_YAW_RATE_PID_D);    
    pid_rate[BD].SetTunings(INPUT_RAT_PID_P, INPUT_RAT_PID_I, INPUT_RAT_PID_D);
    pid_rate[AC].SetTunings(INPUT_RAT_PID_P, INPUT_RAT_PID_I, INPUT_RAT_PID_D);
#endif
    //
    //////////////////////////////////////////////////
    
    update_pid_settings_needed = 0;
}


