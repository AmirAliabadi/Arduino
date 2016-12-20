/*
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

// more sample of pid values that work ok
2.245 0.0 0.0
0.499 0.017 0.056
alpha 0.44
pid refresh of 5
using the ESC library

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


-- switched out a spider ztw esc.
-- way better performance and pid tuning has changed.

-- with spider ztw ESC
0.71  0.0   0.0
0.509 0.007 0.024

-- some real nice performance finally, still cannot get the DTerm chatter
2.0     0.0 0.0
0.496   0.0 0.019
alpha .9
PID Refresh rate 26 -  Setting the pid refresh rate to high (like 1 or 5) make things worse
This is the angle reading rounding to the 1s place
MPU motian fusion of MPU6050_DLPF_BW_98
    
/////////////////////////////////////////////
*/


void init_pid()
{
  system_check |= INIT_PID_ON ;
}

void set_pid_refresh_rate()
{
}

void pid_reset() 
{
  for(int i=0; i< 2; i++)
    for(int j=0; j<3; j++)
      last_i_term[i][j] = 0.0;
}

void update_pid_settings()
{
}

#define MAX_ATTITUDE_YAW_I_TERM  210
#define MAX_ATTITUDE_PR_I_TERM  310
#define MAX_RATE_YAW_I_TERM  210
#define MAX_RATE_PR_I_TERM  310
void do_pid_compute()
{
  pid_temp_error = current_attitude[YAW] - setpoint[YAW]; 
  last_i_term[0][YAW] += pid_temp_error;
  if(last_i_term[0][YAW] > MAX_ATTITUDE_YAW_I_TERM)            last_i_term[0][YAW] = MAX_ATTITUDE_YAW_I_TERM;
  else if(last_i_term[0][YAW] < MAX_ATTITUDE_YAW_I_TERM * -1)  last_i_term[0][YAW] = MAX_ATTITUDE_YAW_I_TERM * -1;
 
  attitude_correction[YAW] = (INPUT_YAW_PID_P * pid_temp_error) + INPUT_YAW_PID_I * (last_i_term[0][YAW]) +  INPUT_YAW_PID_D * (pid_temp_error - last_d_error[0][YAW]) ;
  last_d_error[0][YAW] = pid_temp_error;
// ------------------

  pid_temp_error = current_attitude[BD] - setpoint[BD]; 
  last_i_term[0][BD] +=  pid_temp_error;
  if(last_i_term[0][BD] > MAX_ATTITUDE_PR_I_TERM)            last_i_term[0][BD] = MAX_ATTITUDE_PR_I_TERM;
  else if(last_i_term[0][BD] < MAX_ATTITUDE_PR_I_TERM * -1)  last_i_term[0][BD] = MAX_ATTITUDE_PR_I_TERM * -1;
  
  attitude_correction[BD] = (INPUT_STB_PID_P * pid_temp_error) + INPUT_STB_PID_I * (last_i_term[0][BD]) +  INPUT_STB_PID_D * (pid_temp_error - last_d_error[0][BD]) ;
  last_d_error[0][BD] = pid_temp_error;
// ------------------  

  pid_temp_error = current_attitude[AC] - setpoint[AC]; 
  last_i_term[0][AC] +=  pid_temp_error;
  if(last_i_term[0][AC] > MAX_ATTITUDE_PR_I_TERM)            last_i_term[0][AC] = MAX_ATTITUDE_PR_I_TERM;
  else if(last_i_term[0][AC] < MAX_ATTITUDE_PR_I_TERM * -1)  last_i_term[0][AC] = MAX_ATTITUDE_PR_I_TERM * -1;
  
  attitude_correction[AC] = ((INPUT_STB_PID_P * pid_temp_error) + INPUT_STB_PID_I * (last_i_term[0][AC]) +  INPUT_STB_PID_D * (pid_temp_error - last_d_error[0][AC])) * -1.0 ;
  last_d_error[0][AC] = pid_temp_error;
// ------------------    

#ifdef CASCADE_PIDS
  pid_temp_error = current_rate[YAW] - attitude_correction[YAW];
  last_i_term[1][YAW] += pid_temp_error;
  if(last_i_term[1][YAW] > MAX_RATE_YAW_I_TERM)            last_i_term[1][YAW] = MAX_RATE_YAW_I_TERM;
  else if(last_i_term[1][YAW] < MAX_RATE_YAW_I_TERM * -1)  last_i_term[1][YAW] = MAX_RATE_YAW_I_TERM * -1;
  
  rate_correction[YAW] = (INPUT_YAW_RATE_PID_P * pid_temp_error) + INPUT_YAW_RATE_PID_I * (last_i_term[1][YAW]) +  INPUT_YAW_RATE_PID_D * (pid_temp_error - last_d_error[1][YAW]) ;
  last_d_error[1][YAW] = pid_temp_error;
// --------------  
  
  pid_temp_error = current_rate[BD] - attitude_correction[BD];
  last_i_term[1][BD] += pid_temp_error;
  if(last_i_term[1][BD] > MAX_ATTITUDE_PR_I_TERM)            last_i_term[1][BD] = MAX_ATTITUDE_PR_I_TERM;
  else if(last_i_term[1][BD] < MAX_ATTITUDE_PR_I_TERM * -1)  last_i_term[1][BD] = MAX_ATTITUDE_PR_I_TERM * -1;
    
  rate_correction[BD] = (INPUT_RAT_PID_P * pid_temp_error) + INPUT_RAT_PID_I * (last_i_term[1][BD]) +  INPUT_RAT_PID_D * (pid_temp_error - last_d_error[1][BD]) ;
  last_d_error[1][BD] = pid_temp_error;  
// --------------

  pid_temp_error = current_rate[AC] - attitude_correction[AC];
  last_i_term[1][AC] += pid_temp_error;
  if(last_i_term[1][AC] > MAX_ATTITUDE_PR_I_TERM)            last_i_term[1][AC] = MAX_ATTITUDE_PR_I_TERM;
  else if(last_i_term[1][AC] < MAX_ATTITUDE_PR_I_TERM * -1)  last_i_term[1][AC] = MAX_ATTITUDE_PR_I_TERM * -1;
  
  rate_correction[AC] = (INPUT_RAT_PID_P * pid_temp_error) + INPUT_RAT_PID_I * (last_i_term[1][AC]) +  INPUT_RAT_PID_D * (pid_temp_error - last_d_error[1][AC]) ;
  last_d_error[1][AC] = pid_temp_error;  
// --------------  
#endif   

}


