

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
  yaw_pid.resetITerm();
  att_pid_bd.resetITerm();  
  att_pid_ac.resetITerm();

#ifdef CASCADE_PIDS
  rate_pid_ac.resetITerm();
  rate_pid_bd.resetITerm();  
#endif  
}

unsigned long pid_select_channel;
unsigned long pid_tune_channel;
unsigned short index;
float c[3];
void update_pid_settings()
{
  if( ppm_read && ppm_sync ) {
    cli();
    pid_select_channel = ppm_channels[PID_SELECT_CHANNEL];
    pid_tune_channel = ppm_channels[PID_TUNE_CHANNEL] ;    
    sei();
  }  

  c[0] = INPUT_YAW_PID_P; 
  c[1] = INPUT_YAW_PID_I; 
  c[2] = INPUT_YAW_PID_D;
  yaw_pid.setControlCoeffs(c);

/*
#define INPUT_STB_PID_P       input_values[1]
#define INPUT_STB_PID_I       input_values[2]
#define INPUT_STB_PID_D       input_values[3]

#define INPUT_RAT_PID_P       input_values[4]
#define INPUT_RAT_PID_I       input_values[5]
#define INPUT_RAT_PID_D       input_values[6]
*/

/*
 *
 TX MIXER
 CHANNEL 7 MASTER
 CHANNEL 5 SLAVE 
 0% OFFSET
 NEGATIVE: 80%
 POSITIVE: 80%
 
stable P = 1000 +/- 10 => 0
stable I = 1100 +/- 10 => 1
  rate P = 1400 +/- 10 => 3
stable D = 1600 +/- 10 => 2
 rate I = 1900 +/- 10  => 4
 rate P = 2000 +/- 10  => 5
 
 */

  index = (
      pid_select_channel > 990  &&  pid_select_channel < 1010 ? 1 :   // 1000  +/- 10
      pid_select_channel > 1090 &&  pid_select_channel < 1110 ? 2 :   // 1100  +/- 10
      pid_select_channel > 1590 &&  pid_select_channel < 1610 ? 3 :   // 1600  +/- 10
      pid_select_channel > 1390 &&  pid_select_channel < 1410 ? 4 :   // 1400  +/- 10      
      pid_select_channel > 1890 &&  pid_select_channel < 1910 ? 5 :   // 1900  +/- 10          
      pid_select_channel > 1990 &&  pid_select_channel < 2010 ? 6 :   // 2000  +/- 10    
      32          
    ) ;
    
  if( index >=1 && index <= 6 ) {
    
    input_values[ index ] += ( 
      pid_tune_channel < 1100 ? -0.0010 :     
      pid_tune_channel < 1300 ? -0.0001 : 
      pid_tune_channel > 1900 ? +0.0010 :     
      pid_tune_channel > 1700 ? +0.0001 : 
      0.0000  );
    input_values[ index ] = constrain( input_values[ index ], 0, 10.0 );
  
    c[0] = INPUT_STB_PID_P; 
    c[1] = INPUT_STB_PID_I; 
    c[2] = INPUT_STB_PID_D;
    att_pid_ac.setControlCoeffs(c);  
    att_pid_bd.setControlCoeffs(c);  
  
  #ifdef CASCADE_PIDS
    c[0] = INPUT_RAT_PID_P; 
    c[1] = INPUT_RAT_PID_I; 
    c[2] = INPUT_RAT_PID_D;
    rate_pid_ac.setControlCoeffs(c);
    rate_pid_bd.setControlCoeffs(c);
  #endif    
  
    if( INPUT_STB_PID_I == 0 ) {
      att_pid_ac.resetITerm();    
      att_pid_bd.resetITerm();      
    }
    if( INPUT_RAT_PID_I == 0 ) {
      rate_pid_ac.resetITerm();  
      rate_pid_bd.resetITerm();    
    }  
  }
}

void do_pid_compute()
{
//  attitude_correction[YAW] = yaw_pid.calculate(setpoint[YAW],  current_attitude[YAW]);
  attitude_correction[YAW] = yaw_pid.calculate(setpoint[YAW], current_rate[YAW], false  );
  attitude_correction[AC] = att_pid_ac.calculate(setpoint[AC], current_attitude[AC], true ); 
  attitude_correction[BD] = att_pid_bd.calculate(setpoint[BD], current_attitude[BD], false );   

#ifdef CASCADE_PIDS
//  rate_correction[YAW] = yaw_pid.calculate( attitude_correction[YAW], current_rate[YAW]  );
  rate_correction[AC] = rate_pid_ac.calculate( attitude_correction[AC], current_rate[AC] );
  rate_correction[BD] = rate_pid_bd.calculate( attitude_correction[BD], current_rate[BD] );  
#endif   

}


