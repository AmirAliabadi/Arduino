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
These pids tunings are even better, now that I have upped the PID compute cycle
0.7000  0.0550  0.2010

0.6660  0.0000  0.1000 slugish but very little overshoot
0.8000  0.0000  0.2000

Integral term seems to help with getting closer to desired setpoint
/////////////////////////////////////////////
*/


void init_pid()
{
  if ( !pid_ready )
  {
    //turn the PID on
    yw_pid.SetOutputLimits(-255.0, 255.0);
    ac_pid.SetOutputLimits(-255.0, 255.0);
    bd_pid.SetOutputLimits(-255.0, 255.0);

    yw_pid.SetSampleTime(10);
    ac_pid.SetSampleTime(10);
    bd_pid.SetSampleTime(10);

    setpoint_ac = 0.0 ; // read_setpoint_ac() ; // 0.0
    setpoint_bd = 0.0 ;
    setpoint_yw = 0.0 ;

    output_yw = 0;
    output_ac = 0;
    output_bd = 0;

    yw_pid.SetMode(AUTOMATIC);
    ac_pid.SetMode(AUTOMATIC);
    bd_pid.SetMode(AUTOMATIC);

    pid_ready = true;
  }
}

void pid_off()
{
    output_ac = 0.0;
    output_bd = 0.0;
    output_yw = 0.0;
  
    //turn the PID on
    yw_pid.SetMode(MANUAL);
    ac_pid.SetMode(MANUAL);
    bd_pid.SetMode(MANUAL);

    pid_ready = false ;
}
