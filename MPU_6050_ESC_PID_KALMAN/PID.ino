

void init_pid()
{
  if ( !pid_ready )
  {
    //turn the PID on
    yw_pid.SetOutputLimits(-255.0, 255.0);
    ac_pid.SetOutputLimits(-255.0, 255.0);
    bd_pid.SetOutputLimits(-255.0, 255.0);

    if(read_mpu())
    {
      setpoint_ac = input_ypr[AC] ;
      setpoint_bd = input_ypr[BD] ;
      setpoint_yw = input_ypr[YW] ;
    }
    
    //turn the PID on
    yw_pid.SetMode(AUTOMATIC);
    ac_pid.SetMode(AUTOMATIC);
    bd_pid.SetMode(AUTOMATIC);


    Serial.println("pids are ready...");
    pid_ready = true;
  }
}

