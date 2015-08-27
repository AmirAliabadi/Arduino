
void init_pid()
{
  if ( !pid_ready )
  {
    //turn the PID on
    yw_pid.SetOutputLimits(-255.0, 255.0);
    ac_pid.SetOutputLimits(-255.0, 255.0);
    bd_pid.SetOutputLimits(-255.0, 255.0);

    setpoint_ac = 0.0 ;
    setpoint_bd = 0.0 ;
    setpoint_yw = 0.0 ;

    //turn the PID on
    yw_pid.SetMode(AUTOMATIC);
    ac_pid.SetMode(AUTOMATIC);
    bd_pid.SetMode(AUTOMATIC);

    pid_ready = true;
  }
}
