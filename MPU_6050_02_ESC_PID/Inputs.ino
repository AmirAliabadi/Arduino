
//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
void read_throttle()
{
  if( system_check & INIT_ESC_ARMED ) 
  {
    INPUT_THRUST = constrain(INPUT_THRUST, MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming
  }
  else
  {
    INPUT_THRUST = constrain(INPUT_THRUST, MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming 
  
    if( INPUT_THRUST == 0.0 )
    {
      system_check |= INIT_THROTTLE_ZEROED ;
    }
  }
}

void read_setpoint()
{
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}
///////////////////////////////////////////////////////////////////////
