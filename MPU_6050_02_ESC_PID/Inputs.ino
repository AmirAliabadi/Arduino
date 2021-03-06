
//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
void read_throttle()
{
  if( system_check & INIT_THROTTLE_ZEROED ) 
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
  if( system_check & INIT_ESC_ARMED ) {  
    setpoint[YW] = INPUT_SETPOINT_YAW ;
    setpoint[AC] = INPUT_SETPOINT_ROLL ;
    setpoint[BD] = INPUT_SETPOINT_PITCH ;  
    
  } else {
    setpoint[YW] = 0.0 ;
    setpoint[AC] = 0.0 ;
    setpoint[BD] = 0.0 ;  
    
  }
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}
///////////////////////////////////////////////////////////////////////

