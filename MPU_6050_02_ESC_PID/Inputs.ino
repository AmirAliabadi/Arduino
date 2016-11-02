union _from_processing { 
  byte asBytes[8];    
  float asFloat[2];   
}                    
from_processing; 

void serialEvent() 
{
  if( Serial.readBytes( from_processing.asBytes, 8 ) == 8 ) {
    int cmd = (int)from_processing.asFloat[0];
    if( cmd <= 16 ) {
      input_values[ cmd ] = from_processing.asFloat[1] ;
      if( cmd > 0 and cmd < 13 ) update_pid_settings_needed = 1;
    } else {
      if(cmd == 100) {
        aserial_data_mode = (int)from_processing.asFloat[1];
              
      } else if (cmd == 101) {
        pid_attitude[AC].SetControllerDirection( (int)from_processing.asFloat[1] == 0.0 ? DIRECT : REVERSE );
        
      } else if (cmd == 102) { 
#ifdef CASCADE_PIDS         
        pid_rate[AC].SetControllerDirection( (int)from_processing.asFloat[1] == 0.0 ? DIRECT : REVERSE );
#endif        
        
      } else if (cmd == 200) {
        alpha = from_processing.asFloat[1];

      } else if (cmd == 201) {
        pid_refresh_rate = (int)from_processing.asFloat[1];
        set_pid_refresh_rate();
        
      }
    }
  }
}


void read_throttle()
{
  INPUT_THRUST = constrain(INPUT_THRUST, MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming
}

void read_setpoint()
{
  setpoint[YW] = INPUT_SETPOINT_YAW ;
  setpoint[AC] = INPUT_SETPOINT_ROLL ;
  setpoint[BD] = INPUT_SETPOINT_PITCH ;    
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}
///////////////////////////////////////////////////////////////////////

