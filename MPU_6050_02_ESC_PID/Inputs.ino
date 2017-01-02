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
        
      } else if (cmd == 200) {
        alpha = from_processing.asFloat[1];

      } 
    }
  }
}

unsigned long foo = 0;
void read_throttle()
{
  if( system_check & INIT_ESC_ARMED ) {
//    INPUT_THRUST = constrain(INPUT_THRUST, MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming
//    return;
    
    if( ppm_read && ppm_sync ) {
      cli();
      foo = ppm_channels[THROTTLE_CHANNEL] ;    
      sei();
  
      if( foo < 1490 ) {
          if( foo < 1100 ) {
            INPUT_THRUST = 0;
          } else if( foo < 1300 ) {
            if( INPUT_THRUST >= 10 ) INPUT_THRUST -= 10;
            else INPUT_THRUST = 0;
          } else if( foo < 1410 ) {
            if( INPUT_THRUST >= 1 ) INPUT_THRUST -= 1;
            else INPUT_THRUST = 0;
          } 
      } else if ( foo > 1510 ) {
        if ( foo > 1910 ) {
          if( INPUT_THRUST < 990 ) INPUT_THRUST += 10 ;          
        } else if ( foo > 1610  ) {
          if( INPUT_THRUST < 990 ) INPUT_THRUST += 1 ;          
        }
      } 
    }
  }
}

void read_setpoint()
{
  setpoint[YAW] = INPUT_SETPOINT_YAW ;
  setpoint[AC] = INPUT_SETPOINT_ROLL ;
  setpoint[BD] = INPUT_SETPOINT_PITCH ;    
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}
///////////////////////////////////////////////////////////////////////

