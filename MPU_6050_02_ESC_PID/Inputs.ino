/*
void readCsvToVector() 
{
  byte len;
  if (Serial.available() <= 0)
    return ;
   
  len = Serial.available();
  
  char stream[80];
  char number[10];
  
  // Read the message
  for (byte i = 0; i < len; ++i) {
    stream[i] = Serial.read();
  }
  
  byte i = 0, j = 0, k = 0;

  while ( i < len && k < 12 ) {
    while (stream[i] != ',' && i < len) {
      number[j] = stream[i];
      ++i;
      ++j;
    }
    ++i;
    number[j] = '\0';
    j = 0;

    switch(k) {
      case 0:
          INPUT_THRUST = atof(number);
          break;
      case 1:
          INPUT_STB_PID_P = atof(number);        
          break;
      case 2:      
          INPUT_RAT_PID_P = atof(number);        
          break;
    }
    k++;
  }
}
*/

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
void read_throttle()
{
  if( selected_pot_tuning == 0 ) // STABLE PID tuning P value only...
  {  
    float v = analogRead(Kd_PIN);
    if(v < 50) INPUT_THRUST = 0;
    
    // INPUT_THRUST = map(analogRead(THROTTLE_PIN),0,644,0.0,MAX_INPUT_THRUST);
    v = analogRead(THROTTLE_PIN);
    if(v < 50) INPUT_THRUST -= 0.15;
    else if( v > 900) INPUT_THRUST += 0.1;

    if( system_check & INIT_ESC_ARMED ) 
    {
      INPUT_THRUST = constrain(INPUT_THRUST, MIN_INPUT_THRUST, MAX_INPUT_THRUST*0.90);  // todo: determine max when arming
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
}

void read_setpoint(int type)
{
  if( selected_pot_tuning == 0 ) // STABLE PID tuning P value only...
  {   
    float v = analogRead(Kd_PIN);  
    if(v < 50) input_values[11+type] = 0;  
    else if(v > 900) input_values[11+type] = 0;
    
    v = analogRead(Ki_PIN);
    if(v < 50) input_values[11+type] -= 0.1;
    else if( v > 900) input_values[11+type] += 0.1;

    input_values[11+type] = constrain(input_values[11+type],-45.0,45.0);
    
    if( setpoint[type] != input_values[11+type] ) 
    {  
      setpoint_changed    |= SETPOINT_CHANGED_AC;
      
      last_setpoint[type] = setpoint[type]; 
      setpoint[type]      = input_values[11+type];
    }
  }
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}

void read_pid_tunings(int type)
{
  if( selected_pot_tuning == 1 ) // STABLE PID tuning P value only...
  {
    float v = analogRead(Kp_PIN);
    if(v < 50) INPUT_STB_PID_P -= 0.001;
    else if( v > 900) INPUT_STB_PID_P += 0.001;
    INPUT_STB_PID_P = constrain(INPUT_STB_PID_P, 0.0, 20.0);
  
    v = analogRead(Ki_PIN);
    if(v < 50) INPUT_STB_PID_I -= 0.001;
    else if( v > 900) INPUT_STB_PID_I += 0.001;
    INPUT_STB_PID_I = constrain(INPUT_STB_PID_I, 0.0, 20.0);  

    v = analogRead(Kd_PIN);
    if(v < 50) INPUT_STB_PID_D -= 0.001;
    else if( v > 900) INPUT_STB_PID_D += 0.001;
    INPUT_STB_PID_D = constrain(INPUT_STB_PID_D, 0.0, 20.0);      
    
  } else if( selected_pot_tuning == 2 ) {
    float v = analogRead(Kp_PIN);
    if(v < 50) INPUT_RAT_PID_P -= 0.001;
    else if( v > 900) INPUT_RAT_PID_P += 0.001;
    INPUT_RAT_PID_P = constrain(INPUT_RAT_PID_P, 0.0, 20.0);
  
    v = analogRead(Ki_PIN);
    if(v < 50) INPUT_RAT_PID_I -= 0.001;
    else if( v > 900) INPUT_RAT_PID_I += 0.001;
    INPUT_RAT_PID_I = constrain(INPUT_RAT_PID_I, 0.0, 20.0);  

    v = analogRead(Kd_PIN);
    if(v < 50) INPUT_RAT_PID_D -= 0.001;
    else if( v > 900) INPUT_RAT_PID_D += 0.001;
    INPUT_RAT_PID_D = constrain(INPUT_RAT_PID_D, 0.0, 20.0);    
  }
  
}
///////////////////////////////////////////////////////////////////////
