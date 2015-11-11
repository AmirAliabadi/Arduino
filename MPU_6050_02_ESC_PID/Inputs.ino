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
  INPUT_THRUST = map(analogRead(THROTTLE_PIN),0,644,0.0,MAX_INPUT_THRUST);

  if( INPUT_THRUST < 5.0 ) INPUT_THRUST = 0.0;

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

void read_setpoint(int type)
{
  if( setpoint[type] != input_values[1] ) 
  {  
    setpoint_changed    |= SETPOINT_CHANGED_AC;
    last_setpoint[type] = setpoint[type]; 
    setpoint[type]      = input_values[1];
  }
}

void read_battery_voltage() 
{
  INPUT_VOLTAGE_LEVEL = 12.6;
}

void read_pid_tunings(int type)
{
  //float p = map(analogRead(Kp_PIN),0,644,0.0,100000.0)/2000.0;
  //float i = 0.0; // map(analogRead(Ki_PIN),0,644,0.0,100000.0)/50000.0;
  //float d = map(analogRead(Kd_PIN),0,644,0.0,100000.0)/3000.0;

  //float p_stab = map(analogRead(Kp_PIN),0,644,0.0,1000000.0)/200000.0;
  //float p_rate = map(analogRead(Ki_PIN),0,644,0.0,1000000.0)/200000.0;
  //float kd = map(analogRead(Ki_PIN),0,644,0.0,1000000.0)/400000.0;

  if( selected_pid_tuning == 0 ) 
  {
    float v = analogRead(Kp_PIN);
    if(v < 50) INPUT_STB_PID_P -= 0.001;
    else if( v > 900) INPUT_STB_PID_P += 0.001;
    INPUT_STB_PID_P = constrain(INPUT_STB_PID_P, 0.0, 20.0);
  
    v = analogRead(Ki_PIN);
    if(v < 50) INPUT_STB_PID_D -= 0.0001;
    else if( v > 900) INPUT_STB_PID_D += 0.001;
    INPUT_STB_PID_D = constrain(INPUT_STB_PID_D, 0.0, 20.0);  
  } else {
    float v = analogRead(Kp_PIN);
    if(v < 50) INPUT_RAT_PID_P -= 0.001;
    else if( v > 900) INPUT_RAT_PID_P += 0.001;
    INPUT_RAT_PID_P = constrain(INPUT_RAT_PID_P, 0.0, 20.0);
  
    v = analogRead(Ki_PIN);
    if(v < 5) INPUT_RAT_PID_D -= 0.0001;
    else if( v > 900) INPUT_RAT_PID_D += 0.0001;
    INPUT_RAT_PID_D = constrain(INPUT_RAT_PID_D, 0.0, 20.0);   
  }

  //p = ((int)(p*100.0+.5))/100.0;
  //i = ((int)(i*100.0+.5))/100.0;
  //d = ((int)(d*100.0+.5))/100.0;

  //input_values[2+(type*3)] = p;
  //input_values[3+(type*3)] = i;
  //input_values[4+(type*3)] = d;  

  //INPUT_STB_PID_P   = p_stab;
  //// INPUT_STB_PID_I = xxx
  //INPUT_STB_PID_D   = kd;

  //INPUT_RAT_PID_P   = p_stab;
  // INPUT_STB_PID_I = xxx
  //INPUT_RAT_PID_D   = kd;  
  
  //INPUT_RAT_PID_P = p_rate;

  
}
///////////////////////////////////////////////////////////////////////
