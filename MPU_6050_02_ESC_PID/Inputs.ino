
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
    
    input_values[k++] = atof(number);
  }
}

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
void read_throttle()
{
  //input_values[0] = map(analogRead(THROTTLE_PIN),0,644,0.0,MAX_INPUT_THRUST);

  if( system_check & INIT_ESC_ARMED ) 
  {
    thrust = constrain(input_values[0], MIN_INPUT_THRUST, MAX_INPUT_THRUST*0.90);  // todo: determine max when arming
  }
  else
  {
    thrust = constrain(input_values[0], MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming 

    if( thrust == 0.0 )
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
  voltage = INPUT_VOLTAGE_LEVEL;
}

void read_pid_tunings(int type)
{
  //float p = map(analogRead(Kp_PIN),0,644,0.0,100000.0)/2000.0;
  //float i = 0.0; // map(analogRead(Ki_PIN),0,644,0.0,100000.0)/50000.0;
  //float d = map(analogRead(Kd_PIN),0,644,0.0,100000.0)/3000.0;

  //p = ((int)(p*100.0+.5))/100.0;
  //i = ((int)(i*100.0+.5))/100.0;
  //d = ((int)(d*100.0+.5))/100.0;

  //input_values[2+(type*3)] = p;
  //input_values[3+(type*3)] = i;
  //input_values[4+(type*3)] = d;  
  
  pid_xx_kp[type] = input_values[1+(type*3)];
  pid_xx_ki[type] = input_values[2+(type*3)];
  pid_xx_kd[type] = input_values[3+(type*3)];

  pid_yw_kp[type] = 2.5; //pid_xx_kp[type]; // 15.0 workd but might have been too much
  pid_yw_ki[type] = 0.0; //pid_xx_ki[type];
  pid_yw_kd[type] = 0.75; //pid_xx_kd[type]; // 3.7 with the 15.0
}
///////////////////////////////////////////////////////////////////////
