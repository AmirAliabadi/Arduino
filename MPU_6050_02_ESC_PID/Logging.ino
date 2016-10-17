#ifdef DEBUG 

/********************************************
 * Serial Communication functions / helpers
 ********************************************/

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
        pid_stable[AC].SetControllerDirection( (int)from_processing.asFloat[1] == 0.0 ? DIRECT : REVERSE );
        
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

void SerialSend_A(byte mode)
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/r _ va _ vb _ vc _ vd _ alpha

  Serial.print(F("A "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(output_ypr[mode]);   Serial.print(F(" "));  
  Serial.print(output_rate[mode]);  Serial.print(F(" "));

  Serial.print(pid_stable[mode].GetKp(), 4);   Serial.print(F(" "));
  Serial.print(pid_stable[mode].GetKi(), 4);   Serial.print(F(" "));
  Serial.print(pid_stable[mode].GetKd(), 4);   Serial.print(F(" "));

#ifdef CASCADE_PIDS    
  Serial.print(pid_rate[mode].GetKp(), 4);   Serial.print(F(" "));
  Serial.print(pid_rate[mode].GetKi(), 4);   Serial.print(F(" "));
  Serial.print(pid_rate[mode].GetKd(), 4);   Serial.print(F(" "));
#else
  Serial.print(0.0000, 4);   Serial.print(F(" "));
  Serial.print(0.0000, 4);   Serial.print(F(" "));
  Serial.print(0.0000, 4);   Serial.print(F(" "));
#endif

  Serial.print(pid_stable[mode].pterm, 4);   Serial.print(F(" "));
  Serial.print(pid_stable[mode].iterm, 4);   Serial.print(F(" "));
  Serial.print(pid_stable[mode].dterm, 4);   Serial.print(F(" "));

#ifdef CASCADE_PIDS    
  Serial.print(pid_rate[mode].pterm, 4);   Serial.print(F(" "));
  Serial.print(pid_rate[mode].iterm, 4);   Serial.print(F(" "));
  Serial.print(pid_rate[mode].dterm, 4);   Serial.print(F(" "));
#else  
  Serial.print(0.0000, 4);   Serial.print(F(" "));
  Serial.print(0.0000, 4);   Serial.print(F(" "));
  Serial.print(0.0000, 4);   Serial.print(F(" "));
#endif

  Serial.println(F("E"));

  send_serial = &SerialSend_B;
    
}

void SerialSend_B(byte mode)
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/r _ va _ vb _ vc _ vd _ alpha

  Serial.print(F("B "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
  Serial.print(setpoint[mode]);   Serial.print(F(" "));  

  Serial.print(input_gyro[mode]); Serial.print(F(" "));
  Serial.print(input_ypr[mode]);  Serial.print(F(" "));    

  if(pid_stable[mode].GetMode()==AUTOMATIC) Serial.print(F("A"));
  else Serial.print(F("M"));  
  Serial.print(F(" "));
  if(pid_stable[mode].GetDirection()==DIRECT) Serial.print(F("D"));
  else Serial.print(F("R"));
  Serial.print(F(" "));

#ifdef CASCADE_PIDS 
  if(pid_rate[mode].GetDirection()==DIRECT) Serial.print(F("D"));
  else Serial.print(F("R"));  
#else  
  Serial.print(F("D"));  
#endif  

  Serial.print(F(" "));
  
  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));

  Serial.print(alpha); Serial.print(F(" "));  
  
  Serial.println(F("E"));

  send_serial = &SerialSend_A;
    
}

void log_data()
{
  if( aserial_data_mode == 0 ) send_serial(AC);
  else if( aserial_data_mode == 1 ) send_serial(BD);
  else if( aserial_data_mode == 2 ) send_serial(YW); 
}

#endif
