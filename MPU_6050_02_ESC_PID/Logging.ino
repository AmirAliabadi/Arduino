
void SerialSend_A(byte mode)
{
  Serial.print(F("A "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(attitude_correction[mode]);   Serial.print(F(" "));  // this is the stable pid output (measured angle - setpoint) => desired acceleration
  Serial.print(acceleration_correction[mode]);  Serial.print(F(" "));  // this is the rate pid output (desired acceleration - current acceleration) => motor output

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
  Serial.print(F("B "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
  Serial.print(setpoint[mode]);   Serial.print(F(" "));  

  Serial.print(current_acceleration[mode]); Serial.print(F(" "));  // this is the actual acceleration being read by the MPU
  Serial.print(current_attitude[mode]);     Serial.print(F(" "));  // this is the actual angle being read by the MPU  

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

  Serial.print(pid_refresh_rate); Serial.print(F(" "));  
  
  Serial.println(F("E"));

  send_serial = &SerialSend_A;
    
}

void log_data()
{
  if( aserial_data_mode == 0 ) send_serial(AC);
  else if( aserial_data_mode == 1 ) send_serial(BD);
  else if( aserial_data_mode == 2 ) send_serial(YW); 
}

