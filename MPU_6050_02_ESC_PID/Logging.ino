#ifdef DEBUG 
void log_data(int va, int vc)
{
  Serial.print(log_line++);
  Serial.print("\t");
  
  Serial.print(thrust);
  Serial.print("\t");

  Serial.print(setpoint_ac, 2);
  Serial.print("\t");  

  for(int i=0;i <3; i++ ) {
    Serial.print(input_ypr[i], 2);
    Serial.print("\t");
  }
  
  for(int i=0;i <3; i++ ) {
    Serial.print(output_ypr[i], 2);
    Serial.print("\t");  
  }
    
  Serial.print(va);
  Serial.print("\t");
  Serial.print(vc);
  Serial.print("\t");
  
  Serial.print(ac_pid.pterm, 4);
  Serial.print("\t");
  Serial.print(ac_pid.iterm, 4);
  Serial.print("\t");
  Serial.print(ac_pid.dterm, 4);

  Serial.print("\t");
  Serial.print(ac_pid.GetKp(), 4);
  Serial.print("\t");
  Serial.print(ac_pid.GetKi(), 4);
  Serial.print("\t");
  Serial.print(ac_pid.GetKd(), 4);

//  Serial.print("\t");
//  Serial.print(mpu.getDLPFMode(), 4);

//  Serial.print("\t");
//  Serial.print(mpu.getDHPFMode(), 4);  
  
  Serial.println("");  
}

#endif
