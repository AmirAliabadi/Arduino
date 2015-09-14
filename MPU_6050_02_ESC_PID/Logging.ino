
void log_pid_tuning(double kp, double ki, double kd)
{
  Serial.print("pid_tuning: ");
  Serial.print(kp,4);
  Serial.print("\t");
  Serial.print(ki,4);
  Serial.print("\t");
  Serial.print(kd,4);
  Serial.println("\t");
}

void log_data(float va, float vc)
{
  Serial.print(thrust, 4);
  Serial.print("\t");
  
  Serial.print(input_ypr[AC], 2);
  Serial.print("\t");
  
  Serial.print(setpoint_ac, 2);
  Serial.print("\t");  
    
  Serial.print(output_ac, 2);
  Serial.print("\t");
  
  Serial.print(va, 4);
  Serial.print("\t");
  Serial.print(vc, 4);
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

  Serial.print("\t");
  Serial.print(mpu.getDLPFMode(), 4);

  Serial.print("\t");
  Serial.print(mpu.getDHPFMode(), 4);  
  
  Serial.println("");  
}

void log_data2(float va, float vc)
{
  Serial.print(thrust, 2); 
  Serial.print(" ");
  Serial.print(input_ypr[AC], 2);
  Serial.print(" ");
  Serial.print(setpoint_ac, 2);
  Serial.print(" ");    
  Serial.print(output_ac, 2);
  Serial.print(" ");
//  Serial.print(va, 2);
//  Serial.print(" ");
//  Serial.print(vc, 2);
//  Serial.print(" ");
  Serial.print(ac_pid.pterm, 4);
  Serial.print(" ");
  Serial.print(ac_pid.iterm, 4);
  Serial.print(" ");
  Serial.print(ac_pid.dterm, 4);
  Serial.print(" ");
//  Serial.print(ac_pid.GetKp());
//  Serial.print(" ");
//  Serial.print(ac_pid.GetKi());
//  Serial.print(" ");
//  Serial.print(ac_pid.GetKd());
//  Serial.print(" ");  

  Serial.print("\r");  
}


void log_data(float* input_values)
{
  Serial.print(input_values[0], 4);
  Serial.print("\t");
  Serial.print(input_values[1], 4);
  Serial.print("\t");
  Serial.print(input_values[2], 4);
  Serial.print("\t");
  Serial.print(input_values[3], 4);
  Serial.print("\t");  
  Serial.print(input_values[4], 4);
  Serial.print("\t");    
  Serial.println("\r\n");
}

