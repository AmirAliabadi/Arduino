#ifdef DEBUG 
void log_data(int va, int vc)
{
  Serial.print(log_line++);
  Serial.print("\t");
  
  Serial.print(thrust);
  Serial.print("\t");
  
  Serial.print(input_ypr[AC], 2);
  Serial.print("\t");
  
  Serial.print(setpoint_ac, 2);
  Serial.print("\t");  
    
  Serial.print(output_ac, 2);
  Serial.print("\t");
  
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

void log_data2(int va, int vc)
{
//  plotter_packet[0] = thrust;
//  plotter_packet[1] = input_ypr[AC];
//  plotter_packet[2] = setpoint_ac;  
//  plotter_packet[3] = output_ac;  
//  plotter_packet[4] = va;  
//  plotter_packet[5] = vc;    
//  plotter_packet[6] = ac_pid.pterm;  
//  plotter_packet[7] = ac_pid.iterm;  
//  plotter_packet[8] = ac_pid.dterm;  
//  plotter_packet[9] = ac_pid.GetKp();  
//  plotter_packet[10] = ac_pid.GetKi();  
//  plotter_packet[11] = ac_pid.GetKd(); 
//  Serial.write(plotter_packet, sizeof(plotter_packet)/sizeof(float));
  

  Serial.print(thrust); 
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

  Serial.print("\r\n");  
}


void log_data(float* input_values)
{
  Serial.print(input_values[0]);
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

#endif
