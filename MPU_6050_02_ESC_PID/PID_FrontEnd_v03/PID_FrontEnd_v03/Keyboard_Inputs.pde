
void keyPressed() {
    switch(key) {
      case ' ':
        cur_throttle = 0.0;
        Send_To_Arduino2(0.0, cur_throttle);         
        break;
      case '`':
        cur_throttle = cur_throttle/2.0;
        Send_To_Arduino2(0.0, cur_throttle);         
        break;        
      case 'w':
        outputFileName = "c:\\temp\\drone_" + year()+month()+day()+hour()+minute()+second()+".txt";
        output = createWriter(outputFileName);
        output.println("t S selected.pot.tuning_serial.data.mode INPUT_THRUST SETPOINT inputgyro inputypr outputypr outputrate kp ki kd rKp rKi rKd A/M D/R rD/R va vb vc vd E");        
        break;
      case 'e':
        output.flush(); // Writes the remaining data to the file
        output.close(); // Finishes the file
        outputFileName = "";        
        break;
        
      case 'a':
        if(cur_throttle <= 1000.0) cur_throttle += 10.0;
        Send_To_Arduino2(0.0, cur_throttle);        
        break;
      case 'z':
        if(cur_throttle > 0.0) cur_throttle -= 20.0;
        Send_To_Arduino2(0.0, cur_throttle);           
        break;        
        
        ///
        
      case '7':
        if( kp < 10.0) kp += 0.005;
        Send_To_Arduino2( (i_serial_data_mode == 2 ? 7.0 : 1.0), kp);        
        break;
      case 'u':
        if( kp > 0.0 ) kp -= 0.005; 
        Send_To_Arduino2( (i_serial_data_mode == 2 ? 7.0 : 1.0), kp);        
        break;        
        
      case '8':
        if( ki < 10.0 ) ki += 0.005; 
        Send_To_Arduino2((i_serial_data_mode == 2 ? 8.0 : 2.0), ki);        
        break;
      case 'i':
        if( ki > 0.0 ) ki -= 0.005;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 8.0 : 2.0), ki);        
        break;        
        
      case '9':  
        if( kd < 10.0 ) kd += 0.005; 
        Send_To_Arduino2((i_serial_data_mode == 2 ? 9.0 : 3.0), kd);        
        break;
      case 'o':
        if( kd > 0.0 ) kd -= 0.005;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 9.0 : 3.0), kd);        
        break;      
        
        ////
        
      case 'j':
        if( krp < 10.0 ) krp += 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 10.0 : 4.0), krp);        
        break;
      case 'm':
        if( krp > 0.0 ) krp  -= 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 10.0 : 4.0), krp);        
        break;        
        
      case 'k':
        if( kri < 10.0 ) kri += 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 11.0 : 5.0), kri);        
        break;
      case ',':
        if( kri > 0.0 ) kri -= 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 11.0 : 5.0), kri);        
        break;        
        
      case 'l':
        if( krd < 10.0 ) krd += 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 12.0 : 6.0), krd);        
        break;
      case '.':
        if( krd > 0.0 ) krd -= 0.001;
        Send_To_Arduino2((i_serial_data_mode == 2 ? 12.0 : 6.0), krd);        
        break;
      
      case '5':
        alpha += 0.01;
        Send_To_Arduino2(200.0, alpha);
        break;
      case 't':
        if( alpha > 0 ) alpha -= 0.01;
        Send_To_Arduino2(200.0, alpha);
        break;
        
      case '4':
        pid_refresh_rate += 1.0;
        PidRefreshIntervalLabel.setValue(Float.toString(pid_refresh_rate));
        Send_To_Arduino2(201.0, pid_refresh_rate);
        break;
        
      case 'r':
        if( pid_refresh_rate > 0 ) pid_refresh_rate -= 1.0;
        PidRefreshIntervalLabel.setValue(Float.toString(pid_refresh_rate));        
        Send_To_Arduino2(201.0, pid_refresh_rate);
        break;        
        
        
    }
    last_throttle_position = cur_throttle;
}