//////////////////////////////////////////////////////////////////////
// main autopilot routine
void balance_process()
{
  if(esc_ready)
  {
    if(abs(ypr[AC]) > 45.0) 
    {
      disarm_esc();
      input_values[0] = 0; // throttle to zero      
     
      Serial.print("#esc disarmed : ");
      log_data(0.0,0.0);              
      return;
    }
  }
  else 
  {
      if (millis() - last_log > LOG_FREQUENCY)
      {   
        last_log = millis();       
             
        Serial.print("#esc not ready : ");
        log_data(0.0,0.0); 
      }     

      return;
  }
  
  thrust = read_throttle();

  if(thrust > MIN_INPUT_THRUST) {

    setpoint_ac = read_setpoint_ac();
    //setpoint_bd = read_setpoint_bd();  
    //setpoint_yw = read_setpoint_yw();  

    ///
    // TODO: testing out reset of PID when setpoint changes
    if( last_setpoint_ac != setpoint_ac ) {pid_off();}
    //last_setpoint_bd = 0.0;
    //last_setpoint_yw = 0.0;    
    ///

    if(!pid_ready) init_pid();

    kp = read_kp(); ki = read_ki(); kd = read_kd();
    
    yw_pid.SetTunings(kp, ki, kd);  
    ac_pid.SetTunings(kp, ki, kd);
    bd_pid.SetTunings(kp, ki, kd);
    
    yw_pid.Compute();
    ac_pid.Compute();
    bd_pid.Compute();

    //////////////////////////////////////////////////////
    // compute the boom velocity
    /*
    float v_ac = (abs(output_yw - 100) / 100) * thrust;
    float v_bd = (   (output_yw + 100) / 100) * thrust;
    
    // distribute the boom velocity to each boom motor
    float va = ((output_ac + 100) / 100) * v_ac;
    float vb = ((output_bd + 100) / 100) * v_bd;
    
    float vc = (abs((output_ac - 100) / 100)) * v_ac;
    float vd = (abs((output_bd - 100) / 100)) * v_bd;
    */
    //
    //////////////////////////////////////////////////////
    
    v_ac = thrust;
    v_bd = thrust;

    va = MIN_ESC_SIGNAL + (v_ac + output_ac);
    vc = MIN_ESC_SIGNAL + (v_ac - output_ac);
    vb = MIN_ESC_SIGNAL + (v_bd + output_bd);
    vd = MIN_ESC_SIGNAL + (v_bd - output_bd);

    va = va <= MIN_ESC_SIGNAL ? MIN_ESC_SIGNAL : va;
    vc = vc <= MIN_ESC_SIGNAL ? MIN_ESC_SIGNAL : vc;
    vb = vb <= MIN_ESC_SIGNAL ? MIN_ESC_SIGNAL : vb;
    vd = vd <= MIN_ESC_SIGNAL ? MIN_ESC_SIGNAL : vd;
    
    va = va > MAX_ESC_SIGNAL ? MAX_ESC_SIGNAL : va;
    vc = vc > MAX_ESC_SIGNAL ? MAX_ESC_SIGNAL : vc;
    vb = vb > MAX_ESC_SIGNAL ? MAX_ESC_SIGNAL : vb;
    vd = vd > MAX_ESC_SIGNAL ? MAX_ESC_SIGNAL : vd;  
  }
  else 
  {
    va = MIN_ESC_SIGNAL;
    vb = MIN_ESC_SIGNAL;
    vc = MIN_ESC_SIGNAL;
    vd = MIN_ESC_SIGNAL;    
    
    pid_off();
  }

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  //esc_b.writeMicroseconds(b);
  //esc_d.writeMicroseconds(d);

  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    
#ifdef DEBUG    
    
    //log_pid_tuning(kp,ki,kd);
    log_data(va, vc);
    // log_graphing_data(va,vc);
    //log_data(input_values);
    //plot(va,vc);
    //log_data2(va, vc);
        
    //print_mpu_readings(mode,fifoBuffer);
#endif


    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  }
  
}
//////////////////////////////////////////////////////////////////////
