//////////////////////////////////////////////////////////////////////
// main autopilot routine
void balance_process()
{
  if(esc_ready)
  {
    if(abs(ypr[AC]) > 45.0) 
    {
      disarm_esc();
     
      Serial.print("#esc disarmed : ");
      log_data(0.0,0.0);              
      return;
    }
  }
  else 
  {
      if (millis() - mpu_debug_info_hz > LOG_FREQUENCY)
      {   
        mpu_debug_info_hz = millis();       
             
        Serial.print("#esc not ready : ");
        log_data(0.0,0.0); 
      }     

      return;
  }
  
  thrust = read_throttle();

  float va = MIN_SIGNAL;
  float vb = MIN_SIGNAL;
  float vc = MIN_SIGNAL;
  float vd = MIN_SIGNAL;

  setpoint_ac = read_setpoint_ac();
  //setpoint_bd = read_setpoint_bd();  
  //setpoint_yw = read_setpoint_yw();    
  
  if(thrust > NEUTRAL_THRUST) {

    if(!pid_ready) init_pid();

    double kp = read_kp();
    double ki = read_ki();
    double kd = read_kd();
    
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
    
    float v_ac = thrust;
    float v_bd = thrust;

    va = MIN_SIGNAL + (v_ac + output_ac);
    vc = MIN_SIGNAL + (v_ac - output_ac);
    vb = MIN_SIGNAL + (v_bd + output_bd);
    vd = MIN_SIGNAL + (v_bd - output_bd);

    va = va <= MIN_THRUST ? MIN_SIGNAL : va;
    vc = vc <= MIN_THRUST ? MIN_SIGNAL : vc;
    vb = vb <= MIN_THRUST ? MIN_SIGNAL : vb;
    vd = vd <= MIN_THRUST ? MIN_SIGNAL : vd;
    
    va = va > MAX_THRUST ? MAX_THRUST : va;
    vc = vc > MAX_THRUST ? MAX_THRUST : vc;
    vb = vb > MAX_THRUST ? MAX_THRUST : vb;
    vd = vd > MAX_THRUST ? MAX_THRUST : vd;  
  }
  else 
  {
    pid_off();
  }

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  //esc_b.writeMicroseconds(b);
  //esc_d.writeMicroseconds(d);

  if (millis() - mpu_debug_info_hz > LOG_FREQUENCY)
  {
    mpu_debug_info_hz = millis();
    
#ifdef DEBUG    
    
    //log_pid_tuning(kp,ki,kd);
    //log_data(va, vc);
    // log_graphing_data(va,vc);
    //log_data(input_values);
    //plot(va,vc);
    log_data2(va, vc);
        
    //print_mpu_readings(mode,fifoBuffer);
#endif


    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  }
  
}
//////////////////////////////////////////////////////////////////////
