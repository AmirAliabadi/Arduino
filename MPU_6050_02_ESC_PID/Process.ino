void process_off()
{
  #ifdef DEBUG      
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    Serial.print(F("#processing is off: "));    
    log_data();
  }
  #endif    
}

void wait_for_stable()
{
  if (millis() - last_log > 2000)
  {
    last_log = millis();
    process = &check_if_stable;  
  }
}

void check_if_stable()
{
  static float last_yw = -333.0;

#ifdef DEBUG    
  Serial.print(F("#waiting 4 SR "));
  Serial.println(abs(ypr[YW] - last_yw),2);
#endif

  if(last_yw == ypr[YW]) {
    system_check |= INIT_MPU_STABLE;
    yw_offset = ypr[YW];

    ypr[YW] = 0;
    ypr_last[YW] = 0;
    process = &attitude_process;
  } else {
    last_yw = ypr[YW];
    process = &wait_for_stable;
  }
}

void arm_esc_process()
{
    read_throttle();
    init_esc();
      
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) 
    {
      process = &wait_for_stable;    
    }
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void attitude_process()
{
  int v_ac = 0;
  int v_bd = 0;

  if( system_check & INIT_ESC_ARMED )
  {
    if(abs(ypr[AC]) > 45.0 || abs(ypr[BD]) > 45.0) 
    {
      disarm_esc();
      
#ifdef DEBUG       
      Serial.print(F("#esc off: "));
      log_data();     
#endif      

      process = &process_off;
      return;
    }
  }
  else 
  {

#ifdef DEBUG         
      if (millis() - last_log > LOG_FREQUENCY)
      {   
        last_log = millis();       
             
        Serial.print(F("#esc not ready : "));
        log_data(); 
      }     
#endif      

      return;
  }

  if(thrust > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) init_pid();        

    update_pid_settings();
    
    ac_pid.Compute(); bd_pid.Compute(); yw_pid.Compute();    

    //////////////////////////////////////////////////////
    // compute the boom velocity
    /*
    float v_ac = (abs(output_ypr[YW] - 100) / 100) * thrust;
    float v_bd = (   (output_ypr[YW] + 100) / 100) * thrust;
    
    // distribute the boom velocity to each boom motor
    float va = ((output_ypr[AC] + 100) / 100) * v_ac;
    float vb = ((output_ypr[BD] + 100) / 100) * v_bd;
    
    float vc = (abs((output_ypr[AC] - 100) / 100)) * v_ac;
    float vd = (abs((output_ypr[BD] - 100) / 100)) * v_bd;
    */
    //
    //////////////////////////////////////////////////////

    //////////////////////////////
    // Motor Mix Alorithm       //
    //////////////////////////////
    // compute the boom thrust  //
    v_ac = thrust - output_ypr[YW];
    v_bd = thrust + output_ypr[YW];

    // compute motor speeds
    va = MIN_ESC_SIGNAL + (v_ac + output_ypr[AC]);
    vc = MIN_ESC_SIGNAL + (v_ac - output_ypr[AC]);
    vb = MIN_ESC_SIGNAL + (v_bd + output_ypr[BD]);
    vd = MIN_ESC_SIGNAL + (v_bd - output_ypr[BD]);
    //
    ////////////////////////////////
  }
  else 
  {
    va = vb = vc = vd = MIN_ESC_SIGNAL;
    
    pid_off();
  }

  va = constrain(va, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vc = constrain(vc, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vb = constrain(vb, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vd = constrain(vd, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);


#ifdef DEBUG   
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
    //print_mpu_readings(mode,fifoBuffer);
#endif    
  }
  
}
//////////////////////////////////////////////////////////////////////
