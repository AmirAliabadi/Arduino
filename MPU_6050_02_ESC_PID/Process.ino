void process_off()
{
    Serial.print("#processing is off...");
}

void wait_for_stable()
{
  Serial.print("#waiting fo stable readings...");
  
  system_check |= INIT_MPU_STABLE;
  
  process = &balance_process;
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void balance_process()
{
  int va = MIN_ESC_SIGNAL;
  int vb = MIN_ESC_SIGNAL;
  int vc = MIN_ESC_SIGNAL;
  int vd = MIN_ESC_SIGNAL;
  
  int v_ac = 0;
  int v_bd = 0;
  
  if( !(system_check & INIT_ESC_ARMED) )
  {
    if(abs(ypr[AC]) > 45.0) 
    {
      disarm_esc();

      thrust = 0;
     
      Serial.print("#esc disarmed : ");
      log_data(0.0,0.0);     

      process = &process_off;
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

  if(thrust > MIN_INPUT_THRUST) {

    if( system_check & !INIT_PID_ON ) init_pid();        

    ////////////////////////////////////////////////////
    // Reset of PID when setpoint changes
    if( setpoint_changed & SETPOINT_CHANGED_AC ) {ac_pid.Reset();}
    if( setpoint_changed & SETPOINT_CHANGED_BD ) {bd_pid.Reset();}
    if( setpoint_changed & SETPOINT_CHANGED_YW ) {yw_pid.Reset();}
    
    setpoint_changed = SETPOINT_UNCHANGED;
    //
    ////////////////////////////////////////////////////

    //////////////////////////////////////////////////
    // adaptive PID settings
    int i = 0;
    if( abs(setpoint_ac - input_ypr[AC]) > 5 ) i = 1;
    ac_pid.SetTunings(user_inputs.pid_ac[i].kp, user_inputs.pid_ac[i].ki, user_inputs.pid_ac[i].kd);

    i = 0;
    if( abs(setpoint_bd - input_ypr[BD]) > 5 ) i = 1;
    bd_pid.SetTunings(user_inputs.pid_bd[i].kp, user_inputs.pid_bd[i].ki, user_inputs.pid_bd[i].kd);

    i = 0;
    if( abs(setpoint_yw - input_ypr[YW]) > 5 ) i = 1;
    yw_pid.SetTunings(user_inputs.pid_yw[i].kp, user_inputs.pid_yw[i].ki, user_inputs.pid_yw[i].kd);      
    //
    /////////////////////////////////////////////////
    
    ac_pid.Compute();
    bd_pid.Compute();
    yw_pid.Compute();    

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

    /////////////////////////////
    // compute the boom thrust //
    v_ac = thrust;
    v_bd = thrust;

    ////////////////////////
    // Motor Mix Alorithm //
    va = MIN_ESC_SIGNAL + (v_ac + output_ac);
    vc = MIN_ESC_SIGNAL + (v_ac - output_ac);
    vb = MIN_ESC_SIGNAL + (v_bd + output_bd);
    vd = MIN_ESC_SIGNAL + (v_bd - output_bd);

    va = constrain(va, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
    vc = constrain(vc, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
    vb = constrain(vb, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
    vd = constrain(vd, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
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
  //esc_b.writeMicroseconds(vb);
  //esc_d.writeMicroseconds(vd);

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
