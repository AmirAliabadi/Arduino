void process_off()
{
  #ifdef DEBUG      
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
//    Serial.print(F("#processing is off: "));    
//    log_data();
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

  float yw_reading = (int)(ypr[YW]*10.0 + .5)/10.0;

  if(last_yw == yw_reading) {
    system_check |= INIT_MPU_STABLE;
    
    yw_offset = yw_reading;
    ac_offset = ypr[AC];
    bd_offset = ypr[BD];
    
    ypr[YW] = 0;
    //ypr_last[YW] = 0;
    
    process = &attitude_process;
  } else {
    last_yw = yw_reading;
    process = &wait_for_stable;
    
  }
}

void arm_esc_process()
{
    read_throttle();
    init_esc();
      
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) {
      process = &wait_for_stable;    
    }
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void attitude_process()
{
  if( system_check & INIT_ESC_ARMED )
  {
    if(abs(ypr[AC]) > 45.0 || abs(ypr[BD]) > 45.0) 
    {
      disarm_esc();
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
      }     
#endif      

      return;
  }

  // Update the Stable PID input values
  input_ypr[YW] = ypr[YW];
  input_ypr[BD] = ypr[BD];
  input_ypr[AC] = ypr[AC];

  input_gyro[YW] = gyro.z*-1.0;
  input_gyro[BD] = gyro.y*-1.0;
  input_gyro[AC] = gyro.x;

  if(INPUT_THRUST > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) init_pid();        

    pid_stable[YW].Compute();  pid_rate[YW].Compute();
    pid_stable[BD].Compute();  pid_rate[BD].Compute();   
    pid_stable[AC].Compute();  pid_rate[AC].Compute(); 

    //////////////////////////////
    // Motor Mix Algorithm       //
    //////////////////////////////
    // compute the boom thrust  //
    v_ac = INPUT_THRUST - output_ypr[YW];
    v_bd = INPUT_THRUST + output_ypr[YW];

    // compute motor speeds
    va = MIN_ESC_CUTOFF + (v_ac - output_rate[AC]); // output_ypr output_rate
    vc = MIN_ESC_CUTOFF + (v_ac + output_rate[AC]); // output_ypr
    vb = MIN_ESC_CUTOFF + (v_bd - output_rate[BD]); // output_ypr
    vd = MIN_ESC_CUTOFF + (v_bd + output_rate[BD]); // output_ypr
    //
    ////////////////////////////////
  }
  else 
  {
    va = vb = vc = vd = MIN_ESC_SIGNAL;
    
    pid_reset(); //(MANUAL);
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
  }
#endif
  
}

//////////////////////////////////////////////////////////////////////
