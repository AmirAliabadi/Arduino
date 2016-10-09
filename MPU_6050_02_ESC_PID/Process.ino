void process_off()
{
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
  }
}

void wait_for_stable_process()
{
  if (millis() - last_log > 2000)
  {
    last_log = millis();
    log_data();    
    process = &check_if_stable_process;  
  }
}

void check_if_stable_process()
{
  static float last_yw = -333.0;

  Serial.print(F("#w 4 SR "));
  Serial.println(abs(ypr[YW] - last_yw),2);

  float yw_reading = (int)(ypr[YW]*10.0 + .5)/10.0;

  if(last_yw == yw_reading) {
    system_check |= INIT_MPU_STABLE;
    
    yw_offset = yw_reading;
    ac_offset = 0.0; //ypr[AC];
    bd_offset = 0.0; //ypr[BD];

    ypr_last[YW] = 0.0;
    ypr_last[AC] = 0.0;
    ypr_last[BD] = 0.0;

    Serial.print(F("#stable: "));
    Serial.print(yw_offset);
    Serial.print(F(" "));
    Serial.print(ac_offset);
    Serial.print(F(" "));
    Serial.println(bd_offset);
    
    process = &attitude_process;
  } else {
    last_yw = yw_reading;
    process = &wait_for_stable_process;
    
  }
}

void arm_esc_process()
{
    init_esc();
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) {
      process = &wait_for_stable_process; //attitude_process; //wait_for_stable;    
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
      // ESC is not armed
      if (millis() - last_log > LOG_FREQUENCY)
      {   
        last_log = millis(); 
        log_data();
      }     
      return;
  }

  // Update the Stable PID input values
  input_ypr[YW] = ypr[YW];
  input_ypr[AC] = ypr[AC];
  input_ypr[BD] = ypr[BD];  

  input_gyro[YW] = gyro.z*-1.0;
  input_gyro[AC] = gyro.x;
  input_gyro[BD] = gyro.y*-1.0;  

  if(INPUT_THRUST > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) init_pid();        

    pid_stable[YW].Compute();  pid_rate[YW].Compute();
    pid_stable[AC].Compute();  pid_rate[AC].Compute(); 
    pid_stable[BD].Compute();  pid_rate[BD].Compute();       

    //////////////////////////////
    // Motor Mix Algorithm       //
    //////////////////////////////
    // compute the boom thrust  //
    v_ac = INPUT_THRUST - output_ypr[YW]; // output_rate[YW];
    v_bd = INPUT_THRUST + output_ypr[YW]; // output_rate[YW];

    // compute motor speeds
    va = MIN_ESC_CUTOFF + (v_ac - output_rate[AC]); // output_ypr output_rate
    vc = MIN_ESC_CUTOFF + (v_ac + output_rate[AC]); // output_ypr
    vb = MIN_ESC_CUTOFF + (v_bd - output_rate[BD]); // output_ypr
    vd = MIN_ESC_CUTOFF + (v_bd + output_rate[BD]); // output_ypr
    //
    ////////////////////////////////

    va = constrain(va, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vc = constrain(vc, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vb = constrain(vb, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vd = constrain(vd, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);    
  }
  else 
  {
    va = vb = vc = vd = MIN_ESC_SIGNAL;
    
    pid_reset(); //(MANUAL);
  }

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);

  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
  }
}

//////////////////////////////////////////////////////////////////////
