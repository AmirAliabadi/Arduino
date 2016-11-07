
void wait_for_stable_process()
{
  if (millis() - stable_check_loop > 2000)
  {
    stable_check_loop = millis();
    process = &check_if_stable_process;  
  }
}

void check_if_stable_process()
{
  static float last_yw = -333.0;

  Serial.print(F("#SR "));
  Serial.println(abs(ypr[YW] - last_yw),2);

  float yw_reading = (float)((int)(ypr[YW]*10.0 + .5))/10.0;
  
  if(last_yw == yw_reading) 
  {
    yw_offset = yw_reading;
    ac_offset = ypr[AC];
    bd_offset = ypr[BD];

    Serial.print(F("#stb: "));
    Serial.print(yw_offset);
    Serial.print(F(" "));
    Serial.print(ac_offset);
    Serial.print(F(" "));
    Serial.println(bd_offset);

    //ac_offset = 0;
    //bd_offset = 0;

    system_check |= INIT_MPU_STABLE;

    blink_pattern_3
    process = &arm_esc_process;
    
  } else {
    last_yw = yw_reading;
    process = &wait_for_stable_process;
    
  }
}

void arm_esc_process()
{
    init_esc();
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) {
      blink_pattern_1      
      process = &attitude_process; 
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
      blink_pattern_5      
      disarm_esc();
      process = &do_log;
      return;
    }
  }

  // Update the Stable PID input values
  // Angle reading
  current_attitude[YW] = ypr[YW];
  current_attitude[AC] = ypr[AC];
  current_attitude[BD] = ypr[BD];  

#ifdef CASCADE_PIDS    
  // acceleration rate reading
  current_rate[YW] = gyro.z*-1.0;
  current_rate[AC] = gyro.x;  
  current_rate[BD] = gyro.y*-1.0;    
#endif  

  if(INPUT_THRUST > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) init_pid();    

    if( INPUT_THRUST > 350 ) {

    pid_attitude[YW].Compute();  
    pid_attitude[AC].Compute();      
    pid_attitude[BD].Compute();     

#ifdef CASCADE_PIDS
    pid_rate[YW].Compute();
    pid_rate[AC].Compute();     
    pid_rate[BD].Compute();         
#endif

    }


    //////////////////////////////
    // Motor Mix Algorithm      //
    //////////////////////////////
    // compute the boom thrust  //
#ifdef CASCADE_PIDS    
    v_ac = INPUT_THRUST - rate_correction[YW];
    v_bd = INPUT_THRUST + rate_correction[YW];
#else
    v_ac = INPUT_THRUST - attitude_correction[YW]; 
    v_bd = INPUT_THRUST + attitude_correction[YW]; 
#endif

    // compute motor speeds
#ifdef CASCADE_PIDS
    va = MIN_ESC_CUTOFF + (v_ac - rate_correction[AC]); 
    vc = MIN_ESC_CUTOFF + (v_ac + rate_correction[AC]); 
    vb = MIN_ESC_CUTOFF + (v_bd - rate_correction[BD]); 
    vd = MIN_ESC_CUTOFF + (v_bd + rate_correction[BD]); 
#else
    va = MIN_ESC_CUTOFF + (v_ac - attitude_correction[AC]); 
    vc = MIN_ESC_CUTOFF + (v_ac + attitude_correction[AC]); 
    vb = MIN_ESC_CUTOFF + (v_bd - attitude_correction[BD]); 
    vd = MIN_ESC_CUTOFF + (v_bd + attitude_correction[BD]); 
#endif
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

    yw_offset = (float)((int)(ypr[YW]*10.0 + .5))/10.0;
    
    pid_reset(); //(MANUAL);
  }

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);  

}

//////////////////////////////////////////////////////////////////////

void do_log() {
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
  }
}


