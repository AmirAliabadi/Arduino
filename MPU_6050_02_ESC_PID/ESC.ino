
void init_esc()
{
    disarm_esc();

    if( system_check & INIT_THROTTLE_ZEROED )
    {
      esc_a.attach(MOTOR_PIN_A);
      esc_c.attach(MOTOR_PIN_C);
      esc_b.attach(MOTOR_PIN_B);
      esc_d.attach(MOTOR_PIN_D);
  
      system_check |= INIT_ESC_ATTACHED;
  
      arm_esc();
    }
    else 
    {
      Serial.println(F("#zero the throttle"));
    }
}

void arm_esc()
{
//  esc_a.writeMicroseconds(MIN_ESC_SIGNAL + INPUT_THRUST); 
//  esc_c.writeMicroseconds(MIN_ESC_SIGNAL + INPUT_THRUST); 
//  esc_b.writeMicroseconds(MIN_ESC_SIGNAL + INPUT_THRUST);
//  esc_d.writeMicroseconds(MIN_ESC_SIGNAL + INPUT_THRUST);

//  delay(ESC_ARM_DELAY);

  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
#ifdef DEBUG  
    if( system_check & INIT_ESC_ATTACHED ) 
    {
      Serial.println(F("#Detaching motor pins"));
    }
#endif

    esc_a.writeMicroseconds(MIN_ESC_SIGNAL); 
    esc_c.writeMicroseconds(MIN_ESC_SIGNAL); 
    esc_b.writeMicroseconds(MIN_ESC_SIGNAL);
    esc_d.writeMicroseconds(MIN_ESC_SIGNAL);
    
    esc_a.detach();
    esc_c.detach();
    esc_b.detach();
    esc_d.detach();  
        
    // delay(3000); 

    system_check &= ~(INIT_ESC_ATTACHED | INIT_ESC_ARMED );
}

