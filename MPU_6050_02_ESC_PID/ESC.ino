
void init_esc()
{
    Serial.println(F("#Attaching to motor pins"));
    esc_a.attach(MOTOR_PIN_A);
    esc_b.attach(MOTOR_PIN_B);
    esc_c.attach(MOTOR_PIN_C);
    esc_d.attach(MOTOR_PIN_D);

    system_check |= INIT_ESC_ATTACHED;

    arm_esc();
}

void arm_esc()
{
log_data();  
  esc_a.writeMicroseconds(MIN_ESC_SIGNAL + thrust); 
  esc_b.writeMicroseconds(MIN_ESC_SIGNAL + thrust);
  esc_c.writeMicroseconds(MIN_ESC_SIGNAL + thrust); 
  esc_d.writeMicroseconds(MIN_ESC_SIGNAL + thrust);

  delay(ESC_ARM_DELAY);

  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
    Serial.println(F("#Detaching motor pins"));
    esc_a.detach();
    esc_b.detach();
    esc_c.detach();
    esc_d.detach();  

    system_check &= ~(INIT_ESC_ATTACHED | INIT_ESC_ARMED );
}

