
void init_esc()
{
  if ( !esc_ready )
  {
    Serial.println(F("#Attaching to motor pins"));
    esc_a.attach(MOTOR_PIN_A);
    //esc_b.attach(MOTOR_PIN_B);
    esc_c.attach(MOTOR_PIN_C);
    //esc_d.attach(MOTOR_PIN_D);

    arm_esc();

    esc_ready = true;
  }
}

void arm_esc()
{
  esc_a.writeMicroseconds(MIN_ESC_SIGNAL + read_throttle());
  //esc_b.writeMicroseconds(MIN_SIGNAL+ read_throttle());
  esc_c.writeMicroseconds(MIN_ESC_SIGNAL + read_throttle());
  //esc_d.writeMicroseconds(MIN_SIGNAL+ read_throttle());

  delay(ESC_ARM_DELAY);
}

void disarm_esc()
{
    Serial.println(F("#Detaching motor pins"));
    esc_a.detach();
    //esc_b.attach(MOTOR_PIN_B);
    esc_c.detach();
    //esc_d.attach(MOTOR_PIN_D);  

    esc_ready = false;
}

