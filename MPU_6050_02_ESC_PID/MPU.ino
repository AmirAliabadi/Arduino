
////////////////////////////////////////////////////////////////
// init_mpu
//
void init_mpu()
{
  if (!dmp_ready)
  {
    Serial.println(F("#Initializing MPU I2C connection..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("#Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("#MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("#Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
      // turn on the DMP, now that it's ready
      Serial.println(F("#Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // Supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
      mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
      mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
      mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
      mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
      mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
    
      mpu.setDLPFMode(MPU6050_DLPF_BW_5);
//#define MPU6050_DLPF_BW_256         0x00
//#define MPU6050_DLPF_BW_188         0x01
//#define MPU6050_DLPF_BW_98          0x02
//#define MPU6050_DLPF_BW_42          0x03
//#define MPU6050_DLPF_BW_20          0x04
//#define MPU6050_DLPF_BW_10          0x05
//#define MPU6050_DLPF_BW_5           0x06  

//      mpu.setDHPFMode(MPU6050_DHPF_0P63);
//#define MPU6050_DHPF_RESET          0x00
//#define MPU6050_DHPF_5              0x01
//#define MPU6050_DHPF_2P5            0x02
//#define MPU6050_DHPF_1P25           0x03
//#define MPU6050_DHPF_0P63           0x04
//#define MPU6050_DHPF_HOLD           0x07          

      // enable Arduino interrupt detection
      Serial.println(F("#Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("#DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      mpu_debug_info_hz = last_mpu_read = millis();

      dmp_ready = true;
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("#DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
}

////////////////////////////////////////////////////////////////
// read_mpu
//
bool read_mpu()
{
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();

#ifdef DEBUG
    Serial.println(F("#FIFO overflow!"));
#endif

    return false;

  } // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    last_mpu_read = millis();

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //mpu.dmpGetAccel(&aa, fifoBuffer);
    //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    ypr[YW] = (ypr[YW]) * 180.0 / M_PI ;
    ypr[AC] = (ypr[AC]) * 180.0 / M_PI ;
    ypr[BD] = (ypr[BD]) * 180.0 / M_PI ;

    ypr[YW] = (float)((int)((ypr[YW] * 10.0) + 0.5))/10.0;
    ypr[AC] = (float)((int)((ypr[AC] * 10.0) + 0.5))/10.0;
    ypr[BD] = (float)((int)((ypr[BD] * 10.0) + 0.5))/10.0;

    if( have_first )
    {
      if( abs(ypr[AC] - ypr_last[AC]) > 30) 
      {
        Serial.print(F("#*****   whoa! big change .... *****"));
        Serial.print("\t");
        Serial.print(ypr_last[AC]);
        Serial.print("\t");
        Serial.println(ypr[AC]);
        
        //mpu.resetDMP();
        
        //return false; 
      }

      if (abs(ypr[YW] - ypr_last[YW]) > 30) ypr[YW] = ypr_last[YW];
      if (abs(ypr[BD] - ypr_last[BD]) > 30) ypr[BD] = ypr_last[BD];
      if (abs(ypr[AC] - ypr_last[AC]) > 30) ypr[AC] = ypr_last[AC];
    }
    
    ypr_last[YW] = ypr[YW];
    ypr_last[AC] = ypr[AC];
    ypr_last[BD] = ypr[BD];

    have_first = true;

    // Update the PID input values
    input_ypr[YW] = (double)ypr[YW];
    input_ypr[AC] = (double)ypr[AC];
    input_ypr[BD] = (double)ypr[BD];

    return true;
  }
  else
  {
    // MPU was not ready
  }

  return false;
}

