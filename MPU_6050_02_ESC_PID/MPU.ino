////////////////////////////////////////////////////////////////
// init_mpu
//
void init_mpu()
{
    system_check &= ~(INIT_MPU_ARMED | INIT_MPU_STABLE);
      
    Serial.println(F("#Init MPU I2C conne..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("#Test device conn..."));
    Serial.println(mpu.testConnection() ? F("#MPU6050 ok") : F("MPU6050 failed"));

    delay(500);

    // load and configure the DMP
    Serial.println(F("#Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
      // Supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
      mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
      mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
      mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
      mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
      mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

///////////////////////////////////////////////////////////////////
      mpu.setDLPFMode(MPU6050_DLPF_BW_5);
//#define MPU6050_DLPF_BW_256         0x00
//#define MPU6050_DLPF_BW_188         0x01
//#define MPU6050_DLPF_BW_98          0x02
//#define MPU6050_DLPF_BW_42          0x03
//#define MPU6050_DLPF_BW_20          0x04
//#define MPU6050_DLPF_BW_10          0x05
//#define MPU6050_DLPF_BW_5           0x06
/*
* DLPF_CFG | Bandwidth | Delay | Bandwidth | Delay | Sample Rate
* ---------+-----------+--------+-----------+--------+-------------
* 0         | 260Hz     | 0ms     | 256Hz   | 0.98ms  | 8kHz
* 1         | 184Hz     | 2.0ms   | 188Hz   | 1.9ms   | 1kHz
* 2         | 94Hz      | 3.0ms   | 98Hz    | 2.8ms   | 1kHz
* 3         | 44Hz      | 4.9ms   | 42Hz    | 4.8ms   | 1kHz
* 4         | 21Hz      | 8.5ms   | 20Hz    | 8.3ms   | 1kHz
* 5         | 10Hz      | 13.8ms  | 10Hz    | 13.4ms  | 1kHz
* 6         | 5Hz       | 19.0ms  | 5Hz     | 18.6ms  | 1kHz
* 7         | -- Reserved -- | -- Reserved -- | Reserved  
*/

      // turn on the DMP, now that it's ready
      Serial.println(F("#Enabling DMP..."));
      mpu.setDMPEnabled(true);
      
      // enable Arduino interrupt detection
      //Serial.println(F("#Enabling int detection"));
      //attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("#DMP ready!"));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      system_check |= INIT_MPU_ARMED;
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("#DMP Init failed (cd "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

////////////////////////////////////////////////////////////////
// read_mpu
//
void read_mpu()
{
  // reset interrupt flag and get INT_STATUS byte
  // mpuInterrupt = false;
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

  } // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // this is in radians

    mpu.dmpGetGyro(&gyro1, fifoBuffer); // this is in degrees/s?  I'm certain deg/sec

    // low pass filter on the gyro data
    #define alpha 0.4
    gyro.x = gyro1.x * alpha + (gyro.x * (1.0 - alpha));
    gyro.y = gyro1.y * alpha + (gyro.y * (1.0 - alpha));
    gyro.z = gyro1.z * alpha + (gyro.z * (1.0 - alpha)); 
    // low pass filter on the gyro data   

    // convert radians to degrees
    #define A_180_DIV_PI 57.2957795131
    ypr[YW] = ((ypr[YW] * A_180_DIV_PI) - yw_offset ) ;
    ypr[AC] = ((ypr[AC] * A_180_DIV_PI) - ac_offset ) ;
    ypr[BD] = ((ypr[BD] * A_180_DIV_PI) - bd_offset ) ;

/*
    if( system_check & INIT_MPU_STABLE )
    {
      if( (abs(ypr[AC] - ypr_last[AC]) > 30) ) 
      {
        Serial.print(F("#bg chng"));
        Serial.print("\t");
        Serial.print(ypr_last[AC]);
        Serial.print("\t");
        Serial.println(ypr[AC]);
      }

      if( (abs(ypr[BD] - ypr_last[BD]) > 30) ) 
      {
        Serial.print(F("#bg chng"));
        Serial.print("\t");
        Serial.print(ypr_last[BD]);
        Serial.print("\t");
        Serial.println(ypr[BD]);
      }      

      if (abs(ypr[YW] - ypr_last[YW]) > 30) ypr[YW] = ypr_last[YW];
      if (abs(ypr[BD] - ypr_last[BD]) > 30) ypr[BD] = ypr_last[BD];
      if (abs(ypr[AC] - ypr_last[AC]) > 30) ypr[AC] = ypr_last[AC];
     
    }
    
    ypr_last[YW] = ypr[YW];
    ypr_last[AC] = ypr[AC];
    ypr_last[BD] = ypr[BD];
*/
  
  }
}
