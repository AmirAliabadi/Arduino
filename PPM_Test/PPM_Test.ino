void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(6, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);
  
}

volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned long ppm_dt = 0;
volatile uint16_t ppm_channels[6];
volatile uint8_t ppm_channel = 0;

void ppmRising() {
    current_ppm_clock = micros();
    ppm_dt = current_ppm_clock - last_ppm_clock;
    if( ppm_dt >= 4000 ) ppm_channel = 0;
    else {
      ppm_channels[ppm_channel] = ppm_dt * .8 + ( ppm_channels[ppm_channel] * (1.0 - .8)); 
      //ppm_channels[ppm_channel] = ppm_dt ;
      ppm_channel ++ ;
    }
    last_ppm_clock = current_ppm_clock;   
}

uint16_t throttle = 0;

void read_throttle() {
  if( throttle >= 0 and throttle <= 1000 ) {
    throttle += ( ppm_channels[2] - 1500 );
  }
}

unsigned long last_log = 0;
void loop() 
{  
    read_throttle()
    if( millis() - last_log  > 600 ) {
      Serial.println( throttle );
//      for( int i = 0; i < 6; i ++ )
//      {
//        Serial.print( ppm_channels[i] );
//        Serial.print( " : " );
//      }
//      Serial.println();
      last_log = millis();
    }
}



