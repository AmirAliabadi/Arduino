

void do_blink()
{
  if( millis() - last_blink > blink_pattern[blink_point] )
  {
    if(++blink_point > 3) blink_point = 0;
    
    last_blink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));    
  }
}

