
//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  float foo = map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 400.0);

  return (float)( (int) (((foo * 10.0)+.5)/10.0));
}

double read_kp()
{
  double foo = map(analogRead(Kp_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );
  return foo;
}
double read_ki()
{
  double foo = map(analogRead(Ki_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 2000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );
  return foo;
}
double read_kd()
{
  double foo = map(analogRead(Kd_PIN), 0.0, 668.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );  
  return foo;
}
///////////////////////////////////////////////////////////////////////
