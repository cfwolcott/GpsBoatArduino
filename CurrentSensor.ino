//--------------------------------------------------------------------------------------------------------
#define TIMER_CURRENT_SENSOR_MILLIS  20
void task_CurrentSensor()
{
  static U32 taskTimer = millis();
  
  if( TIME_DIFF( taskTimer ) > TIMER_CURRENT_SENSOR_MILLIS )
  {
    taskTimer = millis();
    
    // Collect and average A2D input values from current sensor
    S16 currentA2dValue = (S16)(analogRead( CURRENT_SENSOR_PIN ) - gCalOffset);
  
    if( currentA2dValue < 0 )
    {
      currentA2dValue = 0;
    }
      
    // Compute the running average
    RA_ComputeSingedAverage( currentA2dValue, &gtRaA2dValue );
    
    // Compute current reading
    float voltage = (5.0 * gtRaA2dValue.s16Average) / 1024;
  
    // Sensor is 100mV/A
    gCurrent = (voltage / 0.1);
  }
}

//--------------------------------------------------------------------------------------------------------
float CalibrateCurrentSensor()
{
  int i;
  S16 a2dSample;
  
  for( i=0; i<RA_MAX_SIGNED_SAMPLES; i++)
  {
    a2dSample = (S16)analogRead( CURRENT_SENSOR_PIN );
    RA_ComputeSingedAverage( a2dSample, &gtRaA2dValue );
  }
  
  // Store the global A2dOffset for no current flow
  gCalOffset = gtRaA2dValue.s16Average;
  
    // Compute current reading
  float voltage = (5.0 * gCalOffset) / 1024;

  // Sensor is 100mV/A
  float currentOffset = ((voltage - 2.5) / 0.1);
  
  return currentOffset;
}
