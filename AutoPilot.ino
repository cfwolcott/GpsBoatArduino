#if USE_BERRY_IMU

typedef enum
{
  E_GO_LEFT,
  E_GO_RIGHT,
  E_GO_STRAIGHT
} E_DIRECTION;

// Angle sweep of the rudder servo
#define RUDDER_RANGE_ANGLE  abs(SERVO_RUDDER_LEFT_MAX - SERVO_RUDDER_RIGHT_MAX)
#define RUDDER_HALF_RANGE   (RUDDER_RANGE_ANGLE / 2)

float gAutoPilotHeading = 0;

//--------------------------------------------------------------------------------------------------------
#define TIMER_AUTOPILOT_MILLIS  40
void task_AutoPilot()
{
  static U32 taskTimer = millis();
  E_DIRECTION eDirToGo;
  
  if( TIME_DIFF( taskTimer ) > TIMER_AUTOPILOT_MILLIS )
  {
    taskTimer = millis();
    
    // Get current compass heading and Calculate correction angle/error

    int Diff = (int)(gAutoPilotHeading - BerryIMU_GetCompassHeading() );
    int AbsDiff = abs(Diff);
    bool bNeg = (Diff < 0);  
    bool bBig = (AbsDiff > 180);

    if( AbsDiff <= 2 )
    {
	// We're with-in a few degrees of the target. Just go straight!
        eDirToGo = E_GO_STRAIGHT;
    }
    else
    {
	if( !bNeg && !bBig )
	  eDirToGo = E_GO_RIGHT;
	if( !bNeg && bBig )
	  eDirToGo = E_GO_LEFT;
	if( bNeg && !bBig )
	  eDirToGo = E_GO_LEFT;
	if( bNeg && bBig )
	  eDirToGo = E_GO_RIGHT;
    }
    
    // Send correction to rudder servo
    int correctionAngle;
    static int lastCorrectionAngle = 0;
    
    if(bBig)
    {
      AbsDiff = 360 - AbsDiff;
    }
    
    switch( eDirToGo )
    {
      case E_GO_LEFT:
#if SERVO_RUDDER_REVERSE
        correctionAngle = min( (SERVO_RUDDER_CENTER + AbsDiff), SERVO_RUDDER_LEFT_MAX );
#else
        correctionAngle = max( (SERVO_RUDDER_CENTER - AbsDiff), SERVO_RUDDER_LEFT_MAX );
#endif
        break;
      case E_GO_RIGHT:
#if SERVO_RUDDER_REVERSE
        correctionAngle = max( (SERVO_RUDDER_CENTER - AbsDiff), SERVO_RUDDER_RIGHT_MAX );
#else
        correctionAngle = min( (SERVO_RUDDER_CENTER + AbsDiff), SERVO_RUDDER_RIGHT_MAX );
#endif
        break;
      default:
      case E_GO_STRAIGHT:
        correctionAngle = SERVO_RUDDER_CENTER;
        break;
    }

    // Apply a low-pass filter to smooth out the transitions
    correctionAngle = LowPassFilter( lastCorrectionAngle, correctionAngle );
    lastCorrectionAngle = correctionAngle;
    
    // apply correction
    Servo_Rudder.write( correctionAngle );
  }
}

//--------------------------------------------------------------------------------------------------------
void AutoPilot_SetNewHeading( float newHeading )
{
  gAutoPilotHeading = newHeading;
}

#endif // #if USE_BERRY_IMU
