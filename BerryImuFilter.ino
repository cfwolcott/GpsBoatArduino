#if (USE_KALMAN_FILTER || USE_COMPLIMENTARY_FILTER)

#define USE_IMU_DEBUG               0

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]

#if USE_KALMAN_FILTER
// Used by Kalman Filters
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float gKalmanX, gKalmanY;
#else
float gCFangleX = 0.0;
float gCFangleY = 0.0;
#endif // USE_KALMAN_FILTER

//--------------------------------------------------------------------------------------------------------
void BerryIMU_Filter_GetAngles( float *x, float *y )
{
#if USE_KALMAN_FILTER
  *x = -gKalmanX;
  *y = -gKalmanY;
#else
  *x = -gCFangleX;
  *y = -gCFangleY;
#endif
}

//--------------------------------------------------------------------------------------------------------
// Computes and stores compass X and Y tilt angles
#define TIMER_IMU_MILLIS  (DT * 1000)
void task_BerryIMU_Filter()
{
  static U32 taskTimer = millis();
  static U32 printTimer = millis();
  
  float rate_gyr_y = 0.0;   // [deg/s]  
  float rate_gyr_x = 0.0;   // [deg/s]
  float rate_gyr_z = 0.0;   // [deg/s]
   
  float AccYangle = 0.0;
  float AccXangle = 0.0;
  
  if( TIME_DIFF( taskTimer ) > TIMER_IMU_MILLIS )
  {
    taskTimer = millis();
    
    //read ACC and GYR data
    readACC(accRaw);
    readGYR(gyrRaw);
    
    //Convert Gyro raw to degrees per second
    rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
    rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
    rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

    //Convert Accelerometer values to degrees
    AccXangle = (float) (atan2(accRaw[1], accRaw[2]) + M_PI) * RAD_TO_DEG;
    AccYangle = (float) (atan2(accRaw[2], accRaw[0]) + M_PI) * RAD_TO_DEG;

    //Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
    //Two different pieces of code are used depending on how your IMU is mounted.
    //If IMU is upside down
/*
    if (AccXangle >180.0)
            AccXangle -= 360.0;

    AccYangle-=90;

    if (AccYangle >180.0)
            AccYangle -= 360.0;
*/
    //If IMU is up the correct way, use these lines
    AccXangle -= 180.0;
    if (AccYangle > 90.0)
      AccYangle -= 270.0;
    else
      AccYangle += 90.0;

#if USE_KALMAN_FILTER
    //Kalman Filter
    gKalmanX = kalmanFilterX(AccXangle, rate_gyr_x);
    gKalmanY = kalmanFilterY(AccYangle, rate_gyr_y);
#else
    //Complementary filter used to combine the accelerometer and gyro values.
    gCFangleX = AA * (gCFangleX + rate_gyr_x * DT) + (1 - AA) * AccXangle;
    gCFangleY = AA * (gCFangleY + rate_gyr_y * DT) + (1 - AA) * AccYangle;
#endif // USE_KALMAN_FILTER

#if USE_IMU_DEBUG
    if( TIME_DIFF( printTimer ) > 1000 )
    {
      printTimer = millis();
#if USE_KALMAN_FILTER
      Serial.print("kalmanX: "); Serial.print(gKalmanX);
      Serial.print(" kalmanY: "); Serial.println(gKalmanY);
#else
      Serial.print("CFangleX: "); Serial.print(gCFangleX);
      Serial.print(" CFangleY: "); Serial.println(gCFangleY);
#endif // USE_KALMAN_FILTER
    }
#endif // USE_IMU_DEBUG
  }
}

#if USE_KALMAN_FILTER
//--------------------------------------------------------------------------------------------------------
float kalmanFilterX(float accAngle, float gyroRate)
{
  float  y, S;
  float K_0, K_1;
  static float KFangleX = 0.0;
  static float x_bias = 0;
  static float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;

  KFangleX += DT * (gyroRate - x_bias);

  XP_00 +=  - DT * (XP_10 + XP_01) + Q_angle * DT;
  XP_01 +=  - DT * XP_11;
  XP_10 +=  - DT * XP_11;
  XP_11 +=  + Q_gyro * DT;

  y = accAngle - KFangleX;
  S = XP_00 + R_angle;
  K_0 = XP_00 / S;
  K_1 = XP_10 / S;

  KFangleX +=  K_0 * y;
  x_bias  +=  K_1 * y;
  XP_00 -= K_0 * XP_00;
  XP_01 -= K_0 * XP_01;
  XP_10 -= K_1 * XP_00;
  XP_11 -= K_1 * XP_01;

  return KFangleX;
}

//--------------------------------------------------------------------------------------------------------
float kalmanFilterY(float accAngle, float gyroRate)
{
  float  y, S;
  float K_0, K_1;
  static float KFangleY = 0.0;
  static float y_bias = 0;
  static float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
  
  KFangleY += DT * (gyroRate - y_bias);

  YP_00 +=  - DT * (YP_10 + YP_01) + Q_angle * DT;
  YP_01 +=  - DT * YP_11;
  YP_10 +=  - DT * YP_11;
  YP_11 +=  + Q_gyro * DT;

  y = accAngle - KFangleY;
  S = YP_00 + R_angle;
  K_0 = YP_00 / S;
  K_1 = YP_10 / S;

  KFangleY +=  K_0 * y;
  y_bias  +=  K_1 * y;
  YP_00 -= K_0 * YP_00;
  YP_01 -= K_0 * YP_01;
  YP_10 -= K_1 * YP_00;
  YP_11 -= K_1 * YP_01;

  return KFangleY;
}
#endif // #if USE_KALMAN_FILTER
#endif // (USE_KALMAN_FILTER || USE_COMPLIMENTARY_FILTER)
