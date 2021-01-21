#if USE_BERRY_IMU

#define USE_COMPASS_DEBUG                0

// Local Function Prototypes
void BerryIMU_ComputeCalMinMax();
void BerryIMU_Filter_GetAngles( float *x, float *y );
void writeAccReg(uint8_t reg, uint8_t value);
void writeMagReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);
void readBlock(uint8_t devAddr, uint8_t command, uint8_t size, uint8_t *data);
void readACC(int  *a);
void readMAG(int  *m);
void readGYR(int *g);

// Compass calibration value
int magXmax = -32000;
int magYmax = -32000;
int magZmax = -32000;
int magXmin = 32000;
int magYmin = 32000;
int magZmin = 32000;

float gCompassHeading = 0.0;

// Global so we can do constant calibration
int magRaw[3];
int accRaw[3];
int gyrRaw[3];

//--------------------------------------------------------------------------------------------------------
// Computes and stores compass heading in gCompassHeading
#define TIMER_COMPASS_MILLIS  100
void task_BerryIMU()
{
  static U32 taskTimer = millis();
  static U32 printTimer = millis();
  
  float scaledMag[3];
  
  if( TIME_DIFF( taskTimer ) > TIMER_COMPASS_MILLIS )
  {
    taskTimer = millis();
    
    readMAG(magRaw);
    
#if USE_COMPASS_CALIBRATION
    
#if USE_COMPASS_AUTO_CALIBRATION
    // Continuously update min/max values
    BerryIMU_ComputeCalMinMax();
#endif // USE_COMPASS_AUTO_CALIBRATION

    //Apply hard iron calibration
    magRaw[0] -= (magXmin + magXmax) / 2 ;
    magRaw[1] -= (magYmin + magYmax) / 2 ;
    magRaw[2] -= (magZmin + magZmax) / 2 ;
  
    //Apply soft iron calibration
    scaledMag[0] = (float)(magRaw[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
    scaledMag[1] = (float)(magRaw[1] - magYmin) / (magYmax - magYmin) * 2 - 1;
    scaledMag[2] = (float)(magRaw[2] - magZmin) / (magZmax - magZmin) * 2 - 1;
  
    //Only needed if the heading value does not increase when the magnetometer is rotated clockwise
//    scaledMag[1] = -scaledMag[1];
#else // USE_COMPASS_CALIBRATION
    scaledMag[0] = (float)magRaw[0];
    scaledMag[1] = (float)magRaw[1];
    scaledMag[2] = (float)magRaw[2];
#endif // USE_COMPASS_CALIBRATION

#if !USE_COMPASS_TILT_COMPENSATION

    //Compute heading - without compensation
    gCompassHeading = 180 * atan2(magRaw[1], magRaw[0]) / M_PI;
    
#else // USE_COMPASS_TILT_COMPENSATION
    
    float accXnorm, accYnorm, pitch, roll, magXcomp, magYcomp;

#if (USE_KALMAN_FILTER || USE_COMPLIMENTARY_FILTER)
    
    BerryIMU_Filter_GetAngles( &roll, &pitch );

    // Convert to radians
    roll *= DEG_TO_RAD;
    pitch *= DEG_TO_RAD;
    
#else // not using (USE_KALMAN_FILTER || USE_COMPLIMENTARY_FILTER)
    readACC(accRaw);
    
#if USE_COMPASS_INVERTED
    accRaw[0] = -accRaw[0];
    accRaw[1] = -accRaw[1];
#endif // USE_COMPASS_INVERTED

    //Normalize accelerometer raw values.
    float accNomalized = sqrt( sq((float)accRaw[0]) + sq((float)accRaw[1]) + sq((float)accRaw[2]) );
    accXnorm = (float)accRaw[0] / accNomalized;
    accYnorm = (float)accRaw[1] / accNomalized;
    
    //Calculate pitch and roll
    pitch = asin(accXnorm);
#if USE_COMPASS_INVERTED
    roll = asin(accYnorm / cos(pitch));
#else
    roll = -asin(accYnorm / cos(pitch));
#endif // USE_COMPASS_INVERTED

#endif // (USE_KALMAN_FILTER || USE_COMPLIMENTARY_FILTER)

    //Calculate the new tilt compensated values
    magXcomp = scaledMag[0] * cos(pitch) + scaledMag[2] * sin(pitch);
    magYcomp = scaledMag[0] * sin(roll) * sin(pitch) + scaledMag[1] * cos(roll) - scaledMag[2] * sin(roll) * cos(pitch);
    
    //Calculate heading
    gCompassHeading = 180.0 * atan2(magYcomp, magXcomp) / M_PI;

#endif // USE_COMPASS_TILT_COMPENSATION

    gCompassHeading -= COMPASS_DECLINATION;

    //Convert heading to 0 - 360
    if(gCompassHeading < 0.0)
    {
      gCompassHeading += 360.0;
    }
#if USE_COMPASS_DEBUG
    if( TIME_DIFF( printTimer ) > 500 )
    {
      printTimer = millis();
      Serial.print("Roll: "); Serial.print(roll, 1);
      Serial.print("\tPitch: "); Serial.print(pitch, 1);
      Serial.print("\tCompass heading: "); Serial.println(gCompassHeading, 1);
    }
#endif
  }
}

//--------------------------------------------------------------------------------------------------------
float BerryIMU_GetCompassHeading()
{
  return gCompassHeading;
}

//--------------------------------------------------------------------------------------------------------
void BerryIMU_Setup()
{
  // Enable accelerometer.
  writeAccReg(CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
  writeAccReg(CTRL_REG2_XM, 0b00100000); // +/- 16G full scale
  
  //Enable the magnetometer
  writeMagReg( CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
  writeMagReg( CTRL_REG6_XM, 0b01100000);   // +/-12gauss
  writeMagReg( CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode
  
  // Enable Gyro
  writeGyrReg( CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled  
  writeGyrReg( CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
  
#if USE_COMPASS_ONETIME_CALIBRATION
  BerryIMU_Calibrate();
#endif // USE_COMPASS_CALIBRATION
}

//--------------------------------------------------------------------------------------------------------
void BerryIMU_Calibrate()
{
  static U32 calTimer = millis() + 10000;
//  int magRaw[3];
  
  while( millis() < calTimer )
  {
    readMAG(magRaw);
    BerryIMU_ComputeCalMinMax();
#if USE_COMPASS_DEBUG
    Serial.print(magXmin); Serial.print(" "); Serial.print(magXmax);
    Serial.print(magYmin); Serial.print(" "); Serial.print(magYmax);
    Serial.print(magZmin); Serial.print(" "); Serial.println(magZmax);
#endif    
    delay(25);
  }
}

//--------------------------------------------------------------------------------------------------------
void BerryIMU_ComputeCalMinMax()
{
    if (magRaw[0] > magXmax) magXmax = magRaw[0];
    if (magRaw[1] > magYmax) magYmax = magRaw[1];
    if (magRaw[2] > magZmax) magZmax = magRaw[2];

    if (magRaw[0] < magXmin) magXmin = magRaw[0];
    if (magRaw[1] < magYmin) magYmin = magRaw[1];
    if (magRaw[2] < magZmin) magZmin = magRaw[2];
}

//--------------------------------------------------------------------------------------------------------
void writeAccReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission( ACC_ADDRESS );
  Wire.write( reg );
  Wire.write( value );
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------------------------------
void writeMagReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission( MAG_ADDRESS );
  Wire.write( reg );
  Wire.write( value );
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------------------------------
void writeGyrReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission( GYR_ADDRESS );
  Wire.write( reg );
  Wire.write( value );
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------------------------------
void readBlock(uint8_t devAddr, uint8_t command, uint8_t size, uint8_t *data)
{
  int8_t count = 0;
  uint32_t t1 = millis();
  uint16_t timeout = 100;
    
  Wire.beginTransmission(devAddr);
  Wire.write(command);
  Wire.endTransmission();
  
  Wire.beginTransmission(devAddr);
  Wire.requestFrom(devAddr, size);
  
  for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++)
  {
      data[count] = Wire.read();
  }
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------------------------------
void readACC(int  *a)
{
  uint8_t block[6];

  readBlock(ACC_ADDRESS, (0x80 | OUT_X_L_A), sizeof(block), block);
  
  *a = (int16_t)(block[0] | block[1] << 8);
  *(a+1) = (int16_t)(block[2] | block[3] << 8);
  *(a+2) = (int16_t)(block[4] | block[5] << 8);

}

//--------------------------------------------------------------------------------------------------------
void readMAG(int  *m)
{
  uint8_t block[6];
  
  readBlock(MAG_ADDRESS, (0x80 | OUT_X_L_M), sizeof(block), block);
  
  *m = (int16_t)(block[0] | block[1] << 8);
  *(m+1) = (int16_t)(block[2] | block[3] << 8);
  *(m+2) = (int16_t)(block[4] | block[5] << 8);

}

//--------------------------------------------------------------------------------------------------------
void readGYR(int *g)
{
  uint8_t block[6];
  
  readBlock(GYR_ADDRESS, (0x80 | OUT_X_L_G), sizeof(block), block);

  *g = (int16_t)(block[0] | block[1] << 8);
  *(g+1) = (int16_t)(block[2] | block[3] << 8);
  *(g+2) = (int16_t)(block[4] | block[5] << 8);
}

#endif // USE_BERRY_IMU
