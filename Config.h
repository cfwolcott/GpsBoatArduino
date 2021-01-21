// Configuration file for GPS Boat Arduino controller

#define USE_POTS            1
#define USE_CMD_MESSENGER   0
#define USE_BERRY_IMU       0
#define USE_LCD             1

#define COMPASS_DECLINATION  -13.0  // California
//#define COMPASS_DECLINATION  6.0  // Florida

// Pin defines ----------------
#define LED_PIN              13
#define EXTRA_LED_PIN        12
#define CURRENT_SENSOR_PIN   A0

#define SERVO_RUDDER_PIN         10
#define SERVO_RUDDER_CENTER      100
#define SERVO_RUDDER_LEFT_MAX    60
#define SERVO_RUDDER_RIGHT_MAX   125

#if (SERVO_RUDDER_CENTER > SERVO_RUDDER_LEFT_MAX)
#define SERVO_RUDDER_REVERSE     0
#else
#define SERVO_RUDDER_REVERSE     1
#endif

#define MOTOR_ESC_PIN            7

// Potentiometer for servo testing -----
  #if USE_POTS
#define RUDDER_POT_PIN  A2
#define ESC_POT_PIN     A3
  #endif // USE_POTS

#if USE_BERRY_IMU
// Compass config flags
#define USE_COMPASS_TILT_COMPENSATION    1
#define USE_COMPASS_CALIBRATION          0
#define USE_COMPASS_AUTO_CALIBRATION     0
#define USE_COMPASS_ONETIME_CALIBRATION  0
#define USE_COMPASS_INVERTED             0
#define USE_COMPASS_TEST                 0

// For compass tilt compensation
 #if USE_COMPASS_TILT_COMPENSATION
#define USE_KALMAN_FILTER           0
#define USE_COMPLIMENTARY_FILTER    1
 #endif // USE_COMPASS_TILT_COMPENSATION

#endif // USE_BERRY_IMU

#if USE_CMD_MESSENGER
// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events

// The message format is:
// Cmd Id, param 1, [...] , param N;

enum
{
  // Commands
  kAcknowledge,   // 0 - Command to acknowledge that cmd was received
  kError,         // 1 - Command to report errors
  kVersion,       // 2 - Command to request version of this GPS_Boat software
  kSetRudderPos,  // 3 - set the rudder servo
  kGetRudderPos,  // 4 - get rudder servo position value  
  kSetSpeed,      // 5 - Command to set the ESC to desired speed
  kGetSpeed,      // 6 - get ESC speed setting
  kSetHeading,    // 7 - Sets auto-pilot heading
  kGetHeading,    // 8 - Gets the current compass heading
  kGetCurrent,    // 9 - Command to get the current reading from current sensor
  kCalCurrent,    // 10 - calibrate zero offset of current sensor
  kExtraLed,      // 11 - Command to set the extra LED on/off
};
#endif // USE_CMD_MESSENGER
