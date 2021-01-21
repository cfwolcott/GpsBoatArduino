
// GPS_Boat Version 2.0

// Includes
#include <Arduino.h> 
#include <Servo.h>
#include <Wire.h>

#include "Config.h"
#include "myTypedefs.h"
#include "RunningAverage.h"
#include "CurrentSensor.h"

#if USE_BERRY_IMU
#include "LSM9DS0.h"
#include "BerryImu.h"
#include "AutoPilot.h"
#endif

#if USE_CMD_MESSENGER
// Uses the CmdMessenger library to communicate with the RPi
#include <CmdMessenger.h>
#include "Comms.h"
#endif

#if USE_LCD
#include <LiquidCrystal_I2C.h>
#endif

//-------------------------------------------
// Defines

// LEDs ----------------------
#define LED_ON         digitalWrite(LED_PIN, HIGH);
#define LED_OFF        digitalWrite(LED_PIN, LOW);

#define EXTRA_LED_ON   digitalWrite(EXTRA_LED_PIN, HIGH);
#define EXTRA_LED_OFF  digitalWrite(EXTRA_LED_PIN, LOW);

// LCD -----------------------
#if USE_LCD
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

// Current Sensors ----------------------
RUNNING_SIGNED_AVERAGE_TYPE gtRaA2dValue;
float gCurrent;
int gCalOffset;  // Will get updated in setup()

// Servos --------------------
// Servos (including the motor ESC which is controlled like a servo)
Servo Servo_Rudder;
int gRudderPot;

Servo Servo_MotorEsc;
int gEscPot;

// Serial Messaging --------------------
int led_state = 0;

#if USE_CMD_MESSENGER
// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);
#endif // USE_CMD_MESSENGER

// Local function defines
void task_StatusUpdate();
void task_ReadPots();
void task_LedBlink();
void clearAndHome();

//--------------------------------------------------------------------------------------------------------
//----- setup --------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(EXTRA_LED_PIN, OUTPUT);
    pinMode(CURRENT_SENSOR_PIN, INPUT);

    // start serial for output
    Serial.begin(9600);
    
#if (USE_BERRY_IMU || USE_LCD)
    // join I2C bus
    Wire.begin();
#endif

#if USE_BERRY_IMU    
    BerryIMU_Setup();
#endif // USE_BERRY_IMU
       
    // Servo Init
    Servo_Rudder.attach(SERVO_RUDDER_PIN);
    Servo_MotorEsc.attach(MOTOR_ESC_PIN);
    
    // Set servo positions
    Servo_Rudder.write( SERVO_RUDDER_CENTER );
    Servo_MotorEsc.write( 0 );    // need to find out what default for OFF will be
    
#if USE_POTS
    pinMode(RUDDER_POT_PIN, INPUT);
    pinMode(ESC_POT_PIN, INPUT);
#endif  // USE_POTS

    RA_Signed_Init( RA_MAX_SIGNED_SAMPLES, &gtRaA2dValue );

    // Calibrate A2D input for current sensor
    CalibrateCurrentSensor();

#if USE_CMD_MESSENGER
    // Adds newline to every command
    cmdMessenger.printLfCr();   
  
    // Attach my application's user-defined callback methods
    attachCommandCallbacks();
    
    cmdMessenger.sendCmd(kAcknowledge, "Ready!");
#else
    Serial.println("Ready!");
#endif // USE_CMD_MESSENGER

#if USE_LCD
  lcd.init();                      // initialize the lcd
  lcd.begin(16,2);
  lcd.backlight();
  lcd.print("GPS Boat v2.0");
#endif
}

//--------------------------------------------------------------------------------------------------------
//----- loop ---------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
void loop() 
{
#if USE_CMD_MESSENGER
  // *** Serial Command Messenger with RPi **************************************
  task_Comms();
#endif // USE_CMD_MESSENGER

  // *** Current Sensor *********************************************************
  task_CurrentSensor();
  
#if USE_BERRY_IMU
  // *** Berry IMU Compass ******************************************************
  task_BerryIMU_Filter();
  task_BerryIMU();
  
  // *** Auto Pilot task ********************************************************
  task_AutoPilot();
#endif  // USE_BERRY_IMU
  
#if USE_POTS
  // *** Servo Pot simulate *****************************************************
  task_ReadPots();
#endif // USE_POTS

  // *** Status Updates *********************************************************
  task_StatusUpdate();

  // *** LED Blinking ***********************************************************
  task_LedBlink();
}

//--------------------------------------------------------------------------------------------------------
#if USE_POTS
#define TIMER_READ_POTS_MILLIS  50
void task_ReadPots()
{
  static U32 taskTimer = millis();
  
  if( TIME_DIFF( taskTimer ) > TIMER_READ_POTS_MILLIS )
  {
    taskTimer = millis();

    // Rudder
    static int rudderPotLast = 0;
    
    gRudderPot = map( analogRead( RUDDER_POT_PIN ), 0, 1023, 0, 179 );
    Servo_Rudder.write( gRudderPot );
    
    if( rudderPotLast != gRudderPot )
    {
      rudderPotLast = gRudderPot;
  #if USE_CMD_MESSENGER
      OnGetRudderPos();
  #endif // USE_CMD_MESSENGER
    }
    
    // ESC
    static int escPotLast = 0;
    
    gEscPot = map( analogRead( ESC_POT_PIN ), 0, 1023, 0, 179 );
    Servo_MotorEsc.write( gEscPot );
    
    if( escPotLast != gEscPot )
    {
      escPotLast = gEscPot;
  #if USE_CMD_MESSENGER
      OnGetSpeed();
  #endif // USE_CMD_MESSENGER
    }
  }
}
#endif // #if USE_POTS

//--------------------------------------------------------------------------------------------------------
// Any periodic, un-solicited messages can be sent from here
#define TIMER_STATUS_UPDATE_MILLIS  500
void task_StatusUpdate()
{
  static U32 taskTimer = millis();
  static U32 update_counter = 0;
#if USE_COMPASS_TEST
  static U32 testTimer = millis();
  static float heading = 0;
  
  if( TIME_DIFF( testTimer ) > 5000 )
  {
    heading += 90.0;
    heading = (heading >= 360.0) ? 0 : heading;
    
    Serial.print("New heading: "); Serial.println(heading);
    AutoPilot_SetNewHeading( heading );
    
    EXTRA_LED_ON;
    delay(100);
    EXTRA_LED_OFF;
    
    testTimer = millis();
  }
  
#endif
  
  if( TIME_DIFF( taskTimer ) > TIMER_STATUS_UPDATE_MILLIS )
  {
    taskTimer = millis();
    
//    Serial.print("Compass HDG: "); Serial.println( BerryIMU_GetCompassHeading(), 1 );

#if USE_CMD_MESSENGER
//    OnGetCurrent();
#endif // USE_CMD_MESSENGER

#if USE_POTS

#if USE_LCD
    lcd.clear();
    lcd.home();
    // Servos
    lcd.print("RUD:"); lcd.print(gRudderPot);
    lcd.print(" ESC:"); lcd.print(gEscPot);
    // Current sensor
    lcd.setCursor(0,1);
    lcd.print("CUR: "); lcd.print(gCurrent); lcd.print(" Amps");
#else
    // Clear terminal screen (only works on non-Arduino Serial terminal)
    clearAndHome();

    Serial.print("Update Counter: "); Serial.println(update_counter++);
    Serial.print("Rudder Pos: "); Serial.println(gRudderPot);
    Serial.print("Esc Pos: "); Serial.println(gEscPot);
    Serial.print("Current: "); Serial.print(gCurrent); Serial.println(" Amps");
#endif // USE_LCD

#endif // USE_POTS

  }
}

//--------------------------------------------------------------------------------------------------------
#define TIMER_LED_BLINK_MILLIS  1000
void task_LedBlink()
{
    static U32 taskTimer = millis();
    static bool bBlinkState = false;
    
    if( TIME_DIFF( taskTimer ) > TIMER_LED_BLINK_MILLIS )
    {
      taskTimer = millis();
      
      // set the LED on/off
      bBlinkState = !bBlinkState;
      
      if( bBlinkState )
      {
        LED_ON;
      }
      else
      {
        LED_OFF;
      }
    }
}

//--------------------------------------------------------------------------------------------------------
void clearAndHome()
{
  Serial.print((char)27); // ESC
  Serial.print("[2J"); // clear screen
  Serial.print((char)27); // ESC
  Serial.print("[H"); // cursor to home
}

