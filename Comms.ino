#if USE_CMD_MESSENGER

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
//--------------------------------------------------------------------------------------------------------
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kVersion,       OnVersion);
  cmdMessenger.attach(kSetRudderPos,  OnSetRudderPos);
  cmdMessenger.attach(kGetRudderPos,  OnGetRudderPos);
  cmdMessenger.attach(kSetSpeed,      OnSetSpeed);
  cmdMessenger.attach(kGetSpeed,      OnGetSpeed);
  cmdMessenger.attach(kSetHeading,    OnSetHeading);
  cmdMessenger.attach(kGetHeading,    OnGetHeading);  
  cmdMessenger.attach(kGetCurrent,    OnGetCurrent);
  cmdMessenger.attach(kCalCurrent,    OnCalCurrent);
  cmdMessenger.attach(kExtraLed,      OnExtraLed);
}

//--------------------------------------------------------------
//--------------------------------------------------------------
void task_Comms()
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
}

// ------------------  C A L L B A C K S -----------------------

//--------------------------------------------------------------
// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError,"Unknown Cmd");
}

//--------------------------------------------------------------
// Callback function that responds that Arduino is ready (has booted up)

//--------------------------------------------------------------
void OnVersion()
{
  cmdMessenger.sendCmd(kAcknowledge, "Version 2.0");
}

//--------------------------------------------------------------
void OnSetRudderPos()
{
  byte cmdSetting = (byte)cmdMessenger.readInt16Arg();
  
  Servo_Rudder.write( cmdSetting );

  // Acknowlege the new setting
  cmdMessenger.sendCmd(kSetRudderPos, Servo_Rudder.read());
}

//--------------------------------------------------------------
void OnGetRudderPos()
{
  cmdMessenger.sendCmd(kGetRudderPos, Servo_Rudder.read());
}

//--------------------------------------------------------------
void OnSetSpeed()
{
  byte cmdSetting = (byte)cmdMessenger.readInt16Arg();
    
  Servo_MotorEsc.write( cmdSetting );

  // Acknowlege the new setting  
  cmdMessenger.sendCmd(kSetSpeed, Servo_MotorEsc.read());
}

//--------------------------------------------------------------
void OnGetSpeed()
{
  cmdMessenger.sendCmd(kGetSpeed, Servo_MotorEsc.read());
}

//--------------------------------------------------------------
void OnSetHeading()
{
  float cmdSetting = cmdMessenger.readFloatArg();
#if USE_BERRY_IMU  
  // Send new heading to auto-pilot
  AutoPilot_SetNewHeading( cmdSetting );
#endif
  // Acknowlege the new setting
  cmdMessenger.sendCmd(kSetHeading, cmdSetting);
}

//--------------------------------------------------------------
void OnGetHeading()
{
#if USE_BERRY_IMU
  cmdMessenger.sendCmd(kGetHeading, BerryIMU_GetCompassHeading());
#endif
}

//--------------------------------------------------------------
void OnGetCurrent()
{
  // Report current
  cmdMessenger.sendCmd(kGetCurrent, gCurrent);
}

//--------------------------------------------------------------
void OnCalCurrent()
{
  float offset = CalibrateCurrentSensor();
  
  // Send out the computed offset
  cmdMessenger.sendCmd(kCalCurrent, offset);
}

//--------------------------------------------------------------
void OnExtraLed()
{
  bool bOnOff = cmdMessenger.readBoolArg();
  
  if( bOnOff )
  {
    EXTRA_LED_ON;
  }
  else
  {
    EXTRA_LED_OFF;
  }
  
  // Acknowlege the new setting
  cmdMessenger.sendCmd(kExtraLed, bOnOff);
}

#endif // USE_CMD_MESSENGER
