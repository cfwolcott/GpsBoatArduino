// Comms.h
// Provides a communication interface between the Arduino and RPi
// Uses the CmdMessenger library
//
// Global Function Prototypes
//
void attachCommandCallbacks();
void task_Comms();
void OnUnknownCommand();
void OnVersion();
void OnSetRudderPos();
void OnGetRudderPos();
void OnSetSpeed();
void OnGetSpeed();
void OnSetHeading();
void OnGetHeading();
void OnGetCurrent();
void OnCalCurrent();
void OnExtraLed();
