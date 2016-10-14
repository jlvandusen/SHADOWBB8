// =======================================================================================
//            SHADOWBB8 :  Small Handheld Arduino Droid Operating Wand for BB8
// =======================================================================================
//                          Last Revised Date: 10/13/2016
//                             Written By: jlvandusen
//                        Inspired by the PADAWAN by danf and KnightShade
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// =======================================================================================
//   Note: You will need a Arduino Mega 1280/2560 to run this sketch,
//   as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//   The arduino mega 2560 adk is the preferred method as it will involve less hardware
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//   for the James Bruton BB8 build version 3.. in replacement of his custom controller
//   see: www.youtube.com/user/jamesbruton and www.xrobots.co.uk
//
//      - Single PS3 Move Navigation
//      - Pair of PS3 Move Navigation controllers (recommended)
//      - Bluetooth connected Phone (Limited Controls)
//
// =======================================================================================

// ---------------------------------------------------------------------------------------
//                          Definitions Variables
// ---------------------------------------------------------------------------------------

// Use the below settings for arduino mega with USB shield and Bluetooth adapter 
// (prevents pin conflicts of 8-13 with shield)


#define ENABLE_PIN       31 // Sets the enable pin throughout the code

#define FLYWHEELLT_PIN   42 // Sets the flywheel pins for PWM 
#define FLYWHEELRT_PIN   44

#define GIMBLELT_PIN     43 // Sets the gimble left/right pins for PWM
#define GIMBLERT_PIN     45
#define GIMBLESP_PIN      7 // Sets the gimble spin motor for PWM

#define MAINFWD_PIN       2 // Sets the forward/backward motor for PWM
#define MAINREV_PIN       3

#define SWINGLT_PIN       5 // Sets the gyro/flywheel swing motor for PWM
#define SWINGRT_PIN       6

// Use the below settings for arduino mega ADK without USB shield
// ---------------------------------------------------------------------------------------
//#define ENABLE_PIN       31 // Sets the enable pin throughout the code

//#define FLYWHEELLT_PIN   9  // Sets the flywheel pins for PWM       
//#define FLYWHEELRT_PIN   10
//
//#define GIMBLELT_PIN     6  // Sets the gimble left/right pins for PWM
//#define GIMBLERT_PIN     7
//#define GIMBLESP_PIN     8  // Sets the gimble spin motor for PWM
//
//#define MAINFWD_PIN      2 // Sets the forward/backward motor for PWM
//#define MAINREV_PIN      3
//
//#define SWINGLT_PIN      4 // Sets the gyro/flywheel swing motor for PWM
//#define SWINGRT_PIN      5


#define SHADOW_DEBUG            //uncomment this for console DEBUG output
#define SHADOW_VERBOSE          //uncomment this for console VERBOSE output
//#define SHADOW_DEBUG_MPU        //uncomment this for console DEBUG MPU6050 output
//#define SHADOW_DEBUG_POT        //uncomment this for console DEBUG POT output
#define SHADOW_DEBUG_JOY        //uncomment this for console DEBUG Joystick(s) output
//#define SHADOW_DEBUG_GIMBLE     //uncomment this for console DEBUG Gimble Servos output
//#define SHADOW_DEBUG_GIMBLETURN //uncomment this for console DEBUG Gimble Servos output
#define SHADOW_DEBUG_FLYWHEEL   //uncomment this for console DEBUG Flywheel Servos
//#define SHADOW_DEBUG_MAINDRIVES
//#define TEST_CONTROLLER       //Support coming soon

// ---------------------------------------------------------------------------------------
//                          Libraries included
// ---------------------------------------------------------------------------------------
#ifdef dobogusinclude
  #include <spi4teensy3.h> // Satisfy IDE, USED FOR UNO.
#endif

#include <PS3BT.h>
#include <SPP.h>
#include <usbhub.h>
#include <Servo.h>

#include <I2Cdev.h>
#include <MPU6050.h>  // utilize raw communications to MPU rather than DMP
#include <Wire.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <MP3Trigger.h>
MP3Trigger trigger;

String PS3MoveNavigatonPrimaryMAC = "00:06:F7:8F:36:7B"; //If using multiple controlers, designate a primary

byte joystickDeadZoneRange = 15;  // For controllers that centering problems, use the lowest number with no drift
byte driveDeadBandRange = 15;     // Used to set the Motor Controller DeadZone for drive motors

// ---------------------------------------------------------------------------------------
//                          Drive/Servo Controller Settings
// ---------------------------------------------------------------------------------------

Servo servo1;
Servo servo2;
Servo servo3;

int varServo1; // Left servo for gimble arm movement
int varServo2; // Right servo for gimble arm movement

int turnnum = 0;      // what is the X left and right of the joysticks.
int mainDrive = 0;    // what is the Y forward and back of the joysticks.

long previousDomeMillis = millis();
long previousDriveMillis = millis();
long previousMainDriveMillis = millis();
long currentMillis = millis();
int serialLatency = 25;   //This is a delay factor in ms to prevent queueing of the Serial data.
                          //25ms seems approprate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation

USB Usb; // USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS3BT *PS3Nav=new PS3BT(&Btd);
PS3BT *PS3Nav2=new PS3BT(&Btd);

uint32_t msgLagTime = 0; //Used for PS3 Fault Detection
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
int badPS3Data = 0;
uint32_t lastLoopTime = 0;

boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

SPP SerialBT(&Btd,"Astromech:BB8","0000"); // Create a BT Serial device(defaults: "Arduino" and the pin to "0000" if not set)
boolean firstMessage = true;
String output = "";



byte vol = 40; // 0 = full volume, 255 off
boolean isStickEnabled = true;
byte isAutomateDomeOn = false;
byte isDriveEnabled = false;
byte isFootMotorStopped = true;
byte isMotorStopped = true;
int DriveSpeed = 0;
unsigned long automateMillis = 0;
byte automateDelay = random(5,30);// set this to min and max seconds between sounds
int turnDirection = 20;
byte action = 0;
unsigned long DriveMillis = 0;

// ---------------------------------------------------------------------------------------
//               MPU Configuration
// ---------------------------------------------------------------------------------------

MPU6050 mpu; //MPU6050 mpu(0x69); // <-- use for AD0 high

unsigned long timer = 0; // Timers
float timeStep = 0.01;
float pitch = 0; // Pitch, Roll and Yaw values
float roll = 0;
float yaw = 0;
unsigned long mpuinterval=1000; // the time we need to wait (1sec)
unsigned long previousmpuMillis=0;

float bodge = 7;

int pot;

// ---------------------------------------------------------------------------------------
//               PID Configuration
// ---------------------------------------------------------------------------------------

double Kp1 = 3;
double Ki1 = 0;
double Kd1 = 0;
double Setpoint1, Input1, Output1, Output1a;    // PID variables

PID PID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1 , Kd1, DIRECT);    // PID Setup - trousers SERVO

double Kp2 = .6; // position it goes to
double Ki2 = 0;
double Kd2 = 0.3;
double Setpoint2, Input2, Output2, Output2a;    // PID variables - Trouser stability swing

PID PID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2 , Kd2, DIRECT);    // PID Setup - Trouser stability swing

double Kp3 = 3; // position it goes to
double Ki3 = 0;
double Kd3 = 0.4;
double Setpoint3, Input3, Output3, Output3a;    // PID variables - Main drive motor

PID PID3(&Input3, &Output3, &Setpoint3, Kp3, Ki3 , Kd3, DIRECT);    // Main drive motor


// ---------------------------------------------------------------------------------------
//               Ramping/Easing Control Configuration
// ---------------------------------------------------------------------------------------
int current_pos_headturn;  // variables for smoothing head rotation
int target_pos_headturn;
int pot_headturn;   // target position/inout
int diff_headturn; // difference of position
double easing_headturn;

int current_pos_drive;  // variables for smoothing main drive
int target_pos_drive;
int pot_drive;   // target position/inout
int diff_drive; // difference of position
double easing_drive;

int current_pos_trousers;  // variables for smoothing trousers
int target_pos_trousers;
int pot_trousers;   // target position/inout
int diff_trousers; // difference of position
double easing_trousers;

int current_pos_head1;  // variables for smoothing head back/forward
int target_pos_head1;
int pot_head1;   // target position/inout
int diff_head1; // difference of position
double easing_head1;

int ch1,ch2,ch3,ch3a,ch4,ch4a,ch5,ch5a,ch6,ch6a;

// ---------------------------------------------------------------------------------------
//                       INTERRUPT DETECTION ROUTINE
// ---------------------------------------------------------------------------------------

// DMP Values required - Comment to use RAW below ----------------------------------------
//volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() 
//{
//    mpuInterrupt = true;
//}

// ---------------------------------------------------------------------------------------
//                       SETUP Definitions
// ---------------------------------------------------------------------------------------

void setup()
{
    Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect, used on Leonardo, Teensy and other
    if (Usb.Init() == -1)
    {
        Serial.print(F("\r\nOSC Bluetooth did not start"));
        while (1); //halt
    }
    Serial.println(F("\r\nBluetooth Library Started"));
    output.reserve(200); // Reserve 200 bytes for the output string

    //Setup for PS3
    PS3Nav->attachOnInit(onInitPS3); // onInit() is called upon a new connection
    PS3Nav2->attachOnInit(onInitPS3Nav2); 

// *************************** MPU Setup and Configuration ***************************************

    if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    }
    else
    {
      Serial.println("Found a valid MPU6050 sensor, calibration beginning");
      // Calibrate gyroscope. The calibration must be at rest.
      // If you don't want calibrate, comment this line.
      mpu.calibrateGyro();
      
      // Set threshold sensivty. Default 3.
      // If you don't want use threshold, comment this line or set 0.
      mpu.setThreshold(3);
      Serial.println("MPU6050 calibration completed");
    }

// *************************** Sound Controller setup *********************************************
    
    trigger.setup(&Serial1);
    trigger.setVolume(vol);

// *************************** Motor Controllers setup *********************************************

    pinMode(ENABLE_PIN, OUTPUT); // enable pin for all motor controllers (button combo of L2 + CIRCLE enables)

//  Motor Controller (single) Gyro/swing Drive
//  PWMTop = Pin 5 on Arduino
//  PWMBot = Pin 6 on Arduino
//  VCC = 5V from this distro
//  Gnd = Gnd from this distro
//  Enable = Pin 31 on Arduino
    pinMode(SWINGLT_PIN, OUTPUT);  // LEFT Direction on swing gyro motor
    pinMode(SWINGRT_PIN, OUTPUT);  // RIGHT Direction on swing gyro motor

//  Motor Controller (Left MC) Main Drive
//  PWMTop = Pin 2 on Arduino
//  PWMBot = Pin 3 on Arduino
//  VCC = 5V from this distro
//  Gnd = Gnd from this distro
//  Enable = Pin 31 on Arduino

//  Motor Controller (Right MC) flywheel Spin
//  PWMTop = Pin 43 on Arduino
//  PWMBot = Pin 44 on Arduino
//  VCC = 5V from this distro
//  Gnd = Gnd from this distro
//  Enable = Pin 31 on Arduino
//  NOTE: Combine the + / - together from both motors.

    
// *************************** Gimble setup *********************************************
   
//  SERVO 3 (Gimble spin)
//  Signal = Pin 8 on arduino
    servo3.attach(GIMBLESP_PIN);     // attach center Servo for gimble

//  SERVO 1/2 (Gimble Head Movement)
//  Left Servo signal = Pin 54 on arduino - Note switched it to 13 due to issues with pin 9
//  Right Servo Signal = Pin 52 on arduino
//    servo1.attach(GIMBLERT_PIN);     // Attach Left Servo for gimble
//    servo2.attach(GIMBLELT_PIN);    // Attach Right Servo for gimble
    

// *************************** Setup for Dome Communications ******************************
//  Will need BT hardware to communicate to Disco Droid BB8 Sound controller in dome
//  start the library, data details / name of serial port.

    Serial3.begin(115200); 

    
// *************************** PID setup **************************************************
  
    PID1.SetMode(AUTOMATIC);              // PID Setup - trousers SERVO
    PID1.SetOutputLimits(-255, 255);
    PID1.SetSampleTime(20);
  
    PID2.SetMode(AUTOMATIC);              // PID Setup - trousers Stability
    PID2.SetOutputLimits(-255, 255);
    PID2.SetSampleTime(20);
  
    PID3.SetMode(AUTOMATIC);              // PID Setup - main drive motor
    PID3.SetOutputLimits(-255, 255);
    PID3.SetSampleTime(20);

}

// =======================================================================================
//                          readUSB Usb.Task() commands
// =======================================================================================
//  The more devices we have connected to the USB or BlueTooth, the more often 
//  Usb.Task need to be called to eliminate latency.

boolean readUSB()
{
    Usb.Task();
    if (PS3Nav->PS3NavigationConnected ) Usb.Task();
    if (PS3Nav2->PS3NavigationConnected ) Usb.Task();
    if (criticalFaultDetect()) // We have a fault condition return false
    {
      return false;
    }
    
    if (criticalFaultDetectNav2()) // We have a fault condition return false
    { 
      return false;
    }
    return true;
}

// =======================================================================================
//                          MAIN APPLICATION LOOP
// =======================================================================================

void loop() // LOOP through functions from highest to lowest priority.
{
    getMPU();   // go and get the values from the MPU6050 and return roll and pitch
    getPOT();   // go and get the values from the Potentiometer and return pot
    initTerminal();
    if ( !readUSB() )
    {
      return;
    }
    else
    {
//      testController();
      GetJoyStickValues();  // this configures the move controllers to update ch1-6 with values
      DomeDrive();          // controls the gimble for left, right and forward/backward movement
//      spinFlywheel();       // Spins the flywheels for stationary turn
      MotorDrive();         // Main Drive and Gyro swing for turning
      toggleSettings();     // Listens for key combinations from the move controllers
    }
    flushTerminal();
}

// =======================================================================================
//                          getMPU test data verify its working
// =======================================================================================

void getMPU()
{
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected) 
  {
    unsigned long currentmpuMillis = millis(); // grab current time 
    if ((unsigned long)(currentmpuMillis - previousmpuMillis) >= mpuinterval) // check if "interval" time has passed (1000 milliseconds)
    {
      float timeStep = 0.01;
      Vector norm = mpu.readNormalizeGyro();    // Read normalized values
      
      pitch = pitch + norm.YAxis * timeStep;    // Calculate Pitch
      roll = roll + norm.XAxis * timeStep;      // Calculate Roll
      yaw = yaw + norm.ZAxis * timeStep;        // Calculate yaw
      
      // Output raw
      #ifdef SHADOW_DEBUG_MPU
        output += "\tPitch: ";
        output += pitch;
        output += "\tRoll: ";
        output += roll;
      #endif 
      previousmpuMillis = ((timeStep*1000) - (millis() - timer));    // Wait to full timeStep period
    } 
  }
  else return;

}

// =======================================================================================
//                          getPOT test data verify its working
// =======================================================================================

void getPOT()
{
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected) 
  {
    pot = analogRead(A0);   // read trousers pot
    #ifdef SHADOW_DEBUG_POT
      output += "\tPOT: ";
      output += pot;
    #endif 
  }
  else return;
}

// =======================================================================================
//                          moveGIMBLE test data verify its working
// =======================================================================================

void DomeDrive()
{
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected)
  {
    readUSB();  // verify PS3 navigation controller is still alive.
    if ((millis() - previousDomeMillis) < (2 * serialLatency) ) return;
//    ch3a=map(ch3,50,590,0,180); // jlv modified variables
    ch3a=map(ch3,0,255,0,180);
    ch3a = ch3a-(pitch*4.5);
    ch3a=constrain(ch3a,0,180);
    
    target_pos_head1 = ch3a;
    
    easing_head1 = 100;          //modify this value for sensitivity
    easing_head1 /= 1000;
    
    // Work out the required travel.
    diff_head1 = target_pos_head1 - current_pos_head1;    
    
    // Avoid any strange zero condition
    if( diff_head1 != 0.00 ) {
      current_pos_head1 += diff_head1 * easing_head1;
      }
    
    varServo1 = current_pos_head1;
    varServo2 = current_pos_head1;
    
    ch4a=map(ch4,0,255,-50,50); 
    
    varServo1 = varServo1-ch4a;
    varServo2 = varServo2+ch4a;
    
    varServo2 = map(varServo2,0,180,180,0);
    
    varServo1 = constrain(varServo1,10,170);
    varServo2 = constrain(varServo2,10,170);

    target_pos_headturn = map(ch5, 0,255,0,180);
    
//    easing_headturn = 50;          //modify this value for sensitivity
//    easing_headturn /= 1000;
    
    // Work out the required travel.
    diff_headturn = target_pos_headturn - current_pos_headturn;    
    
    // Avoid any strange zero condition
    if( diff_headturn != 0.00 ) 
    {
//      current_pos_headturn += diff_headturn * easing_headturn;
      current_pos_headturn += diff_headturn; // jlv modified variables
    }
    #ifdef SHADOW_DEBUG_GIMBLETURN
      output += "\tCH5: ";
      output += ch5;
      output += "\tTargetPOS: ";
      output += target_pos_headturn;
      output += "\tDiffHeadTurn: ";
      output += diff_headturn;
      output += "\tCurrentPOS: ";
      output += current_pos_headturn;    
    #endif
        
    if (!isDriveEnabled)        // enable pin is off
    {
      if (servo1.read()!=60) servo1.write(60);         // set servos to back/middle position
      if (servo2.read()!=120) servo2.write(120);
      servo1.detach();
      servo2.detach();
    }
    else if (isDriveEnabled)    // enable pin is off
    {
      servo1.attach(GIMBLERT_PIN);      // Attach Left Servo for gimble
      servo2.attach(GIMBLELT_PIN);      // Attach Right Servo for gimble
      servo1.write(varServo1);          // set servos to stick positions
      servo2.write(varServo2); 
//      servo3.writeMicroseconds(current_pos_headturn); 
      servo3.write(current_pos_headturn); // jlv modified variables
    }
    #ifdef SHADOW_DEBUG_GIMBLE
      output += "\tCH3: ";
      output += ch3;
      output += "\tCH4: ";
      output += ch4;
      output += "\tCH5: ";
      output += ch5;    
      output += "\tServo1: ";
      output += varServo1;
      output += "\tServo2: ";
      output += varServo2;
      output += "\tServo3: ";
      output += current_pos_headturn;
    #endif
    previousDomeMillis = millis();
  }
  else return;
}


// =======================================================================================
//                          spinFlywheel test data verify its working
// =======================================================================================

void spinFlywheel()
{
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected)
  {
    readUSB();
//    if ((millis() - previousDomeMillis) < (2 * serialLatency) ) return;
//    ch6a = map(ch6,0,512,255,-255);
    if (isDriveEnabled)
    {
      ch6a = map(ch6,0,255,127,-127); // jlv modified variables  PS3 LEFT L2 + RIGHT X
      if (ch6a <= -10)                // decide which way to turn the wheels based on deadSpot variable
      {
        ch6a = abs(ch6a);
        analogWrite(FLYWHEELLT_PIN, ch6a);         // set PWM pins 
        analogWrite(FLYWHEELRT_PIN, 0);
        }
      if (ch6a > 10)            // decide which way to turn the wheels based on deadSpot variable
        { 
        ch6a = abs(ch6a);
        analogWrite(FLYWHEELRT_PIN, ch6a);  
        analogWrite(FLYWHEELLT_PIN, 0);
        }
      
      else {
        analogWrite(FLYWHEELLT_PIN, 0);  
        analogWrite(FLYWHEELRT_PIN, 0);
      }
      #ifdef SHADOW_DEBUG_FLYWHEEL
        output += "\tCH6: ";
        output += ch6;
        output += "\tCH6a: ";
        output += ch6a;
      #endif
    }
  }
  else return;
}

void MotorDrive()
{
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected)
  {
    if (isDriveEnabled)
    {
      readUSB();
    
// =====================================================================================================================
//                          TROUSER Stability PID Controls (Left/Right)
// =====================================================================================================================
      target_pos_trousers = map(ch2, 0,255,-40,40); //255 get left and right off PS3 controller
      easing_trousers = 80;          //modify this value for sensitivity
      easing_trousers /= 1000;
      diff_trousers = target_pos_trousers - current_pos_trousers;    // Work out the required travel.
      if( diff_trousers != 0.00 )
      {
        current_pos_trousers += diff_trousers * easing_trousers;     // Avoid any strange zero condition
      }    
      Setpoint2 = current_pos_trousers;    
      Input2 = (roll*-1)+bodge;   // ****add a bit to the IMU to get the real middle point    
      Setpoint2 = constrain(Setpoint2, -40,40);    
      
      PID2.Compute();       // figure out PID for Joystick input (target_pos_trousers)    
      
      Setpoint1 = Output2;  // send the output2 from the joystick (after PID) as SetPoint1    
      pot = map(pot, 0, 1024, -255,255);
      pot = pot+9;    
      Input1  = pot;
      Input1 = constrain(Input1,-40,40);
      Setpoint1 = constrain(Setpoint1, -40,40);
      Setpoint1 = map(Setpoint1,40,-40,-40,40);
      PID1.Compute(); // determine PID from values provided by the MPU roll and combine with the potentiometer

// =====================================================================================================================
//                      gyro/flywheel swing motor
// =====================================================================================================================
      
      if (Output1 < -10)            // decide which way to turn the wheels based on deadSpot variable - was 0
        {
        Output1a = abs(Output1);
          analogWrite(SWINGLT_PIN, Output1a); // set PWM pins in direction of the Output1
          analogWrite(SWINGRT_PIN, 0);
        }
      else if (Output1 >= 20)      // decide which way to turn the wheels based on deadSpot variable - was 0
        { 
        Output1a = abs(Output1);
          analogWrite(SWINGRT_PIN, Output1a); // set PWM pins in the direction of the Output1
          analogWrite(SWINGLT_PIN, 0);
        } 
      else
        {       
          analogWrite(SWINGLT_PIN, 0); // set PWM pins to 0 and send no power to the drives
          analogWrite(SWINGRT_PIN, 0); 
        }
// =====================================================================================================================
//                          MAIN Drive Stability PID Controls (fwd/back)
// =====================================================================================================================
        target_pos_drive = map(ch1, 0,255,65,-65);
        
        easing_drive = 200;          //modify this value for sensitivity
        easing_drive /= 1000;
        
        // Work out the required travel.
        diff_drive = target_pos_drive - current_pos_drive;    
        
        // Avoid any strange zero condition
        if( diff_drive != 0.00 ) {
        current_pos_drive += diff_drive * easing_drive;
        }
        
        Setpoint3 = current_pos_drive;
        
        Input3 = pitch+3;
        
        PID3.Compute();
      
// =====================================================================================================================
//                      main motor
// =====================================================================================================================    
        if (Output3 <= -5)                               // decide which way to turn the wheels based on deadSpot variable (was 1)
        {
        Output3a = abs(Output3);
          analogWrite(MAINFWD_PIN, Output3a);           // set PWM pins 
          analogWrite(MAINREV_PIN, 0);
        }
        else if (Output3 > 5)                           // decide which way to turn the wheels based on deadSpot variable (was 1)
        { 
        Output3a = abs(Output3);
          analogWrite(MAINREV_PIN, Output3a);  
          analogWrite(MAINFWD_PIN, 0);
        }
        else
        {
          analogWrite(MAINREV_PIN, 0);  
          analogWrite(MAINFWD_PIN, 0);
        }
        #ifdef SHADOW_DEBUG_MAINDRIVES
          output += "\tPot: ";
          output += pot;
          output += "/ Input1(pot): ";
          output += Input1;
          output += "/ SetPoint1: ";
          output += Setpoint1;
          output += "\tOutput1a: ";
          output += Output1a;
          output += "\tOutput3a: ";
          output += Output3a;
        #endif
    }
  }
  else
    {
      analogWrite(MAINFWD_PIN, 0); // set PWM pins to zero
      analogWrite(MAINREV_PIN, 0);
      analogWrite(SWINGLT_PIN, 0); // set PWM pins to zero
      analogWrite(SWINGRT_PIN, 0);
      
      isMotorStopped = true;
      
      #ifdef SHADOW_VERBOSE
              output += "\r\n***All Motors STOPPED***\r\n";
      #endif
    }
  return;
}

void ps3ToggleSettings(PS3BT* myPS3 = PS3Nav)
{
    readUSB();
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(L3))
    {
      //Quick Shutdown of PS3 Controller
      output += "\r\nDisconnecting the controller.\r\n";
      myPS3->disconnect();
    }
 
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CROSS)) //// enable / disable right stick & play sound
    {
        if(isStickEnabled)
          {
            #ifdef SHADOW_DEBUG
              output += "Disabling the DriveStick\r\n";
            #endif
            isStickEnabled = false;
          }
          else
          {
            #ifdef SHADOW_DEBUG
              output += "Enabling the DriveStick\r\n";
            #endif
            isStickEnabled = true;
          }
    }
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CIRCLE))
    {
        #ifdef SHADOW_DEBUG
          output += "Not Used\r\n";
        #endif
    }



    if(myPS3->getButtonPress(L2)&&myPS3->getButtonClick(CROSS))
    {
        if(isAutomateDomeOn)
        {
          #ifdef SHADOW_DEBUG
            output += "Disabling the Dome Automation\r\n";        
          #endif
          isAutomateDomeOn = false;
        }
        else
        {
          #ifdef SHADOW_DEBUG
            output += "Enabling the Dome Automation\r\n";
          #endif
          isAutomateDomeOn = true;
        }
        action = 0;
    }
    if(myPS3->getButtonPress(L2)&&myPS3->getButtonClick(CIRCLE))
    {
        if(isDriveEnabled)
        {
          #ifdef SHADOW_DEBUG
            output += "Disabling the Drives\r\n";        
          #endif
          isDriveEnabled = false;
          digitalWrite(31, LOW);
        }
        else
        {
          #ifdef SHADOW_DEBUG
            output += "Enabling the Drives\r\n";
          #endif
          isDriveEnabled = true;
          digitalWrite(31, HIGH);
        }
        action = 0;
    }

}

// =====================================================================================================================
//                          Get JoyStick Values from the PS3 move controller(s)
// =====================================================================================================================
// ch1 = main drive forward and back (right Joy Y)
// ch2 = gyro side to side (right joy X)
// ch3 = head front/back (left Joy Y)
// ch4 = head side to side (left Joy X)
// ch5 = head spin (right L2 + Left Joy X)
// ch6 = flywheel spin (left L2 + right joy X)
// ch7 = Unassigned  (right L2 + Left Joy Y)
// ch8 = unassigned  (leftL2 + right Joy Y)

void GetJoyStickValues()
{
  if ((millis() - previousMainDriveMillis) < serialLatency) return;  //Flood control prevention
  readUSB();
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected)
  {
    if (PS3Nav->getAnalogHat(LeftHatX) > 137 || PS3Nav->getAnalogHat(LeftHatX) < 117 || PS3Nav->getAnalogHat(LeftHatY) > 137 || PS3Nav->getAnalogHat(LeftHatY) < 117 || PS3Nav2->getAnalogHat(LeftHatX) > 137 || PS3Nav2->getAnalogHat(LeftHatX) < 117 || PS3Nav2->getAnalogHat(LeftHatY) > 137 || PS3Nav2->getAnalogHat(LeftHatY) < 117) 
    {
      ch1 = PS3Nav->getAnalogHat(LeftHatY);
      if (!PS3Nav2->getButtonPress(L2) && PS3Nav->getAnalogHat(LeftHatX))
      {
        ch2 = (PS3Nav->getAnalogHat(LeftHatX));
      }
      ch3 = PS3Nav2->getAnalogHat(LeftHatY);
      if (!PS3Nav->getButtonPress(L2) && PS3Nav2->getAnalogHat(LeftHatX))
      {
        ch4 = PS3Nav2->getAnalogHat(LeftHatX);
      }
      if (PS3Nav->getButtonPress(L2) && PS3Nav2->getAnalogHat(LeftHatX))
      {
        ch5 = PS3Nav2->getAnalogHat(LeftHatX); // ch5 = head spin
      }
      if (PS3Nav2->getButtonPress(L2) && PS3Nav->getAnalogHat(LeftHatX))
      {
        ch6 = PS3Nav->getAnalogHat(LeftHatX); // ch6 = flywheel spin
      }
      #ifdef SHADOW_DEBUG_JOY
        output += "\tCH1: ";
        output += ch1;
        output += "\tCH2: ";
        output += ch2;
        output += "\tCH3: ";
        output += ch3;
        output += "\tCH4: ";
        output += ch4;
        output += "\tCH5: ";
        output += ch5;
        output += "\tCH6: ";
        output += ch6;
      #endif
    }
    else
    {
       ch1 = 130;
       ch2 = 130;
       ch3 = 130;
       ch4 = 130;
       ch5 = 130;
       ch6 = 130;
    }
  }
  return;
}

#ifdef TEST_CONTROLLER
void PS3testController(PS3BT* myPS3 = PS3Nav)
{
    if (myPS3->PS3Connected || myPS3->PS3NavigationConnected) 
    {
        if (myPS3->getButtonPress(PS) && (myPS3->getAnalogHat(LeftHatX) > 137 || myPS3->getAnalogHat(LeftHatX) < 117 || myPS3->getAnalogHat(LeftHatY) > 137 || myPS3->getAnalogHat(LeftHatY) < 117 || myPS3->getAnalogHat(RightHatX) > 137 || myPS3->getAnalogHat(RightHatX) < 117 || myPS3->getAnalogHat(RightHatY) > 137 || myPS3->getAnalogHat(RightHatY) < 117)) {     
            output += "LeftHatX: ";
            output += myPS3->getAnalogHat(LeftHatX);
            output += "\tLeftHatY: ";
            output += myPS3->getAnalogHat(LeftHatY);
            if (myPS3->PS3Connected) { // The Navigation controller only have one joystick
                output += "\tRightHatX: ";
                output += myPS3->getAnalogHat(RightHatX);
                output += "\tRightHatY: ";
                output += myPS3->getAnalogHat(RightHatY);
            }
        }
        //Analog button values can be read from almost all buttons
        if (myPS3->getButtonPress(PS) && (myPS3->getAnalogButton(L2) || myPS3->getAnalogButton(R2)))
        {
            if (output != "")
                output += "\r\n";
            output += "z: ";
            output += myPS3->getAnalogButton(L2);
            if (myPS3->PS3Connected) {
                output += "\tR2: ";
                output += myPS3->getAnalogButton(R2);
            }
        }
        if (myPS3->getButtonClick(L2)) {
            output += " - L2";
            //myPS3->disconnect();
        }
        if (myPS3->getButtonClick(R2)) {
            output += " - R2";
            //myPS3->disconnect();
        }
        if (output != "") {
            Serial.println(output);
            if (SerialBT.connected)
                SerialBT.println(output);
            output = ""; // Reset output string
        }
        if (myPS3->getButtonClick(PS)) {
            output += " - PS";
            //myPS3->disconnect();
        }
        else {
            if (myPS3->getButtonClick(TRIANGLE))
                output += " - Traingle";
            if (myPS3->getButtonClick(CIRCLE))
                output += " - Circle";
            if (myPS3->getButtonClick(CROSS))
                output += " - Cross";
            if (myPS3->getButtonClick(SQUARE))
                output += " - Square";

            if (myPS3->getButtonClick(UP)) {
                output += " - Up";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED4);
                }
            }
            if (myPS3->getButtonClick(RIGHT)) {
                output += " - Right";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED1);
                }
            }
            if (myPS3->getButtonClick(DOWN)) {
                output += " - Down";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED2);
                }
            }
            if (myPS3->getButtonClick(LEFT)) {
                output += " - Left";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED3);
                }
            }

            if (myPS3->getButtonClick(L1))
                output += " - L1";
            if (myPS3->getButtonClick(L3))
                output += " - L3";
            if (myPS3->getButtonClick(R1))
                output += " - R1";
            if (myPS3->getButtonClick(R3))
                output += " - R3";

            if (myPS3->getButtonClick(SELECT)) {
                output += " - Select";
                myPS3->printStatusString();
            }
            if (myPS3->getButtonClick(START)) {
                output += " - Start";
            }
        }
    }          
}

void testController()
{
   if (PS3Nav->PS3NavigationConnected) PS3testController(PS3Nav);
   if (PS3Nav2->PS3NavigationConnected) PS3testController(PS3Nav2);
}  

#endif

void initTerminal()
{
  if (SerialBT.connected)    //Setup for Bluetooth Serial Monitoring
  {
    if (firstMessage)
    {
        firstMessage = false;
        SerialBT.println(F("Hello from S.H.A.D.O.W.")); // Send welcome message
    }
    //TODO:  Process input from the SerialBT
    //if (SerialBT.available())
    //    Serial.write(SerialBT.read());
  }
  else
  {
    firstMessage = true;  // reset and wait for device to connect
//    SerialBT.disconnect;
  }
}

void flushTerminal()
{
  if (output != "")
  {
    if (Serial) Serial.println(output);
    if (SerialBT.connected)
    {
      SerialBT.println(output);
      SerialBT.send();
    }
    else
    {
//      SerialBT.disconnect;
    }
    output = ""; // Reset output string
  }
}
void toggleSettings()
{
   if (PS3Nav->PS3NavigationConnected) ps3ToggleSettings(PS3Nav);
   if (PS3Nav2->PS3NavigationConnected) ps3ToggleSettings(PS3Nav2);
}


// =======================================================================================
// //////////////////////////Process PS3 Controller Fault Detection///////////////////////
// =======================================================================================
boolean criticalFaultDetect()
{
    if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
    {
        lastMsgTime = PS3Nav->getLastMessageTime();
        currentTime = millis();
        if ( currentTime >= lastMsgTime)
        {
          msgLagTime = currentTime - lastMsgTime;
        } else
        {
             #ifdef SHADOW_DEBUG
               output += "Waiting for PS3Nav Controller Data\r\n";
             #endif
             badPS3Data++;
             msgLagTime = 0;
        }
        
        if (msgLagTime > 100 && !isMotorStopped)
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 100ms since we heard from the PS3 Controller\r\n";
              output += "Shutting down motors, and watching for a new PS3 message\r\n";
            #endif
            isMotorStopped = true;
            return true;
        }
        if ( msgLagTime > 30000 )
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 30s since we heard from the PS3 Controller\r\n";
              output += "msgLagTime:";
              output += msgLagTime;
              output += "  lastMsgTime:";
              output += lastMsgTime;
              output += "  millis:";
              output += millis();            
              output += "\r\nDisconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }

        //Check PS3 Signal Data
        if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
        {
            // We don't have good data from the controller.
            //Wait 10ms, Update USB, and try again
            delay(10);
            Usb.Task();
            if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
            {
                badPS3Data++;
                #ifdef SHADOW_DEBUG
                    output += "\r\nInvalid data from PS3 Controller.";
                #endif
                return true;
            }
        }
        else if (badPS3Data > 0)
        {
            //output += "\r\nPS3 Controller  - Recovered from noisy connection after: ";
            //output += badPS3Data;
            badPS3Data = 0;
        }
        if ( badPS3Data > 10 )
        {
            #ifdef SHADOW_DEBUG
                output += "Too much bad data coming fromo the PS3 Controller\r\n";
                output += "Disconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }
    }
    else if (!isMotorStopped)
    {
        #ifdef SHADOW_DEBUG      
            output += "No Connected Controllers were found\r\n";
            output += "Shuting downing motors, and watching for a new PS3 message\r\n";
        #endif
        PS3Nav->disconnect();
        isMotorStopped = true;
        return true;
    }
    return false;
}


// =======================================================================================
//          Process of PS3 Secondary Controller Fault Detection
// =======================================================================================
boolean criticalFaultDetectNav2()
{
  if (PS3Nav2->PS3NavigationConnected || PS3Nav2->PS3Connected)
  {
    lastMsgTime = PS3Nav2->getLastMessageTime();
    currentTime = millis();
    
    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    } 
    else
    {
      #ifdef SHADOW_DEBUG
        output += "Waiting for PS3Nav Secondary Controller Data\r\n";
      #endif
      badPS3Data++;
      msgLagTime = 0;
    }
    
    if ( msgLagTime > 100000 )
    {
      #ifdef SHADOW_DEBUG
        output += "It has been 10s since we heard from the PS3 secondary Controller\r\n";
        output += "msgLagTime:";
        output += msgLagTime;
        output += " lastMsgTime:";
        output += lastMsgTime;
        output += " millis:";
        output += millis(); 
        output += "\r\nDisconnecting the secondary controller.\r\n";
      #endif
//      SyR->stop();
      PS3Nav2->disconnect();
      return true;
    }
    
    //Check PS3 Signal Data
    if(!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
    {
      // We don't have good data from the controller.
      //Wait 15ms, Update USB, and try again
      delay(15);
      Usb.Task();
      if(!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
      {
        badPS3Data++;
        #ifdef SHADOW_DEBUG
          output += "\r\nInvalid data from PS3 Secondary Controller.";
        #endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {
      badPS3Data = 0;
    }
  
    if ( badPS3Data > 10 )
    {
      #ifdef SHADOW_DEBUG
        output += "Too much bad data coming from the PS3 Secondary Controller\r\n";
        output += "Disconnecting the controller.\r\n";
      #endif
      PS3Nav2->disconnect();
      return true;
    }
  }
  
  return false;
}

String getLastConnectedBtMAC()
{
    String btAddress = "";
    for(int8_t i = 5; i > 0; i--)
    {
        if (Btd.disc_bdaddr[i]<0x10)
        {
            btAddress +="0";
        }
        btAddress += String(Btd.disc_bdaddr[i], HEX);
        btAddress +=(":");
    }
    btAddress += String(Btd.disc_bdaddr[0], HEX);
    btAddress.toUpperCase();
    return btAddress; 
}

void swapPS3NavControllers()
{
    PS3BT* temp = PS3Nav;
    PS3Nav = PS3Nav2;
    PS3Nav2 = temp;

    boolean tempStatus = isPS3NavigatonInitialized;     //Correct the status for Initialization
    isPS3NavigatonInitialized = isSecondaryPS3NavigatonInitialized;
    isSecondaryPS3NavigatonInitialized = tempStatus;

    PS3Nav->attachOnInit(onInitPS3);    //Must relink the correct onInit calls
    PS3Nav2->attachOnInit(onInitPS3Nav2); 
}

void onInitPS3()
{
    lastMsgTime = millis(); // this is added to reset the last checkin from the PS3 controller
    String btAddress = getLastConnectedBtMAC();
    PS3Nav->setLedOn(LED1);
    isPS3NavigatonInitialized = true;
    badPS3Data = 0;
    #ifdef SHADOW_DEBUG
      output += "\r\nBT Address of Last connected Device when Primary PS3 Connected: ";
      output += btAddress;
      if (btAddress == PS3MoveNavigatonPrimaryMAC)
      {
          output += "\r\nWe have our primary controller connected.\r\n";
      }
      else
      {
          output += "\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n";
      }
    #endif
}

void onInitPS3Nav2()
{
    lastMsgTime = millis(); // this is added to reset the last checkin from the PS3 controller
    String btAddress = getLastConnectedBtMAC();
    PS3Nav2->setLedOn(LED1);
    isSecondaryPS3NavigatonInitialized = true;
    badPS3Data = 0;
    if (btAddress == PS3MoveNavigatonPrimaryMAC) swapPS3NavControllers();
    #ifdef SHADOW_DEBUG
      output += "\r\nBT Address of Last connected Device when Secondary PS3 Connected: ";
      output += btAddress;
      if (btAddress == PS3MoveNavigatonPrimaryMAC)
      {
          output += "\r\nWe have our primary controller connecting out of order.  Swapping locations\r\n";
      }
      else
      {
          output += "\r\nWe have a secondary controller connected.\r\n";
      }
    #endif
}
