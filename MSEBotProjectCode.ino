
//MSE 2202 Final Project
//Western Engineering base code
//2020 05 13 E J Porter
//Modified by: Azhar Rasiwala 3/18/2021

/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage)
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/

//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h"
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h"
#include "climbing.h"

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 1000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;
//Inner switch index
unsigned char ucMotorStateIndex2 = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

///////////////////////////////////////////////////////////////////////////////////////////////FOR QUICK AND EASY MOTOR ADJUSTMENT//////////////////////////////////////////////////
uint8_t leftWheelSpeedMod = 12;
uint8_t rightWheelSpeedMod = 8; 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//If beacon is hit change state
boolean beaconHit = false;
//Climbing Timer
const long climbInterval = 9500; // edited per robot based on time to reach top
unsigned long startClimb = 0;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  pinMode(ciHeartbeatLED, OUTPUT);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);

  SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
  SmartLEDs.clear();                          // Set all pixel colours to off
  SmartLEDs.show();                           // Send the updated pixel colours to the hardware

  
  setupClimbing();
   
}

void loop()
{
  //WSVR_BreakPoint(1);

  //average the encoder tick times
  ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();
        ENC_ClearRightOdometer();
        btRun = !btRun;
        Serial.println(btRun);
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          ucMotorStateIndex = 0;
          ucMotorStateIndex2 = 0;
          beaconHit = false;
          ucMotorState = 0;
          move(0);

        }

      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state

  if (!digitalRead(ciLimitSwitch))
  {
    btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = 0;
    ucMotorState = 0;
    move(0);
  }

  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    // Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
  }
  CR1_ulMainTimerNow = micros();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
  {
    WDT_ResetCore1();
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      case 0:
        {
          if (btRun)
          {
            CR1_ulMotorTimerNow = millis();
            if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)
            {

              // Serial.print(ucMotorStateIndex);
              // Serial.print("\n");
              switch (ucMotorStateIndex)
              {
                //Head Towards Obstacle
                //Position bot so that end of arduino board aligns w side of door frame furthest from door
                //Tested w inside bedroom door (blue walls)
                //Position bot 4.5 inches from wall or else limit switch bumper will crash into wall
                case 0:
                  {
                    ENC_SetDistance(75, 75);
                    ucMotorState = 1; //drive straight
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed + leftWheelSpeedMod;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed + rightWheelSpeedMod;
                    ucMotorStateIndex = 1; //proceed to next case
                    break;
                  }
                //stop
                case 1:
                  {
                    if (ENC_ISMotorRunning() == 0)
                    {
                      ucMotorStateIndex = 2;
                      ucMotorState = 5;
                    }
                    break;
                  }
                //Turn left
                case 2:
                  {
                    ENC_SetDistance(115, 115); //turn 270 left to get 90 right bc my robot doesn't like turning right
                    ucMotorState = 3;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed + leftWheelSpeedMod;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed + rightWheelSpeedMod;
                    ucMotorStateIndex = 3;
                    break;
                  }
                //stop
                case 3:
                  {
                    if (ENC_ISMotorRunning() == 0)
                    {
                      ucMotorState = 0;
                      ucMotorStateIndex = 4;
                    }
                    break;
                  }
                //Head Along Obstacle
                case 4:
                  {
                    ENC_SetDistance(90, 90); //need enough clearance so limit switch bumper doesn't bump into object
                    ucMotorState = 1;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed + leftWheelSpeedMod;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed + rightWheelSpeedMod;
                    ucMotorStateIndex = 5;
                    break;
                  }
                //Stop after end of obstacle
                case 5:
                  {
                    if (ENC_ISMotorRunning() == 0)
                    {
                      ucMotorStateIndex = 6;
                      ucMotorState = 5;
                    }
                    break;
                  }
                //turn parallel to door
                case 6:
                  {
                    ENC_SetDistance(32, 32); //35 ticks is a perfect 90 degree turn //note 75 ticks is 180 degrees
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed + leftWheelSpeedMod;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed + rightWheelSpeedMod;
                    ucMotorState = 2; //left turn
                    ucMotorStateIndex = 7;
                    break;
                  }
                //stop
                case 7:
                  {
                    if (ENC_ISMotorRunning() == 0)
                    {
                      ucMotorStateIndex = 8;
                      ucMotorState = 5;
                    }
                    break;
                  }
                //Head along obstacle towards beacon
                case 8:
                  {
                    ENC_SetDistance(250, 250); //100 ticks means it moves forward 37cm
                    ucMotorState = 1;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed + leftWheelSpeedMod;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed + rightWheelSpeedMod;
                    ucMotorStateIndex = 9;
                    break;
                  }
                //Stop after end of obstacle
                case 9:
                  {
                    if (ENC_ISMotorRunning() == 0)
                    {
                      ucMotorStateIndex = 10;
                      ucMotorState = 5;
                    }
                    break;
                  }
                case 10: //look for beacon
                  {
                    if (CR1_ui8IRDatum == 0x55 && beaconHit == false)
                    {
                      ucMotorStateIndex2 = 1;
                    }
                    else if (CR1_ui8IRDatum == 0x41 && beaconHit == false)
                    {
                      ucMotorStateIndex2 = 2;
                      beaconHit = true;
                    }
                    else
                    {
                      if (beaconHit == false)
                      {
                        ucMotorStateIndex2 = 0;
                      }
                    }
                    switch (ucMotorStateIndex2)
                    {
                      //Scan for beacon
                      case 0:
                        {
                          ENC_SetDistance(1, 1);
                          ucMotorState = 2;
                          CR1_ui8LeftWheelSpeed = 135;
                          CR1_ui8RightWheelSpeed = 135;
                          break;
                        }

                      //Head towards beacon
                      case 1:
                        {
                          ENC_SetDistance(10, 10);
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          break;
                        }

                      //Stop at beacon
                      case 2:
                        {
                          ucMotorState = 0;
                          ucMotorStateIndex2 = 3;
                          break;
                        }
                      //Start turning motor
                      case 3:
                      {
                          startClimb = millis(); 
                          startMotion();
                          ucMotorStateIndex2 = 4; 
                          break;
                      }
                      case 4:
                      {
                        if (millis()- startClimb > climbInterval)
                        {
                          stopMotion();
                        }
                      }
                      
                    }
                  }
              }
            }
          }
          CR1_ucMainTimerCaseCore1 = 1;
          break;
        }
      //###############################################################################
      case 1:
        {
          //Hardcode motor speed
          CR1_ui8WheelSpeed = 150;  // adjust to range that will produce motion

          CR1_ucMainTimerCaseCore1 = 2;
          break;
        }
      //###############################################################################
      case 2:
        {
          // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS

          //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS

          CR1_ucMainTimerCaseCore1 = 3;
          break;
        }
      //###############################################################################
      case 3:
        {
          //move bot X number of odometer ticks
          if (ENC_ISMotorRunning())
          {
            MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8RightWheelSpeed);
          }

          CR1_ucMainTimerCaseCore1 = 4;
          break;
        }
      //###############################################################################
      case 4:
        {

          CR1_ucMainTimerCaseCore1 = 5;
          break;
        }
      //###############################################################################
      case 5:
        {


          CR1_ucMainTimerCaseCore1 = 6;
          break;
        }
      //###############################################################################
      case 6:
        {


          CR1_ucMainTimerCaseCore1 = 7;
          break;
        }
      //###############################################################################
      case 7:
        {
          if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
            SmartLEDs.setPixelColor(0, 0, 25, 0);      // make LED1 green with 10% intensity
          }
          else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
            SmartLEDs.setPixelColor(0, 25, 0, 25);     // make LED1 purple with 10% intensity
          }
          else {                                       // otherwise
            SmartLEDs.setPixelColor(0, 25, 0, 0);      // make LED1 red with 10% intensity
          }
          SmartLEDs.show();                            // send updated colour to LEDs

          CR1_ucMainTimerCaseCore1 = 8;
          break;
        }
      //###############################################################################
      case 8:
        {

          CR1_ucMainTimerCaseCore1 = 9;
          break;
        }
      //###############################################################################
      case 9:
        {

          CR1_ucMainTimerCaseCore1 = 0;
          break;
        }

    }
  }

  // Heartbeat LED
  CR1_ulHeartbeatTimerNow = millis();
  if (CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
  {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
    // Serial.println((vui32test2 - vui32test1)* 3 );
  }

}
