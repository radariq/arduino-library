//-------------------------------------------------------------------------------------------------
//                                                                            				           --
//                             			RadarIQ C-SDK Demo Application                               --
//                                         'Proximity Sensor'                                 	   --
//                   		        (C) 2021 RadarIQ <support@radariq.io>                    			   --
//                                                                            					         --
//                            			        License: MIT                                    	   --
//                                                                            					         --
//------------------------------------------------------------------------------------------------- 


//-------------------------------------------------------------------------------------------------
// Introduction
//-------------
//
// This Application demonstrates the majority of the RadarIQ API commands
// and demonstrates the ability of the sensor to act as a basic proximity sensor.
//
// When the sensor powers on, the sensor is configured with the relevant settings
// A scene calibration is then run. The purpose of the scene calibration is to remove the effect of
// any objects which are within 1m of the sensor. This provides a clear environment for the proximity
// sensing.
//
// When an object is moved within 1m of the RadarIQ sensor the on board LED will begin flashing and the
// approximate distance of the closest reflection is outputted over serial. The closer the reflection,
// the faster the LED will blink.

//-------------------------------------------------------------------------------------------------
// Instructions
//-------------
// These instructions are for the Arduino MEGA. Adjust as necessary for other types of Arduino.
//
// An external power supply should be used to DIRECTLY power the RadarIQ sensor because 
// some Arduinos are not able to supply enough instantaneous current.
//
// Connections:
// RED = 5V
// BLACK = GND
// BLUE = RX1
// GREEN = TX1

// Place the sensor on a flat surface and power the sensor on.
// Wave your hand over the sensor and the on board LED should begin flashing
//
// Additional documentation can be found at docs.radariq.io
//

//-------------------------------------------------------------------------------------------------
// Defines
//----------      

// Frame rate that the radar will run at in frames/second
#define RADAR_FRAME_RATE   8u

// For use of the assert() macro - must be before including assert.h
#define __ASSERT_USE_STDERR 

//-------------------------------------------------------------------------------------------------
// Includes
//----------

#include <assert.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include "RadarIQ.h"

//-------------------------------------------------------------------------------------------------
// Objects
//---------

// The RadarIQ object instance
static RadarIQHandle_t myRadar;

//-------------------------------------------------------------------------------------------------
// Function Prototypes
//---------------------

// Callbacks used by the RadarIQ object
static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static RadarIQUartData_t callbackReadSerialData(void);
static void callbackRadarLog(char * const buffer);
static uint32_t callbackMillis(void);

//-------------------------------------------------------------------------------------------------
// Setup Function
//-------------------------------------------------------------------------------------------------

/**
 * This function will run once on power-up or reset of your board
 */
void setup()
{
  // Setup the on-board LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Setup serial ports
  Serial.begin(115200);   // PC USB debug
  Serial1.begin(115200);  // RadarIQ device

  // Create buffer for debug printing
  char buffer[128];

  Serial.print("* Heat-map started\n\r");

  // Create the RadarIQ object
  myRadar = RadarIQ_init(callbackSendRadarData, callbackReadSerialData, callbackRadarLog, callbackMillis);
  sprintf(buffer, "* Created RadarIQ instance, using %u bytes of memory\n\r", RadarIQ_getMemoryUsage());
  Serial.print(buffer);

  // Set frame rate
  if (RadarIQ_setFrameRate(myRadar, RADAR_FRAME_RATE) == RADARIQ_RETURN_VAL_OK)
  {
    uint8_t frameRate;
    if (RadarIQ_getFrameRate(myRadar, &frameRate) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Frame rate is set to %u\n\r", frameRate);
      Serial.print(buffer);
    }
    else
    {
      Serial.print("* Error reading frame rate from RadarIQ module\n");
    }
  }
  else
  {
    Serial.print("* Error setting frame rate of RadarIQ module\n\r");  
  }

  // Set capture mode to point-cloud
  if (RadarIQ_setMode(myRadar, RADARIQ_MODE_HEAT_MAP) == RADARIQ_RETURN_VAL_OK)
  {
    RadarIQCaptureMode_t mode;
    if (RadarIQ_getMode(myRadar, &mode) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Capture mode is set to: %i\n\r", mode);
      Serial.print(buffer);
    }
    else
    {
      Serial.print("* Error reading capture mode from RadarIQ module\n\r");  
    }
  }
  else
  {
    Serial.print("* Error setting capture mode of RadarIQ module\n\r");  
  }

  // Send start capture command to capture frames continuously
  RadarIQ_start(myRadar, 0);
}

//-------------------------------------------------------------------------------------------------
// Loop Function
//-------------------------------------------------------------------------------------------------

/**
 * This function will run repeatedly after the setup() function has run
 */
void loop()
{
  // Create variables
  char buffer[256];                       // Buffer for debug printing
  static int16_t yMin = 9999;             // Nearest detected point distance from radar
  static int32_t ledPeriod = -1;          // LED blinking period in milliseconds
  static int32_t ledLastToggleTime = 0;   // Last time the LED was toggled in milliseconds
  static int32_t radarLastFrameTime = 0;  // Last time a point-cloud frame was recieved in millisecons

  // Process the serial data
  RadarIQCommand_t packet = RadarIQ_readSerial(myRadar);
  
  // Check recieved packet type
  switch (packet)
  {
    // Pointcloud frame
    case RADARIQ_CMD_HEAT_MAP_FRAME:
    {
      // Get a copy of the current radar frame data
      RadarIQData_t radarData;
      RadarIQ_getData(myRadar, &radarData);
      
      Serial.print("Frame ");
      Serial.println(radarData.heatMap.numPoints);

      if (radarData.heatMap.isFrameComplete)
      {
        Serial.print("DONE ");
        Serial.println(radarData.heatMap.heatMapSize);
      }

    }
    // Ignore all other packet types
    default:
    {
      break;
    }    
  }
}

//-------------------------------------------------------------------------------------------------
// Radar Callback Functions
//--------------------------

/**
 * This callback function sends a buffer of data to the radar over the serial port
 */
static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{
    Serial1.write(buffer, len);
}

/**
 * This callback function checks for and reads data recieved from the radar on the serial port
 */
static RadarIQUartData_t callbackReadSerialData()
{ 
  RadarIQUartData_t ret;
  
  ret.isReadable = (Serial1.available() > 0);
  if (ret.isReadable)
  {
    int data = Serial1.read();
    if (data < 0)
    {
      ret.isReadable = false;
    }
    else
    {
      ret.data = (uint8_t)data;
    }
  }
  
  return ret;
}

/**
 * This callback prints debug log messages created from the RadarIQ object
 */
static void callbackRadarLog(char * const buffer)
{
  Serial.println(buffer);    
}

/**
 * This callback function lets the RadarIQ object read the current uptime in milliseconds
 */
static uint32_t callbackMillis(void)
{
  return millis() & 0xFFFFFFFF;
}

//-------------------------------------------------------------------------------------------------
// Assertion Handler
//-------------------

/**
 * This funtion will be called if an assert() condition fails
 * Debug information is printed, then the program will reset by forcing the watchdog to timeout
 */
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp)
{
  delay(100);
  char buffer[256];
  sprintf(buffer, "ASSERTION FAILED [%s]: File: %s, Line: %i, Func: %s\n\r", __sexp, __file, __lineno, __func);
  Serial.print(buffer);
  //delay(100);

  wdt_disable();
  //wdt_enable(WDTO_15MS);
  //while(1);
}
