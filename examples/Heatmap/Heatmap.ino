//-------------------------------------------------------------------------------------------------
//                                                                            				           --
//                             			RadarIQ C-SDK Demo Application                               --
//                                            'Heat-Map'                                 	       --
//                   		        (C) 2021 RadarIQ <support@radariq.io>                    			   --
//                                                                            					         --
//                            			        License: MIT                                    	   --
//                                                                            					         --
//------------------------------------------------------------------------------------------------- 


//-------------------------------------------------------------------------------------------------
// Introduction
//-------------


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

#define HEATMAP_PRINT_BUFFER_SIZE 256u  ///< Size of string for printing points
#define HEATMAP_POINTS_PER_PRINT  25u   ///< Number of points allowed in string (allows 10 chars per real+imag point)

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
  static bool framePrinted;

  // Process the serial data
  RadarIQCommand_t packet = RadarIQ_readSerial(myRadar);

  // Check recieved packet type
  if ((packet == RADARIQ_CMD_HEAT_MAP_FRAME) && (!framePrinted))
  { 
    // Get a copy of the current radar frame data
    RadarIQData_t radarData;
    RadarIQ_getData(myRadar, &radarData);

    // Print all frame data at end of frame
    if (radarData.heatMap.isFrameComplete)
    {    
      Serial.println("FRAME READY");

      char pointBuffer[16];
      char publishBuffer[HEATMAP_PRINT_BUFFER_SIZE];
      memset(publishBuffer, 0, sizeof(publishBuffer));  

      uint32_t point;
      for (point = 0u; point < RADARIQ_MAX_HEATMAP; point++)
      {
        sprintf(pointBuffer, "%i,%i,", radarData.heatMap.points[point].real, radarData.heatMap.points[point].imaginary);
        strcat(publishBuffer, pointBuffer);

        // Check if correct number of points to print line
        if ((point % HEATMAP_POINTS_PER_PRINT) == (HEATMAP_POINTS_PER_PRINT - 1u))
        {
          Serial.println(publishBuffer);
          memset(publishBuffer, 0, sizeof(publishBuffer)); 
        }
      }

      // Check if there are remaining points to print
      if ((point % HEATMAP_POINTS_PER_PRINT) != (HEATMAP_POINTS_PER_PRINT - 1u))
      {
        Serial.println(publishBuffer);
        memset(publishBuffer, 0, sizeof(publishBuffer)); 
      }

      framePrinted = true;
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
