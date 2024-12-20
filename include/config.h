// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H


// SEGGER RTT Debug
#define DEBUG_SERIAL rtt // Uncomment to enable DEBUG
#ifdef DEBUG_SERIAL
  #undef BUFFER_SIZE_UP
  #define BUFFER_SIZE_UP (512)  // Size of the buffer for terminal output of target, up to host (Default: 1k)
#endif

// Hover serial protocol
// #define HOVER_SERIAL                    // Send commands to the mainboard and receive feedback
#ifdef HOVER_SERIAL
  #define HOVER_SERIAL_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

// Test
// #define TEST                            // Will send test commands to the mainboard, HOVER_SERIAL should also be enabled

#define LOOP_PERIOD_US        100000    // [us] Sending time interval (minimum circa 2500us / 400Hz)

// 433Mhz RF Remote
//#define REMOTE                          // Uncomment to enable 433Mhz Receiver. 
#ifdef REMOTE 
  #define REMOTE_BUTTON1 6637793        // Switch to FOC control type
  #define REMOTE_BUTTON2 6637794        // Switch to SIN control type
  #define REMOTE_BUTTON3 6637796        // Switch to COM control type
  #define REMOTE_BUTTON4 6637800        // Does nothing for now
#endif 

#define AHRS
#define ENABLE_IMU

// Crossfire protocol support
// #define CRSF                            // Uncomment to enable CRSF telemetry
#ifdef CRSF
  #define CRSF_PIN_TX         1           // [-] Pin for CRSF telemetry TX
  #define CRSF_PIN_RX         3           // [-] Pin for CRSF telemetry RX
  #define CRSF_BAUD           57600     // [-] Baud rate for CRSF telemetry
#endif

#endif //  CONFIG_H
