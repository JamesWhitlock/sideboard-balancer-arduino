#include <Arduino.h>
#include <Defines.h>
#include <Config.h>
#include <SimpleFOC.h>


#ifdef HOVER_SERIAL
  #include <Hoverserial.h>
  Hoverserial hoverserial(Serial2);
#endif

#ifdef REMOTE
  #include <RCSwitch.h>
  RCSwitch mySwitch = RCSwitch();
#endif

#ifdef DEBUG_SERIAL
  #include <RTTStream.h>
  RTTStream rtt;
  Commander commander = Commander(rtt);
  int debug = 0;
#endif

#ifdef ENABLE_IMU
#ifdef AHRS
  #include "Adafruit_AHRS_Madgwick.h"
  Adafruit_Madgwick filter;
#endif

#endif //IMU

// Callbacks for custom commander commands
#ifdef DEBUG_SERIAL
  #ifdef HOVER_SERIAL
  void onCmdSerialIn(char* cmd){ debug ^= PRINT_SERIAL_IN; }
  void onCmdSerialOut(char* cmd){ debug ^= PRINT_SERIAL_OUT; }
  #endif
  #ifdef REMOTE
  void onCmdRemote(char* cmd){ debug ^= PRINT_REMOTE; }
  #endif
  #ifdef ENABLE_IMU  
    void onCmdAccel(char* cmd){ debug ^= PRINT_ACCEL;}
    void onCmdGyro(char* cmd){ debug ^= PRINT_GYRO;}
    void onCmdTemp(char* cmd){ debug ^= PRINT_TEMP; }
    #ifdef AHRS
      void onCmdQuat(char* cmd){ debug ^= PRINT_QUAT; }
      void onCmdEuler(char* cmd){ debug ^= PRINT_EULER; }
    #endif    
  #endif //ENABLE_IMU
#endif

#ifdef ENABLE_IMU
  #include <FastIMU.h>
  #include "imu_factory.h"
  IMUBase* imu;
#endif //ENABLE_IMU

#ifdef CRSF
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
#endif

extern TwoWire Wire1;

unsigned long loop_start_us = 0;

// ########################## SETUP ##########################
void setup(){
  // Define Leds as output
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_UP_PIN, OUTPUT);
  pinMode(LED_DOWN_PIN, OUTPUT);

  // Turn all Leds ON for half a second and then OFF
  digitalWrite(LED_RED_PIN,HIGH);
  digitalWrite(LED_GREEN_PIN,HIGH);
  digitalWrite(LED_YELLOW_PIN,HIGH);
  digitalWrite(LED_UP_PIN,HIGH);
  digitalWrite(LED_DOWN_PIN,HIGH);
  delay(500);
  digitalWrite(LED_RED_PIN,LOW);
  digitalWrite(LED_GREEN_PIN,LOW);
  digitalWrite(LED_YELLOW_PIN,LOW);
  digitalWrite(LED_UP_PIN,LOW);
  digitalWrite(LED_DOWN_PIN,LOW);
 
  // Define Sensors as input
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  Wire1.begin();

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
  #endif

  #ifdef REMOTE
    mySwitch.enableReceive(digitalPinToInterrupt(RECEIVER_PIN));  // Receiver on interrupt
  #endif
  
  #ifdef ENABLE_IMU
    imu = GetImu(Wire1);
    if (imu){
      DEBUG_SERIAL.print(F("IMU found: "));
      DEBUG_SERIAL.println(imu->IMUName());
      digitalWrite(LED_GREEN_PIN,HIGH);
      calData cal = {0};
      imu->init(cal, 0x68); // TODO: GetImu should return the address
    }
    else {
      DEBUG_SERIAL.println(F("No IMU found"));
      digitalWrite(LED_RED_PIN,HIGH);
    }
  #endif //ENABLE_IMU 


  // Add custom commander commands
  #ifdef DEBUG_SERIAL
    #ifdef HOVER_SERIAL
      commander.add('i',onCmdSerialIn,"Print Serial In");
      commander.add('o',onCmdSerialOut,"Print Serial Out");
    #endif
    #ifdef REMOTE
      commander.add('r',onCmdRemote,"Print Remote");
    #endif

    #ifdef ENABLE_IMU
      commander.add('a',onCmdAccel,"Print Accelerometer");
      commander.add('g',onCmdGyro,"Print Gyroscope");
      if (imu && imu->hasTemperature()){
      commander.add('t',onCmdTemp,"Print Temperature");
      }
      #ifdef AHRS
        commander.add('q',onCmdQuat,"Print Quaternions");
        commander.add('e',onCmdEuler,"Print Euler Angles");
      #endif
    #endif
    DEBUG_SERIAL.println(F("Hoverboard Serial v1.0"));
  #endif //DEBUG_SERIAL
  
  loop_start_us = micros();
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(){

  #ifdef HOVER_SERIAL
    // Check for new received data
    hoverserial.receive();
  #endif

  #ifdef DEBUG_SERIAL
    commander.run(); // reads Serial instance form constructor
  #endif
  
  #ifdef ENABLE_IMU
    AccelData accel;
    GyroData gyro;

    if (imu){ 
      imu->update();
      imu->getAccel(&accel);
      imu->getGyro(&gyro);

      #ifdef AHRS
        filter.updateIMU(gyro.gyroX,
                         gyro.gyroY,
                         gyro.gyroZ,
                         accel.accelX,
                         accel.accelY,
                         accel.accelZ);
      #endif
    }
  #endif //ENABLE_IMU
  
  #ifdef ENABLE_IMU
    if (imu){
      #ifdef DEBUG_SERIAL
        if (debug & PRINT_TEMP){
          DEBUG_SERIAL.print(F("Temperature "));
          DEBUG_SERIAL.print(imu->getTemp());
          DEBUG_SERIAL.println(F(" deg C"));
        }
        
        if (debug & PRINT_ACCEL){
          /* Display the results (acceleration is measured in m/s^2) */
          DEBUG_SERIAL.print(F("Accel X: "));
          DEBUG_SERIAL.print(accel.accelX);
          DEBUG_SERIAL.print(F(" Y: "));
          DEBUG_SERIAL.print(accel.accelY);
          DEBUG_SERIAL.print(F(" Z: "));
          DEBUG_SERIAL.print(accel.accelZ);
          DEBUG_SERIAL.println(F(" m/s^2 "));
        }

        if (debug & PRINT_GYRO){
          /* Display the results (rotation is measured in rad/s) */
          DEBUG_SERIAL.print(F("Gyro X: "));
          DEBUG_SERIAL.print(gyro.gyroX);
          DEBUG_SERIAL.print(F(" Y: "));
          DEBUG_SERIAL.print(gyro.gyroY);
          DEBUG_SERIAL.print(F(" Z: "));
          DEBUG_SERIAL.print(gyro.gyroZ);
          DEBUG_SERIAL.println(F(" radians/s "));
        }

        #ifdef AHRS
          float qw, qx, qy, qz;
          float roll, pitch, yaw;

          if (imu->hasQuatOutput() && debug & PRINT_TEMP){
            Quaternion quat; 
            imu->getQuat(&quat);
            qw=quat.qW;
            qx=quat.qX;
            qy=quat.qY;
            qz=quat.qZ;
            std::tie(roll, pitch, yaw) = QuatToEuler(qw, qx, qy, qz);
          } else {
            filter.getQuaternion(&qw, &qx, &qy, &qz);
            roll = filter.getRoll();
            pitch = filter.getPitch();
            yaw = filter.getYaw();
          }

          if (debug & PRINT_QUAT){
            DEBUG_SERIAL.print(F("qW: "));
            DEBUG_SERIAL.print(qw);
            DEBUG_SERIAL.print(F(" qX: "));
            DEBUG_SERIAL.print(qx);
            DEBUG_SERIAL.print(F(" qY: "));
            DEBUG_SERIAL.print(qy);
            DEBUG_SERIAL.print(F(" qZ: "));
            DEBUG_SERIAL.println(qz);
          }

          if (debug & PRINT_EULER){
            DEBUG_SERIAL.print(F("Roll: "));
            DEBUG_SERIAL.print(filter.getRoll());
            DEBUG_SERIAL.print(F(" Pitch: "));
            DEBUG_SERIAL.print(filter.getPitch());
            DEBUG_SERIAL.print(F(" Yaw: "));
            DEBUG_SERIAL.println(filter.getYaw());
          }
        #endif //AHRS
      #endif //DEBUG_SERIAL
    }
  #endif //ENABLE_IMU

  #ifdef REMOTE
    if (mySwitch.available()) {
      #ifdef DEBUG_SERIAL
        if (debug & PRINT_REMOTE){
          DEBUG_SERIAL.print(F("Received "));
          DEBUG_SERIAL.print(mySwitch.getReceivedValue());
          DEBUG_SERIAL.print(F(" / "));
          DEBUG_SERIAL.print(mySwitch.getReceivedBitlength());
          DEBUG_SERIAL.print(F("bit "));
          DEBUG_SERIAL.print(F("Protocol: "));
          DEBUG_SERIAL.println(mySwitch.getReceivedProtocol());
        }
      #endif

      int button_pressed = 0;
      switch(mySwitch.getReceivedValue()){
        case REMOTE_BUTTON1:
          button_pressed = 1;
          hoverserial.setCtrlTyp(FOC_CTRL);
          break;
        case REMOTE_BUTTON2:
          button_pressed = 2;
          hoverserial.setCtrlTyp(SIN_CTRL);
          break;
        case REMOTE_BUTTON3:
          button_pressed = 3;
          hoverserial.setCtrlTyp(COM_CTRL);
          break;
        case REMOTE_BUTTON4:
          button_pressed = 4;
          break;
      }

      #ifdef DEBUG_SERIAL 
        if (button_pressed > 0){
          DEBUG_SERIAL.print(F("Button "));
          DEBUG_SERIAL.print(button_pressed); 
          DEBUG_SERIAL.println(F(" pressed"));
        }
      #endif

      mySwitch.resetAvailable();
    }
  #endif

  #ifdef HOVER_SERIAL
    #ifdef TEST
      hoverserial.test();
    #else
      hoverserial.setCmd1(0);
      hoverserial.setCmd2(0);
    #endif
    hoverserial.send();
  #endif

  // Delay
  uint32_t time_now_ms = micros();
  uint32_t time_diff = time_now_ms - loop_start_us;
  if (time_diff < LOOP_PERIOD_US) {
    delayMicroseconds(LOOP_PERIOD_US - time_diff);
  }
  loop_start_us = micros();
}
