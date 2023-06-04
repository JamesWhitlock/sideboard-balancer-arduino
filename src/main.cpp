#include <Arduino.h>
#include <Defines.h>
#include <Config.h>

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
  int debug = 0;
#endif

#ifdef MPU6050
  #include <Adafruit_MPU6050.h>
  Adafruit_MPU6050 mpu;
  #define IMU MPU6050
#endif

#ifdef ICM20948
  #include <Adafruit_ICM20948.h>
  #include <Adafruit_ICM20X.h>
  Adafruit_ICM20948 mpu;
  #define IMU ICM20948
#endif

#ifdef IMU
#include <Adafruit_AHRS.h>
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
#endif //IMU

// ########################## SETUP ##########################
void setup(){
  
  // Define Leds as output
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);

  // Turn all Leds ON for half a second and then OFF
  digitalWrite(LED1_PIN,HIGH);
  digitalWrite(LED2_PIN,HIGH);
  digitalWrite(LED3_PIN,HIGH);
  digitalWrite(LED4_PIN,HIGH);
  digitalWrite(LED5_PIN,HIGH);
  delay(500);
  digitalWrite(LED1_PIN,LOW);
  digitalWrite(LED2_PIN,LOW);
  digitalWrite(LED3_PIN,LOW);
  digitalWrite(LED4_PIN,LOW);
  digitalWrite(LED5_PIN,LOW);
 
  // Define Sensors as input
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
  #endif

  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println(F("Hoverboard Serial v1.0"));
  #endif

  #ifdef REMOTE
    mySwitch.enableReceive(digitalPinToInterrupt(RECEIVER_PIN));  // Receiver on interrupt
  #endif
  
  #ifdef IMU
    #ifdef MPU6050
    if (!mpu.begin()){
    #endif
    #ifdef ICM20948
    if (!mpu.begin_I2C()){
    #endif
      digitalWrite(LED1_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("Failed to find IMU chip"));
      #endif
    }else{
      digitalWrite(LED2_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("IMU found!"));
      #endif

      mpu_temp = mpu.getTemperatureSensor();
      #ifdef DEBUG_SERIAL
        mpu_temp->printSensorDetails();
      #endif

      mpu_accel = mpu.getAccelerometerSensor();
      #ifdef DEBUG_SERIAL
        mpu_accel->printSensorDetails();
      #endif

      mpu_gyro = mpu.getGyroSensor();
      #ifdef DEBUG_SERIAL
        mpu_gyro->printSensorDetails();
      #endif
    }
  #endif //IMU 
}


// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

#ifdef TEST
int iTest = 0;
int iStep = SPEED_STEP;
#endif

void loop(){

  #ifdef HOVER_SERIAL
    // Check for new received data
    hoverserial.receive();
  #endif

  #ifdef DEBUG_SERIAL
    // Process debug commands
    if(DEBUG_SERIAL.available() > 0) {
      char buf[SERIAL_RX_BUFFER_SIZE + 1];
      size_t numRead = DEBUG_SERIAL.readBytes(buf, sizeof(buf));
      buf[numRead] = '\0';
      if (buf[1] == '\n'){
        switch (buf[0]){
          case 'h':
            DEBUG_SERIAL.println(F("=== HELP ==="));
            DEBUG_SERIAL.println(F("h: Print Help"));
            #ifdef HOVER_SERIAL
              DEBUG_SERIAL.println(F("s: Print Serial"));
            #endif             
            
            #ifdef REMOTE
              DEBUG_SERIAL.println(F("r: Print Remote"));
            #endif
            
            #ifdef IMU
              DEBUG_SERIAL.println(F("a: Print Accelerometer"));
              DEBUG_SERIAL.println(F("g: Print Gyroscope"));
              DEBUG_SERIAL.println(F("q: Print Quaternion"));
              DEBUG_SERIAL.println(F("e: Print Euler angles"));
              DEBUG_SERIAL.println(F("t: Print Temperature"));
            #endif // IMU

            DEBUG_SERIAL.println(F("============"));
            break;
          #ifdef IMU  
          case 'a':
            debug ^= PRINT_ACCEL;
            break;
          case 'g':
            debug ^= PRINT_GYRO;
            break;
          case 'q':
            debug ^= PRINT_QUAT;
            break;
          case 'e':
            debug ^= PRINT_EULER;
            break;
          case 't':
            debug ^= PRINT_TEMP;
            break;
          #endif //IMU

          #ifdef HOVER_SERIAL
          case 's':
            debug ^= PRINT_SERIAL;
            break;
          #endif //HOVERSERIAL

          #ifdef REMOTE
          case 'r':
            debug ^= PRINT_REMOTE;
            break;
           #endif //REMOTE
        }
      }
    }
  #endif

  // Delay
  unsigned long timeNow = millis();
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
 
  #ifdef IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    #ifdef DEBUG_SERIAL
      if (debug & PRINT_TEMP){
        DEBUG_SERIAL.print(F("Temperature "));
        DEBUG_SERIAL.print(temp.temperature);
        DEBUG_SERIAL.println(F(" deg C"));
      }
      
      if (debug & PRINT_ACCEL){
        /* Display the results (acceleration is measured in m/s^2) */
        DEBUG_SERIAL.print(F("Accel X: "));
        DEBUG_SERIAL.print(accel.acceleration.x);
        DEBUG_SERIAL.print(F(" Y: "));
        DEBUG_SERIAL.print(accel.acceleration.y);
        DEBUG_SERIAL.print(F(" Z: "));
        DEBUG_SERIAL.print(accel.acceleration.z);
        DEBUG_SERIAL.println(F(" m/s^2 "));
      }

      if (debug & PRINT_GYRO){
        /* Display the results (rotation is measured in rad/s) */
        DEBUG_SERIAL.print(F("Gyro X: "));
        DEBUG_SERIAL.print(gyro.gyro.x);
        DEBUG_SERIAL.print(F(" Y: "));
        DEBUG_SERIAL.print(gyro.gyro.y);
        DEBUG_SERIAL.print(F(" Z: "));
        DEBUG_SERIAL.print(gyro.gyro.z);
        DEBUG_SERIAL.println(F(" radians/s "));
      }
    #endif
  #endif //IMU

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

      switch(mySwitch.getReceivedValue()){
        case REMOTE_BUTTON1:
          DEBUG_SERIAL.println(F("Button 1 pressed"));
          hoverserial.setCtrlTyp(FOC_CTRL);
          break;
        case REMOTE_BUTTON2:
          DEBUG_SERIAL.println(F("Button 2 pressed"));
          hoverserial.setCtrlTyp(SIN_CTRL);
          break;
        case REMOTE_BUTTON3:
          DEBUG_SERIAL.println(F("Button 3 pressed"));
          hoverserial.setCtrlTyp(COM_CTRL);
          break;
        case REMOTE_BUTTON4:
          DEBUG_SERIAL.println(F("Button 4 pressed"));
          break;
      }

      mySwitch.resetAvailable();
    }
  #endif

  #ifdef HOVER_SERIAL
    #ifdef TEST
      hoverserial.setCmd2(iTest);
      // Calculate test command signal
      iTest += iStep;

      // invert step if reaching limit
      if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
        iStep *= -1;
    #else
      hoverserial.setCmd1(0);
      hoverserial.setCmd2(0);
    #endif
    hoverserial.send();
  #endif
}