#include <Arduino.h>
#include <Stream.h>
#include "Defines.h"
#include "Hoverserial.h"
#include "config.h"

#ifdef DEBUG_SERIAL
  #include <RTTStream.h>
  extern RTTStream rtt;
  extern int debug;
#endif

#define START_FRAME 0xABCD      // [-] Start frame definition for reliable serial communication

Hoverserial::Hoverserial(Stream &serial) : _serial(serial) {
    _cmd1 = 0;
    _cmd2 = 0;
    _ctrlTyp = FOC_CTRL;
    _ctrlMod = VLT_MODE;
    _fieldWeak = 0;
    _aux = 0;
    _lock = 0;
    _serial = serial;
    _idx = 0;
    _iTest = 0;
    _iStep = SPEED_STEP; 
}

void Hoverserial::setCmd1(int16_t cmd1){
  if IN_RANGE(cmd1,-1000,1000){  
    _cmd1 = cmd1;
  }
}

void Hoverserial::setCmd2(int16_t cmd2){
  if IN_RANGE(cmd2,-1000,1000){  
    _cmd2 = cmd2;
  }
}

void Hoverserial::setCtrlMod(ctrlMod_t ctrlMod){
  if IN_RANGE(ctrlMod,0,2){  
    _ctrlMod = ctrlMod;
  }
}

void Hoverserial::setCtrlTyp(ctrlTyp_t ctrlTyp){
  if IN_RANGE(ctrlTyp,0,2){  
    _ctrlTyp = ctrlTyp;
  }
}

void Hoverserial::setFieldWeak(uint8_t fieldWeak){
  if IN_RANGE(fieldWeak,0,1){  
    _fieldWeak = fieldWeak;
  }
}

void Hoverserial::setAux(uint8_t aux){
  if IN_RANGE(aux,0,1){  
    _aux = aux;
  }
}
void Hoverserial::setLock(uint8_t lock){
  if IN_RANGE(lock,0,1){  
    _lock = lock;
  }
}

void Hoverserial::send(){

  SerialCommand Command;
  uint16_t cmdSwitch   = (uint16_t)(
        _aux            | // Aux input 0:OFF 1:ON
        _ctrlTyp   << 1 | // 0:FOC 1:Sinusoidal 2:Commutation
        _ctrlMod   << 3 | // 0:VOLTAGE 1:SPEED 2:TORQUE
        _fieldWeak << 5); // Field Weakening OFF
                                     
  // Create command
  Command.start   = (uint16_t)START_FRAME;
  Command.pitch   = 0;
  Command.dPitch  = 0;
  Command.cmd1    = _cmd1;
  Command.cmd2    = _cmd2;
  Command.sensors = (uint16_t)( (cmdSwitch << 8)  | (digitalRead(SENSOR1_PIN) | (digitalRead(SENSOR2_PIN) << 1) | (0 << 2)) );
  Command.checksum = (uint16_t)(Command.start ^ Command.pitch ^ Command.dPitch ^ Command.cmd1 ^ Command.cmd2 ^ Command.sensors);
  
  // Write to Serial
  _serial.write((uint8_t *) &Command, sizeof(Command));

  #ifdef DEBUG_SERIAL
    if (debug & PRINT_SERIAL_OUT){
      // Print data to built-in Serial
      DEBUG_SERIAL.print(F("1: "));   DEBUG_SERIAL.print(Command.pitch);
      DEBUG_SERIAL.print(F(" 2: "));  DEBUG_SERIAL.print(Command.dPitch);
      DEBUG_SERIAL.print(F(" 3: "));  DEBUG_SERIAL.print(Command.cmd1);
      DEBUG_SERIAL.print(F(" 4: "));  DEBUG_SERIAL.print(Command.cmd2);
      DEBUG_SERIAL.print(F(" 5: "));  DEBUG_SERIAL.print(Command.sensors);
      DEBUG_SERIAL.print(F(" 6: "));  DEBUG_SERIAL.println(Command.checksum);
    }
  #endif

}

void Hoverserial::receive(){
    
  // Check for new data availability in the Serial buffer
  while (_serial.available()) {
    _incomingByte = _serial.read(); // Read the incoming byte
    if (_idx >= 2 && _idx < sizeof(SerialFeedback)) { // Save the new received data
      *_p++    = _incomingByte; 
      _idx++;

      // Check if we reached the end of the package
      if (_idx == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
                  _NewFeedback.start ^ 
                  _NewFeedback.cmd1 ^ 
                  _NewFeedback.cmd2 ^ 
                  _NewFeedback.speedR_meas ^ 
                  _NewFeedback.speedL_meas ^ 
                  _NewFeedback.batVoltage ^ 
                  _NewFeedback.boardTemp ^
                  _NewFeedback.cmdLed
                  );

        // Check validity of the new data
        if (_NewFeedback.start == START_FRAME && checksum == _NewFeedback.checksum) {
          // Copy the new data
          memcpy(&_Feedback, &_NewFeedback, sizeof(SerialFeedback));

          #ifdef DEBUG_SERIAL
            if (debug & PRINT_SERIAL_IN){
              // Print data to built-in Serial
              DEBUG_SERIAL.print(F("1: "));   DEBUG_SERIAL.print(_Feedback.cmd1);
              DEBUG_SERIAL.print(F(" 2: "));  DEBUG_SERIAL.print(_Feedback.cmd2);
              DEBUG_SERIAL.print(F(" 3: "));  DEBUG_SERIAL.print(_Feedback.speedR_meas);
              DEBUG_SERIAL.print(F(" 4: "));  DEBUG_SERIAL.print(_Feedback.speedL_meas);
              DEBUG_SERIAL.print(F(" 5: "));  DEBUG_SERIAL.print(_Feedback.batVoltage);
              DEBUG_SERIAL.print(F(" 6: "));  DEBUG_SERIAL.print(_Feedback.boardTemp);
              DEBUG_SERIAL.print(F(" 7: "));  DEBUG_SERIAL.println(_Feedback.cmdLed);
            }
          #endif

          // handle Leds
          digitalWrite(LED_RED_PIN,_Feedback.cmdLed & LED_RED_SET?HIGH:LOW);
          digitalWrite(LED_GREEN_PIN,_Feedback.cmdLed & LED_GREEN_SET?HIGH:LOW);
          digitalWrite(LED_YELLOW_PIN,_Feedback.cmdLed & LED_YELLOW_SET?HIGH:LOW);
          digitalWrite(LED_UP_PIN,_Feedback.cmdLed & LED_UP_SET?HIGH:LOW);
          digitalWrite(LED_DOWN_PIN,_Feedback.cmdLed & LED_DOWN_SET?HIGH:LOW);
            
        } else {
          #ifdef DEBUG_SERIAL
            if (debug & PRINT_SERIAL_IN){
              DEBUG_SERIAL.println(F("Non-valid data skipped"));
            }
          #endif
        }
        _idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
      }
    }else{
      _bufStartFrame	= ((uint16_t)(_incomingByte) << 8) | _incomingBytePrev; // Construct the start frame
      // Copy received data
      if (_bufStartFrame == START_FRAME) { // Initialize if new data is detected
        _p    = (byte *)&_NewFeedback;
        *_p++ = _incomingBytePrev;
        *_p++ = _incomingByte;
        _idx  = 2;	
      }
    }
    // Update previous states
    _incomingBytePrev = _incomingByte;
  }
}

void Hoverserial::test(){
  setCmd2(_iTest);
  // Calculate test command signal
  _iTest += _iStep;

  // invert step if reaching limit
  if (_iTest >= SPEED_MAX_TEST || _iTest <= -SPEED_MAX_TEST)
    _iStep *= -1;
}