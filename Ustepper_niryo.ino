/*
*			Example sketch for PID position control!
*
*	This example demonstrates how easy closed loop position control can be achieved using the uStepper !
*	The only thing needed to activate closed loop control, is in the stepper.setup() function, where the
*	object is initiated with the keyword "PID", followed by the microstepping setting, faultTolerance (in steps),
*	hysteresis (in steps) and the P, I and D coefficients. For more information, check out the documentation:
*	http://ustepper.com/docs/html/index.html
*
*	Once the PID is activated, the use of the library is the same as without the PID !
*
*/

#include <uStepper.h>
#include <SPI.h>
#include <Arduino.h>
#include "config.h"
#include "StepperController.h"
#include "CanBus.h"
#include "data.h"
#include <EEPROM.h>
#include <uStepper.h>
#include <Arduino.h>
#define SerialDebug Serial
uint8_t motor_id = 6;
uStepper stepper;
StepperController stepper1(&stepper);
MCP_CAN can_driver(8);
CanBus canBus(&can_driver, &stepper1);
unsigned long timebegin=0;
void parseData();
void nhanuart();
void movesteppertopos(int32_t vitri);
bool loadStepConf();
void saveStepConf(void);


unsigned int driver_temperature = 530;

unsigned long time_last_write_position = micros();
unsigned long write_frequency_position = 50000; // 12.5Hz

unsigned long time_last_write_diagnostics = micros();
unsigned long write_frequency_diagnostics = 20000000; // 0.5Hz

//unsigned long time_last_read_temperature = micros();
//unsigned long read_temperature_frequency = 2000000; // 0.5 Hz
//unsigned long read_temperature_frequency = 2000000; // 0.5 Hz


unsigned long time_last_write_firmware_version = micros();
unsigned long write_frequency_firmware_version = 10000000; // 0.2 Hz
unsigned long time_begin_debug_serial = micros();
void setup(void)
{
  Serial.begin(115200);

 Serial.println("Begin");
 
  EEPROM.begin();
 loadStepConf();
 canBus.setup(motor_id);
 //canBus.setting_id();
 stepper.setup(); 

  //stepper.setup(PID,SIXTEEN,10,5,1.0,0.02,0.006);     //Initiate the stepper object to use closed loop PID control QUARTER
                                                      //The behaviour of the controller can be adjusted by tuning 
                                                        //the P, I and D paramenters in this initiation (the last three parameters)
                                                        //Also the hysteresis and faultTolerance can be adjusted by adjusting 
                                                        //parameter 3 and 4. For more information, check out the documentation:
                                                        //http://ustepper.com/docs/html/index.html
    stepper.setMaxAcceleration(DataConf.MAXACCEL);
  stepper.setMaxVelocity(DataConf.MAXVELOC);
 stepper.encoder.setHome();
 //stepper1.setControlMode(STEPPER_CONTROL_MODE_STANDARD);
 stepper1.setControlMode(STEPPER_CONTROL_MODE_RELAX);
 timebegin= micros();
 
}
bool action_available = true;
void loop(void)
{ nhanuart();
  /*if (micros() - timebegin >= 3000000) {
    timebegin=micros();
    Serial.print("Encode pos :");
    Serial.println((stepper.encoder.getAngleMoved()*16*200)/360); 
        Serial.print("Step pos :");
    Serial.println(stepper.getStepsSinceReset()); 
    Serial.print("AngleMoved : ");
    Serial.println(stepper.encoder.getAngleMoved()); 
   Serial.print("Angle : ");
   Serial.println(stepper.encoder.getAngle());
   Serial.print("Sp : "); 
    Serial.println(stepper.encoder.getSpeed());  
   Serial.print("Magnet : "); 
    Serial.println(stepper.encoder.detectMagnet());
    Serial.println("------------");
  }*/
  action_available = true;
stepper1.update();

  // read position from sensor
 // update_current_position(stepper.getMicroSteps());

  // read CAN if available
  if (action_available) {
    if(canBus.available()) 
    {//Serial.println("read");
      canBus.read();
      action_available = false;
    }
  }
 
  // write position CAN
  if (action_available) {
    if (micros() - time_last_write_position > write_frequency_position) {
      time_last_write_position += write_frequency_position;
      canBus.writePosition();
      action_available = false;
    }
  }

  // write diagnostics CAN
  if (action_available) {
    if (micros() - time_last_write_diagnostics > write_frequency_diagnostics) {
      time_last_write_diagnostics += write_frequency_diagnostics;
      canBus.writeDiagnostics(stepper1.getControlMode(), driver_temperature);
      action_available = false;
    }
  }

  // write firmware version
  if (action_available) {
    if (micros() - time_last_write_firmware_version > write_frequency_firmware_version) {
      
      time_last_write_firmware_version += write_frequency_firmware_version;
      canBus.writeFirmwareVersion(NIRYO_STEPPER_VERSION_MAJOR, NIRYO_STEPPER_VERSION_MINOR, NIRYO_STEPPER_VERSION_PATCH);
      action_available = false;
    }
  }
}
