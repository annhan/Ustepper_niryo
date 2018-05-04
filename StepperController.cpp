/*
    StepperController.cpp
    Copyright (C) 2017 Niryo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "StepperController.h"
volatile long step_old = 0;
boolean chieu = 0;
StepperController::StepperController(uStepper* sc)
{
  ustep = sc;
  current_step_number = 0;
  goal_step_number = 0;
 // delay_between_two_updates = STEPPER_DELAY_MIN;
  time_last_update = micros();
  time_last_step = micros();

  // default config below

  // cmd will be reached after 0.1 sec (will make trajectory smoothier if send cmd > 10 Hz)
  // if you want/need to send cmd at a lower rate, update this value
  micros_to_reach_goal = 100000; // todo get from CAN
  control_mode = STEPPER_CONTROL_MODE_RELAX;
  micro_steps = STEPPER_DEFAULT_MICRO_STEPS;
 //umax = UMAX_DEFAULT;

  steps_to_add_to_goal = 0;
}

void StepperController::reset()
{
  // not yet implemented
}

/*
   If standard mode is activated, you need to call this function at the beginning and end of trajectory
   It will do 2 things :
   - start at current sensor position if the motor missed some steps at previous trajectory
   - start and finish with a little offset to compensate a small error between current steps and real motor steps
*/
void StepperController::synchronizePosition(bool begin_trajectory)
{ current_step_number = motor_position_steps;
  ustep->updatepos(current_step_number);
  steps_to_add_to_goal = 0;

  Serial.print(motor_position_steps);
  Serial.print("-");
  Serial.println(current_step_number);
}

void StepperController::attach()
{
  steps_to_add_to_goal = 0;
  ustep->hardStop(HARD);
}

void StepperController::detach()
{
  //output(-1800 * current_step_number / micro_steps, 0);
  ustep->hardStop(SOFT);
  //fan_LOW();
}

void StepperController::start()
{
  is_enabled = true;
  time_last_update = micros();
  time_last_step = micros();
  ustep->hardStop(HARD);
}

void StepperController::stop()
{
  is_enabled = false;
  ustep->hardStop(SOFT);
}

/*
   Allows micro steps between 1 and 32
*/
void StepperController::setMicroSteps(uint8_t new_micro_steps)
{

  micro_steps = new_micro_steps;

}

/*
   Modify max effort -> amount of used current
*/
void StepperController::setMaxEffort(uint8_t effort)
{
  umax = effort;
}

void StepperController::setControlMode(uint8_t control_mode)
{
  this->control_mode = control_mode;
  if (control_mode == STEPPER_CONTROL_MODE_RELAX) {
    detach();
  }
  else if (control_mode == STEPPER_CONTROL_MODE_STANDARD) {
    attach();
  }
}

void StepperController::relativeMove(long steps, unsigned long delay)
{
  current_step_number = ustep->getStepsSinceReset();
  ustep->updatepos(current_step_number);
  goal_step_number = current_step_number + steps;
  movestepper(goal_step_number);
}

/*
   This method will :
   - set new step goal
   - calculate according delay between steps
   - if standard_mode, a small amount can be added to compensate some small errors

*/

void StepperController::movestepper(int32_t vitri)
{ int32_t vitriht = ustep->getStepsSinceReset();
  if (vitri < vitriht) {
    ustep->moveSteps(vitriht - vitri, CCW, HARD);
  }
  else {
    ustep->moveSteps(vitri - vitriht, CW, HARD);
  }
}
uint8_t StepperController::calib() // timeout in seconds
{ ustep->moveSteps(100, CW, HARD);
  unsigned long thoigian = micros();
  // Serial.println("Toi");
  do {
    // Serial.println(current_step_number);
    current_step_number = ustep->getStepsSinceReset();
    motor_position_steps = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  } while (micros() - thoigian < 3000000 && current_step_number < 100);
  if (current_step_number > 0 && motor_position_steps > 0) {
    chieu = 1;
  }
  else if (current_step_number < 0 && motor_position_steps < 0) {
    chieu = 1;
  }
  else chieu = 0;
  ustep->moveSteps(100, CCW, HARD);
  thoigian = micros();
   Serial.println("Lui");
  do {
    // Serial.println(current_step_number);
    current_step_number = ustep->getStepsSinceReset();
    motor_position_steps = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  } while (micros() - thoigian < 3000000 && current_step_number > 0);
  Serial.println("calib OK");
}

void StepperController::chay_motor(int32_t tam){
  if (tam < 0  ) {
      if (chieu > 0) ustep->moveSteps(abs(tam), CCW, HARD);
      else ustep->moveSteps(abs(tam), CW, HARD);
    }
    else
    {
      if (chieu > 0) ustep->moveSteps(abs(tam), CW, HARD);
      else ustep->moveSteps(abs(tam), CCW, HARD);
    }
}
void StepperController::setNewGoal(int32_t steps)
{
  goal_step_number = steps ;
  int32_t tam = 0;
  tam = steps - motor_position_steps;
  bool end_motor = ustep->getMotorState();
  if (tam1 != steps) {
    chay_motor(tam);
    tam1 = steps;
  }
  else if ((end_motor == 0) && (micros() - time_last_step > 2000000)) {
    time_last_step = micros();
    chay_motor(tam);
  }
}

/*
   Update controller depending on control mode

   RELAX mode :
   - disable motor, and copy sensor position to goal position

   STANDARD mode :
   - motor is powered with a constant current.
   - it will follow the command step by step with precision
   - no protection against missed steps (if many) during a trajectory, but
   - every trajectory will correct previous trajectory missed steps
   --> you need to call synchronizePosition(1) at the beginning of any given trajectory
   --> you need to call synchronizePosition(0) at the end of any given trajectory

*/
void StepperController::update()
{
  if (control_mode == STEPPER_CONTROL_MODE_RELAX) {
    relaxModeUpdate();
  }
  else if (control_mode == STEPPER_CONTROL_MODE_STANDARD) {
    standardModeUpdate();
  }
}

void StepperController::relaxModeUpdate()
{
  time_last_update = micros();
  motor_position_steps = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  current_step_number = motor_position_steps;
  goal_step_number = motor_position_steps;
  detach();
}

void StepperController::standardModeUpdate()
{
  current_step_number = ustep->getStepsSinceReset();
  motor_position_steps = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  //Serial.print(current_step_number);
  //Serial.print(":");
  //Serial.println(motor_position_steps);
}

uint8_t StepperController::calibrate() // timeout in seconds
{
  bool calibration_ok = false;
  int miss_steps_counter = 0;
  int MISS_STEPS_TRESHOLD = 10;
  long time_begin_calibration = micros();

  long last_motor_position_steps1 = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  long last_motor_position_steps = ustep->getStepsSinceReset();

  Serial.print(last_motor_position_steps1);
  Serial.print("-");
  Serial.println(last_motor_position_steps);

  movestepper(3200);
  delay(3000);
  last_motor_position_steps = last_motor_position_steps + 3200;
  last_motor_position_steps1 = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
  miss_steps_counter = last_motor_position_steps - last_motor_position_steps1;
  Serial.print(last_motor_position_steps1);
  Serial.print("-");
  Serial.print(last_motor_position_steps);
  Serial.print("-");
  Serial.println(miss_steps_counter);
  if (miss_steps_counter < MISS_STEPS_TRESHOLD) {
    calibration_ok = true;
  }
  last_motor_position_steps = motor_position_steps;
  calibration_ok = true;
  movestepper(0);
  delay(3000);
  ustep->encoder.setHome();


  // back to relax mode
  detach();

  if (calibration_ok) {
    current_step_number = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
    ustep->updatepos(current_step_number);
    goal_step_number = (ustep->encoder.getAngleMoved() * 16 * 200) / 360;
    Serial.println("Calibration OK, set offset : ");
    return STEPPER_CALIBRATION_OK;
  }
  else {
    //SerialUSB.println("Calibration timeout");
    return STEPPER_CALIBRATION_TIMEOUT;
  }
}


