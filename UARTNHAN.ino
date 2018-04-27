
const byte buffSize = 20;
char inputBuffer[buffSize];
const char startMarker = '{';
const char endMarker = '}';
byte bytesRecvd = 0;
boolean readInProgress = false;
unsigned long count = 0;
int valbegin = 0;
void movesteppertopos(int32_t vitri)
{   int32_t vitriht=stepper.getStepsSinceReset();
Serial.print("AA :");
Serial.println(vitriht);
    if (vitri<vitriht){
       stepper.moveSteps(vitriht-vitri,CCW,HARD);  
    }
    else{
      stepper.moveSteps(vitri-vitriht,CW,HARD);
    }
}
void nhanuart() {
  if (Serial.available() > 0) {
    char x = Serial.read();
    if (x == endMarker) {
      readInProgress = false;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    if (readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }
    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}
/* {0,maxacell,maxvelox}
 *  {1,p,i,d}
 *  {2,vitri}  movesteppertopos(-1000);
 *  {3,sobuoc)  stepper.moveSteps(3000,CCW,HARD); 
 *  {4, goc} stepper.moveToAngle(360.0,HARD);
 *  {5,Stop} 1 lÃ  Hard 2 lÃ  sof
 */ 
void parseData() {
  //Serial.print("{");
  //Serial.println(inputBuffer);
 // Serial.println("}");
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(inputBuffer, ",");     // get the first part - the string  // láº¥y kÃ½ tá»± trÆ°á»›c dáº¥u "," lÆ°u vÃ o biáº¿n strtokIndx
  valbegin = atoi(strtokIndx);
  if (valbegin == 0) {
    strtokIndx = strtok(NULL, ","); // step
    DataConf.MAXACCEL = atoi(strtokIndx);
    // newFlashInterval = atoi(strtokIndx);     // convert this part to an integer  //chuyen thanh kieu interger
    strtokIndx = strtok(NULL, ",");    //Vspeed ","); lÃ  500 vÃ   strtokIndx = strtok(NULL, ","); lÃ 200
    DataConf.MAXVELOC = atoi(strtokIndx);
    Serial.println(DataConf.MAXVELOC);
    Serial.println(DataConf.MAXACCEL);
    stepper.setMaxAcceleration(DataConf.MAXACCEL);
    stepper.setMaxVelocity(DataConf.MAXVELOC);
    saveStepConf();
  }
  else if (valbegin == 1) {
    strtokIndx = strtok(NULL, ",");  //acc
    DataConf.p = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); //scole
    DataConf.i = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    DataConf.d = atof(strtokIndx);
    Serial.println(DataConf.p);
    Serial.println(DataConf.i);
    Serial.println(DataConf.d);
    //stepper.updatePID(DataConf.p,DataConf.i,DataConf.d);
    saveStepConf();
  }
  else if (valbegin == 2) {
    strtokIndx = strtok(NULL, ","); // step
    int32_t vitri = atol(strtokIndx);
    
     //movesteppertopos(vitri);
    // stepper1.movestepper(vitri);
     stepper1.setNewGoal(vitri);
  }
  else if (valbegin == 3) {
    stepper1.calibrate();
  }
  else if (valbegin == 4) {
    strtokIndx = strtok(NULL, ","); // step
    float vitri = atof(strtokIndx);
    stepper.moveToAngle(vitri,HARD);
  }
  else if (valbegin == 5) {
    strtokIndx = strtok(NULL, ","); // step
    int stopvt = atoi(strtokIndx);
    Serial.println(stopvt);
    if (stopvt ==1 )stepper.softStop(HARD);
    else stepper.softStop(SOFT);
  }
}

