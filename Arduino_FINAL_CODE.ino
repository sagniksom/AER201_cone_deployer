#include <Wire.h>
#include <Servo.h>


// I2C variables

int sendCycle = 1;                          // Represents the type of byte, out of the 12 below, last sent from the Arduino to the PIC. 1 to 12.

// I2C syncronized variables: The upper 4 bits of each byte is an identifier from 0000 to 1011, indicating which byte out of 12 it is. The lower 4 bits contain actual data.

byte receiveData = 0;                       // The PIC sends to the Arduino by overwriting this byte.

byte sendSteps1 = 0;                        // Average number of steps taken by both motors.
byte sendSteps2 = 0;
byte sendSteps3 = 0;
byte sendSteps4 = 0;

byte sendTarget1 = 0;                       // Target position, in steps, of the conveyor.
byte sendTarget2 = 0;
byte sendTarget3 = 0;
byte sendTarget4 = 0;

byte sendCracksLeft = 0;                    // Total number of left cracks, center cracks, right cracks, and holes detected thus far.
byte sendCracksCenter = 0;
byte sendCracksRight = 0;
byte sendHoles = 0;

// Crack detection variables

const unsigned long sensorCrackDebounceTime;// Tunable variable; debouncing time for crack detection readings.

int sensorCrackBool[3] = {0,0,0};           // Boolean crack detection sensor readings. 0 = white, 1 = black. Updated by readSensorsBool(), used by debounceCrackSensors().
int sensorCrackDebounced[3] = {0,0,0};      // Debounced crack detection sensor readings. 0 = white, 1 = black. Updated by debounceCrackSensors().
unsigned long sensorCrackDebounceEndTime=-1;// Time at which a debounced reading will be taken. Used by debounceCrackSensors().
bool noDifference = true;                   // Whether or not a difference between sensorCrackBool and sensorCrackDebounced exists. Used by debounceCrackSensors().
int sensorCrackTransition[3] = {0,0,0};     // For any program cycle, these values will be 0, unless a white-to-black (1) or black-to-white (-1) transition is detected during that program cycle. Updated by debounceCrackSensors().
unsigned int numCracksLeft = 0;
unsigned int numCracksCenter = 0;
unsigned int numCracksRight = 0;
unsigned int numHoles = 0;

// Line following variables

int sensorLineBool[2] = {0,0};              // Boolean line following sensor readings. 0 = white, 1 = black. Updated by readSensorsBool().

// Stepper motor driving variables

unsigned int driveStepsLeft = 0;            // Total number of steps forward on the left side.
unsigned int driveStepsRight = 0;           // Total number of steps forward on the right side.

// Conveyor driving variables

const unsigned long conveyorTimePerStep;    // Tunable variable; velocity of the conveyor in milliseconds per step. Used by moveConveyor().
const unsigned long conveyorBufferTime;     // Tunable variable; milliseconds of time given to the conveyor to stabilize after movement. Used by moveConveyor().

unsigned long conveyorEndTime = 0;          // Time at which the conveyor will finish moving. Used by moveConveyor().
unsigned int conveyorTarget = 0;                     // Target position of the convoyer. Updated by moveConveyor(). Will be sent via I2C.
int conveyorPosition = 0;                   // Current location of the conveyor. Updates after each conveyor move. Used by moveConveyor().
int conveyorDone = 1;                       // Boolean indicating whether the moveConveyor function has finished. Used by moveConveyor().

// DC motor driving variables

const unsigned long dcSpinTime;             // Tunable variable; milliseconds of time for which the DC motor is powered in order to attain one revolution. Used by dropCone().
const unsigned long dcBufferTime;           // Tunable variable; milliseconds of time given to the DC motor to deaccelerate. Used by dropCone().

unsigned long dcStartTime = 0;              // Time at which DC motor started rotating. Used by dropCone().
int dcDone = 1;                             // Boolean indicating whether the dropCone function is done dropping a cone. Used by dropCone().


// Servo driving variables

const unsigned long servoToggleTime;        // Tunable variable; milliseconds of time required for the servo to toggle positions.

unsigned long servoEndTime = 0;             // Time at which the servo will have moved to the target position. Used by moveServo().
int servoState = 0;                         // Internal tracker of the servo's state. Used by moveServo().
int servoDone = 1;                          // Boolean indicating whether the moveServo function is done moving the servo. Used by moveServo().

// Wait function variables

unsigned long waitEndTime = 0;              // Time at which the requested wait ends. Used by wait().
int waitDone = 1;                           // Boolean indicating whether the wait function is done waiting. Used by wait().

// Hardware-pin interfacing functions

void readSensorsBool() {
  // Reads IR sensors, resolves analog readings into boolean (0 = white, 1 = black), and updates the values of sensorCrackBool[0,1,2] and sensorLineBool[0,1].

  // THIS IS A PLACEHOLDER FUNCTION AND MUST BE FILLED IN TO SPECFICATIONS.
}

void stepLeft(int dir) {
  // Steps the left motor in a given direction, determined by dir. If dir = 1, the motor moves a step in the positive direction. If dir = 0, the motor moves in the negative direction. Also adjusts driveStepsLeft accordingly.

  // THIS IS A PLACEHOLDER FUNCTION AND MUST BE FILLED IN TO SPECIFICATIONS.
}

void stepRight(int dir) {
  // Steps the right motor in a given direction, determined by dir. If dir = 1, the motor moves a step in the positive direction. If dir = 0, the motor moves in the negative direction. Also adjusts driveStepsRight accordingly.

  // THIS IS A PLACEHOLDER FUNCTION AND MUST BE FILLED IN TO SPECIFICATIONS.
}

void setMotor(int motorState) {
  // Sets the dc motor to either continuously spin or stop based on motorState. If motorState = 1, the motor spins. If motorState = 0, the motor is stationary.

  // THIS IS A PLACEHOLDER FUNCTION AND MUST BE FILLED IN TO SPECIFICATIONS.
}

void setServo(int servoTarget) {
  // Sets the servo to hold either a lowered or raised position based on servoTarget. If servoTarget = 0, the servo is lowered. If servoTarget = 1, the servo is raised.

  // THIS IS A PLACEHOLDER FUNCTION AND MUST BE FILLED IN TO SPECIFICATIONS.
}


// I2C functions

void setupI2C() {
  // Performs setup operations for I2C.
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void updateSendBytes() {
  // Updates the 12 bytes to be sent via I2C.
  unsigned int driveStepsAvg = ((driveStepsLeft + driveStepsRight) / 2);
  sendSteps1 = (byte) ((driveStepsAvg & 0b0000000000001111));
  sendSteps2 = (byte) ((driveStepsAvg & 0b0000000011110000) >> 4);
  sendSteps3 = (byte) ((driveStepsAvg & 0b0000111100000000) >> 8);
  sendSteps4 = (byte) ((driveStepsAvg & 0b1111000000000000) >> 12);

  sendTarget1 = (byte) ((conveyorTarget & 0b0000000000001111));
  sendTarget2 = (byte) ((conveyorTarget & 0b0000000011110000) >> 4);
  sendTarget3 = (byte) ((conveyorTarget & 0b0000111100000000) >> 8);
  sendTarget4 = (byte) ((conveyorTarget & 0b1111000000000000) >> 12);

  sendCracksLeft   = (byte) (numCracksLeft   & 0b0000000000001111);
  sendCracksCenter = (byte) (numCracksCenter & 0b0000000000001111);
  sendCracksRight  = (byte) (numCracksRight  & 0b0000000000001111);
  sendHoles        = (byte) (numHoles        & 0b0000000000001111);
  
  sendSteps1       = sendSteps1       | 0b00000000;
  sendSteps2       = sendSteps2       | 0b00010000;
  sendSteps3       = sendSteps3       | 0b00100000;
  sendSteps4       = sendSteps4       | 0b00110000;
  sendTarget1      = sendTarget1      | 0b01000000;
  sendTarget2      = sendTarget2      | 0b01010000;
  sendTarget3      = sendTarget3      | 0b01100000;
  sendTarget4      = sendTarget4      | 0b01110000;
  sendCracksLeft   = sendCracksLeft   | 0b10000000;
  sendCracksCenter = sendCracksCenter | 0b10010000;
  sendCracksRight  = sendCracksRight  | 0b10100000;
  sendHoles        = sendHoles        | 0b10110000;
}

void requestEvent() {
  // Function called when the PIC requests a byte of data. Note that updateSendBytes() is not called if only part of a 16-bit number has been sent.
  if(sendCycle == 1) {
    updateSendBytes();
    Wire.write(sendSteps1);
  }
  else if(sendCycle == 2) {
    Wire.write(sendSteps2);
  }
  else if(sendCycle == 3) {
    Wire.write(sendSteps3);
  }
  else if(sendCycle == 4) {
    Wire.write(sendSteps4);
  }
  else if(sendCycle == 5) {
    updateSendBytes();
    Wire.write(sendTarget1);
  }
  else if(sendCycle == 6) {
    Wire.write(sendTarget2);
  }
  else if(sendCycle == 7) {
    Wire.write(sendTarget3);
  }
  else if(sendCycle == 8) {
    Wire.write(sendTarget4);
  }
  else if(sendCycle == 9) {
    updateSendBytes();
    Wire.write(sendCracksLeft);
  }
  else if(sendCycle == 10) {
    updateSendBytes();
    Wire.write(sendCracksCenter);
  }
  else if(sendCycle == 11) {
    updateSendBytes();
    Wire.write(sendCracksRight);
  }
  else if(sendCycle == 12) {
    updateSendBytes();
    Wire.write(sendHoles);
  }

  sendCycle = sendCycle + 1;
  if(sendCycle >= 13) {
    sendCycle = 1;
  }
}

void receiveEvent() {
  receiveData = Wire.read();
}

// All other functions

void setConveyorTarget(int target) {
  // Simply sets the value conveyorTarget. This value will be transmitted to the PIC via I2C, which will then move the conveyor.
  conveyorTarget = (unsigned int) target;
}

void debounceCrackSensors() {
  // Applies debouncing to crack detection sensors. Generates values for sensorCrackDebounced[0,1,2] and sensorCrackTransition[0,1,2]. Also updates values for numCracksRight, numCracksCenter, numCracksLeft, and numHoles.
  sensorCrackTransition[0] = 0;
  sensorCrackTransition[1] = 0;
  sensorCrackTransition[2] = 0;
  noDifference = sensorCrackBool[0] == sensorCrackDebounced[0] && sensorCrackBool[1] == sensorCrackDebounced[1] && sensorCrackBool[2] == sensorCrackDebounced[2];
  if(!noDifference && sensorCrackDebounceEndTime < 0) {
    sensorCrackDebounceEndTime = millis() + sensorCrackDebounceTime;
  }
  else if(millis() > sensorCrackDebounceEndTime && !noDifference) {
    sensorCrackTransition[0] = sensorCrackBool[0] - sensorCrackDebounced[0];
    sensorCrackTransition[1] = sensorCrackBool[1] - sensorCrackDebounced[1];
    sensorCrackTransition[2] = sensorCrackBool[2] - sensorCrackDebounced[2];
    
    sensorCrackDebounced[0] = sensorCrackBool[0];
    sensorCrackDebounced[1] = sensorCrackBool[1];
    sensorCrackDebounced[2] = sensorCrackBool[2];

    if(sensorCrackTransition[0] == 1 && sensorCrackTransition[1] == 1 && sensorCrackTransition[2] == 0) {
      numCracksLeft += 1;
    }
    if(sensorCrackTransition[0] == 1 && sensorCrackTransition[1] == 1 && sensorCrackTransition[2] == 1) {
      numCracksCenter += 1;
    }
    if(sensorCrackTransition[0] == 0 && sensorCrackTransition[1] == 1 && sensorCrackTransition[2] == 1) {
      numCracksRight += 1;
    }
    if(sensorCrackTransition[0] == 0 && sensorCrackTransition[1] == 1 && sensorCrackTransition[2] == 0) {
      numHoles += 1;
    }
    
    sensorCrackDebounceEndTime = -1;
  }
}

int moveConveyor(int target) {
  // Moves the conveyor to a target position, and updates conveyorPosition once done. If this function returns 0, the conveyor is still being moved. If this function returns 1, the conveyor has reached its target. Call this function repeatedly until it returns 1 once, then stop.
  if(conveyorDone == 1) {                                                 // Since the previous conveyor move has been completed, begin a new conveyor move.
    conveyorEndTime = millis() + conveyorTimePerStep * abs(target - conveyorPosition) + conveyorBufferTime;
    setConveyorTarget(target);
    conveyorDone = 0;
    return(0);
  }
  else if(millis() > conveyorEndTime) {                                   // The conveyor has moved to its target position, so update conveyorPosition and indicate to software that the conveyor move is done.
    conveyorPosition = target;
    conveyorDone = 1;
    return(1);
  }
  else {                                                                  // The conveyor is still moving, so force software to wait.
    return(0);
  }
}

int dropCone() {
  // Sets the dc motor to spin exactly one revolution. If this function returns 0, the cone is still being dropped. If this function returns 1, the cone has been dropped. Call this function repeatedly until it returns 1 once, then stop.
  if(dcDone == 1) {                                                       // Since the previous cone drop has been completed, begin a new cone drop.
    dcStartTime = millis();
    setMotor(1);
    dcDone = 0;
    return(0);
  }
  else if(millis() > dcStartTime + dcSpinTime + dcBufferTime) {           // Both dcSpinTime and dcBufferTime have elapsed, so indicate to software that the cone drop is done.
    dcDone = 1;
    return(1);
  }
  else if(millis() > dcStartTime + dcSpinTime) {                          // dcSpinTime has elapsed, so turn the motor off. dcBufferTime has not elapsed, so force software to wait.
    setMotor(0);
    return(0);
  }
  else {                                                                  // dcSpinTime has not elapsed, so force software to wait.
    return(0);
  }
}

int moveServo(int servoTarget) {
  // Moves the servo to the target state (servoTarget = 1 corresponds to up, 0 corresponds to down). If this function returns 0, the servo is still moving. If this function returns 1, the servo has finished moving. Call this function repeatedly until it returns 1 once, then stop.
  if(servoDone == 1) {                                                    // Previous servo move is complete.
    if(servoTarget == servoState) {                                       // Silly programmer asked the servo to move to the position it's already in. Immediately indicate completion.
      return(1);
    }
    else {                                                                // Begin a new servo move.
      servoEndTime = millis() + servoToggleTime;
      setServo(servoTarget);
      servoDone = 0;
      return(0);
    }
  }
  else if(millis() > servoEndTime) {                                      // Servo move is complete, so update servoState and indicate to software that the move is done.
    servoState = servoTarget;
    servoDone = 1;
    return(1);
  }
  else {
    return(0);                                                            // Servo move is not done, so force software to wait.
  }
}

int wait(unsigned long duration) {
  // Used to have the robot wait. If this function returns 0, the requested duration has not elapsed. If this function returns 1, the requested duration has elapsed. Call this function repeatedly until it returns 1 once, then stop.
  if(waitDone == 1) {
    waitEndTime = millis() + duration;
    waitDone = 0;
    return(0);
  }
  else if(millis() > waitEndTime) {
    waitDone = 1;
    return(1);
  }
  else {
    return(0);
  }
}

long int mainCount = 0;
void setup() {
  setupI2C();
  Serial.begin(9600);
  mainCount = millis();
}

unsigned int sendNum = 0;

void loop() {
  
  mainCount +=1;
  if (millis() - mainCount >= 1000){
    sendNum += 2;
  }
  driveStepsLeft = sendNum;
  
}
