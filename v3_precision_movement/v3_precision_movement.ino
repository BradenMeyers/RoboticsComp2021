unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt funciton LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt funciton RIGHT
//Pin Assignments
const int interruptPinR = 19;
const int interruptPinL = 18;
const int motorLS = 4;  
const int motorLD = 2; 
const int motorRS = 17;
const int motorRD = 16;
//const int sleepL = 32;
//const int sleepR = 26;
//Adjustable speed and movement properties
int leftDutyC = 300;
int rightDutyC = 300;
int encoderCountL = 0;
int encoderCountR = 0;
int freqSpeed = 100;
unsigned long currentTime = 0;
bool setTime = false;

// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int resolution = 10;
const int pwmChannelR = 1;

void setup() {
  //set speed to 0
  //ledcWrite(pwmChannelL, 0);
  //ledcWrite(pwmChannelR, 0);
  // set pinmodes for the GPIO
  pinMode(motorLD, OUTPUT);
  pinMode(motorRD, OUTPUT);
  digitalWrite(motorLD, LOW);
  digitalWrite(motorLD, LOW);
  //pinMode(sleepL, OUTPUT);
  //pinMode(sleepR, OUTPUT);
  //digitalWrite(sleepL, HIGH);
  //digitalWrite(sleepR, HIGH);
   
  // configure LED PWM functionalitites
  ledcSetup(pwmChannelL, freq, resolution);
  ledcSetup(pwmChannelR, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motorLS, pwmChannelL);
  ledcAttachPin(motorRS, pwmChannelR);
  // pinmodes and interrupts
  pinMode(interruptPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinR), addrotR, CHANGE);
  pinMode(interruptPinL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinL), addrotL, CHANGE);
  
  Serial.begin(115200);
  delay(1000);    //give time to initiazile
}


void loop() {
 /*
  ledcWrite(pwmChannelL, leftDutyC);
  ledcWrite(pwmChannelR, rightDutyC);
  checkSpeed();
  Serial.println("point 1");
  delay(3000);
  digitalWrite(sleepL, LOW);
  digitalWrite(sleepR, LOW);
  Serial.println("point 2");
  delay(3000);
  digitalWrite(sleepL, HIGH);
  digitalWrite(sleepR, HIGH);  
  */
  
  moveForward(1700,1700);
  delay(250);
  moveBackward(1700,1700);
  delay(250);
  /*
  turnLeft(317);
  delay(200);
  moveForward(350,350);
  delay(200);
  turnLeft(317);
  delay(200);
  moveForward(1000,1000);
  delay(200);
  turnRight(335);
  delay(200);
  moveForward(350,350);
  delay(200);
  turnRight(335);
  delay(3000);
  */
}

void checkSpeed(){
  if (setTime == false){        //only will happen on the intialization of the function
    currentTime = millis();
    pastRRot = totalRRot;
    pastLRot = totalLRot;
    setTime = true;
  }
  Serial.println("checking Speed");
  if ((millis() - currentTime) >= freqSpeed){
    encoderCountL = (totalLRot - pastLRot);
    encoderCountR = (totalRRot - pastRRot);
    if ((encoderCountL - encoderCountR) >= 2){
      rightDutyC++;
      Serial.println("left moves faster");
    }
    if ((encoderCountR - encoderCountL) >= 2){
      rightDutyC--;
      Serial.println("right moves faster");
    }
    Serial.print("Right Speed Set");
    Serial.println(rightDutyC);
    Serial.print("left rotations:");
    Serial.println(encoderCountL);
    Serial.print("Right rotations:");
    Serial.println(encoderCountR);     
    setTime = false;                  //Reset the whole process
  }
}

void moveBackward(int leftRotation, int rightRotation ) {
  digitalWrite(motorLD,HIGH);                  //set direction as Backward
  digitalWrite(motorRD,HIGH);
  leftDutyC = 300;
  rightDutyC = 300;
  Serial.println("MOVING Backward");
  rightCount = (rightRotation + totalRRot);
  leftCount = (leftRotation + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    checkSpeed();
    Serial.println(rightDutyC);
    //Serial.print("Left Rotation Count:");
    //Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      //Serial.print("Right Rotation Count:");
      //Serial.println(rightCount);
      Serial.println("Motor Right Stopped First");
    }
  }
  ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
  while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.println("Right motor continue");
  }
  ledcWrite(pwmChannelL, 0);            //turn both motors off
  ledcWrite(pwmChannelR, 0);
  //Serial.print("Left Rotation Count:");
  //Serial.println(leftCount);
  //Serial.print("Right Rotation Count:");
  //Serial.println(rightCount);
  //Serial.println("Movement Completed");
}

void moveForward(int leftRotation, int rightRotation ) {
  digitalWrite(motorLD,LOW);                  //set direction as forward
  digitalWrite(motorRD,LOW);
  leftDutyC = 300;
  rightDutyC = 300;
  Serial.println("MOVING FOWARD");
  rightCount = (rightRotation + totalRRot);
  leftCount = (leftRotation + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    checkSpeed();
    Serial.println(rightDutyC);
    //Serial.print("Left Rotation Count:");
    //Serial.println(totalLRot);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      //Serial.print("Right Rotation Count:");
      //Serial.println(totalRRot);
      Serial.println("Motor Right Stopped First");
    }
  }
  ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
  while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.println("Right motor continue");
  }
  ledcWrite(pwmChannelL, 0);            //turn both motors off
  ledcWrite(pwmChannelR, 0);                        
  Serial.println("Movement Completed");
}

void turnLeft(int turnRotate) {
  digitalWrite(motorLD, HIGH);      //set direction of the motors
  digitalWrite(motorRD, LOW);
  leftDutyC = 500;
  rightDutyC = 500;
  Serial.println("Moving LEFT");
  rightCount = (turnRotate + totalRRot);
  leftCount = (turnRotate + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    checkSpeed();
    //Serial.print("Left Rotation Count:");
    //Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      //Serial.print("Right Rotation Count:");
      //Serial.println(rightCount);
      Serial.println("Motor Right Stopped First");
    }
  }
  ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
  while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.println("Right motor continue");
  }
  ledcWrite(pwmChannelL, 0);            //turn both motors off
  ledcWrite(pwmChannelR, 0);                       
  //Serial.print("Left Rotation Count:");
  //Serial.println(leftCount);
  //Serial.print("Right Rotation Count:");
  //Serial.println(rightCount);
  Serial.println("turn completed");
}

void turnRight(int turnRotate) {
  digitalWrite(motorLD, LOW);      //set direction of the motors
  digitalWrite(motorRD, HIGH);
  leftDutyC = 500;
  rightDutyC = 500;
  Serial.println("Moving RIGHT");
  rightCount = (turnRotate + totalRRot);
  leftCount = (turnRotate + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    checkSpeed();
    //Serial.print("Left Rotation Count:");
    //Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      //Serial.print("Right Rotation Count:");
      //Serial.println(rightCount);
      Serial.println("Motor Right Stopped First");
    }
  }
  ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
  while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.println("Right motor continue");
  }
  ledcWrite(pwmChannelL, 0);            //turn both motors off
  ledcWrite(pwmChannelR, 0);
  //leftCount = 0;                        //Reset the rotation count values
  //Serial.print("Left Rotation Count:");
  //Serial.println(leftCount);
  //rightCount = 0;
  //Serial.print("Right Rotation Count:");
  //Serial.println(rightCount);
  Serial.println("turn completed");
}

void addrotR(){
  totalRRot += 1;
}
void addrotL(){
  totalLRot += 1;
}
