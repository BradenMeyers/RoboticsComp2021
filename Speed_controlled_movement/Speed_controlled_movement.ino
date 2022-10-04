unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt funciton LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt funciton RIGHT
//Pin Assignments
const int interruptPinR = 32;
const int interruptPinL = 34;
const int motorLS = 26;  
const int motorLD = 12; 
const int motorRS = 25;
const int motorRD = 33;
const int sleepL = 27;
const int sleepR = 35;
//Adjustable speed and movement properties
int leftDutyC = 400;
int rightDutyC = 400;
int encoderCountL = 0;
int encoderCountR = 0;
int freqSpeed = 100;
unsigned long currentTime = 0;
boolean setTime = false;

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
  pinMode(sleepL, OUTPUT);
  pinMode(sleepR, OUTPUT);
  digitalWrite(sleepL, LOW);
  digitalWrite(sleepR, LOW);
   
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
 
  ledcWrite(pwmChannelL, leftDutyC);
  ledcWrite(pwmChannelR, rightDutyC);
  checkSpeed();
  delay(1000);
  digitalWrite(sleepL, HIGH);
  digitalWrite(sleepR, HIGH);
  delay(500);
  digitalWrite(sleepL, LOW);
  digitalWrite(sleepR, LOW);  
  
 /*
  moveForward(2000,2000);
  delay(250);
  //turnLeft(340);
  delay(250);
  //turnRight(340);
  delay(250);
  //moveForward(2000,2000);
  moveBackward(2000,2000);
  delay(5000);
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
