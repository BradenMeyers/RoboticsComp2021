unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt funciton LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt funciton RIGHT
//Pin Assignments
const int interruptPinR = 32;
const int interruptPinL = 34;
int motorLS = 26;  
int motorLD = 12; 
int motorRS = 25;
int motorRD = 33;

int leftDutyC = 100;
int rightDutyC = 106;
int turnRotate = 390;

// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int resolution = 8;
const int pwmChannelR = 1;

void setup() {
  //set speed to 0
  //ledcWrite(pwmChannelL, 0);
  //ledcWrite(pwmChannelR, 0);
  // put your setup code here, to run once:
  pinMode(motorLD, OUTPUT);
  pinMode(motorRD, OUTPUT);
  digitalWrite(motorLD, LOW);
  digitalWrite(motorLD, LOW);
   
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
  // put your main code here, to run repeatedly:
  Serial.print("Left Rotation Count:");
  Serial.println(leftCount);
  Serial.print("Right Rotation Count:");
  Serial.println(rightCount);
  moveForward(1000,1000);
  delay(500);
  turnLeft();
  delay(500);
  turnRight();
  delay(500);
  moveBackward(1000,1000);
  delay(5000);
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
    Serial.print("Left Rotation Count:");
    Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      Serial.print("Right Rotation Count:");
      Serial.println(rightCount);
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
    Serial.print("Left Rotation Count:");
    Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      Serial.print("Right Rotation Count:");
      Serial.println(rightCount);
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
  Serial.print("Right Rotation Count:");
  Serial.println(rightCount);
  Serial.println("Movement Completed");
}

void turnLeft() {
  digitalWrite(motorLD, HIGH);      //set direction of the motors
  digitalWrite(motorRD, LOW);
  Serial.println("Moving LEFT");
  rightCount = (turnRotate + totalRRot);
  leftCount = (turnRotate + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.print("Left Rotation Count:");
    Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      Serial.print("Right Rotation Count:");
      Serial.println(rightCount);
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

void turnRight() {
  digitalWrite(motorLD, LOW);      //set direction of the motors
  digitalWrite(motorRD, HIGH);
  Serial.println("Moving RIGHT");
  rightCount = (turnRotate + totalRRot);
  leftCount = (turnRotate + totalLRot);
  while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
    ledcWrite(pwmChannelL, leftDutyC);
    ledcWrite(pwmChannelR, rightDutyC);
    Serial.print("Left Rotation Count:");
    Serial.println(leftCount);
    if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
      ledcWrite(pwmChannelR, 0);
      Serial.print("Right Rotation Count:");
      Serial.println(rightCount);
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
  Serial.print("Left Rotation Count:");
  Serial.println(leftCount);
  //rightCount = 0;
  Serial.print("Right Rotation Count:");
  Serial.println(rightCount);
  Serial.println("turn completed");
}

void addrotR(){
  totalRRot += 1;
}
void addrotL(){
  totalLRot += 1;
}
