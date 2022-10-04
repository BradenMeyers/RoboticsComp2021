long compTime = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt function LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt function RIGHT
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorLD = 9;
const int motorLS = 10;
const int motorRD = 12;
const int motorRS = 11;
int rightSpeed = 0;
int leftSpeed = 0;
//check stall vars
bool setTime = false;
unsigned long nowTime = 0;
unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
int freqSpeed = 100;
unsigned long encoderCountL = 0;
unsigned long encoderCountR = 0;
unsigned long triggerTime = 0;
bool trigger = false;

void checkStall(int speedM){
  if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
        //Serial.println("set time");
  }
   if ((millis() - nowTime)>= freqSpeed){
        encoderCountL = (totalLRot - pastLRot);
        encoderCountR = (totalRRot - pastRRot);
        Serial.print("right: ");
        Serial.println(encoderCountR);
        Serial.print("left: ");
        Serial.println(encoderCountL);
        Serial.print("time since trigger");
        Serial.println(millis() - triggerTime);
        if(millis() - triggerTime >= 1000){
          int checkStallCount = map(speedM, 50,255, 25, 50);
          Serial.print("stallCount: ");
          Serial.println(checkStallCount);
          if (encoderCountL <= checkStallCount){
            triggerTime = millis();
            trigger = true;
            Serial.println("triggered L");
            //Serial.println("trigger on");
          }
          else if(encoderCountR <= checkStallCount){
            triggerTime = millis();
            trigger = true;
            Serial.println("triggered R");
          }
        }
        setTime = false;
        //Serial.print("reset stall");
    }
}

void moveForward(int rotation, int speedL, int speedR){
  rightSpeed = speedR;
  leftSpeed = speedL;
  triggerTime = millis();
  rightCount = rotation + totalRRot;
  leftCount = rotation + totalLRot;
  digitalWrite(motorLD, LOW);
  digitalWrite(motorRD, LOW);
  while(rightCount >= totalRRot && leftCount >= totalLRot){
    analogWrite(motorLS, leftSpeed);
    analogWrite(motorRS, rightSpeed);
    checkStall(speedL);
    if(trigger == true){
      leftCount = totalLRot;
      trigger = false;
      Serial.println("stopped");
    }
  }
  analogWrite(motorLS, 0);
  analogWrite(motorRS, 0);
}
void moveBackward(int rotation, int speedL, int speedR){
  rightSpeed = speedR;
  leftSpeed = speedL;
  triggerTime = millis();
  rightCount = rotation + totalRRot;
  leftCount = rotation + totalLRot;
  digitalWrite(motorLD, HIGH);
  digitalWrite(motorRD, HIGH);
  while(rightCount >= totalRRot && leftCount >= totalLRot){
    analogWrite(motorLS, leftSpeed);
    analogWrite(motorRS, rightSpeed);
    checkStall(speedL);
    if(trigger == true){
      leftCount = totalLRot;
      trigger = false;
    }
  }
  analogWrite(motorLS, 0);
  analogWrite(motorRS, 0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(motorLS, OUTPUT);
  pinMode(motorRS, OUTPUT);
  pinMode(motorLD, OUTPUT);
  pinMode(motorRD, OUTPUT);
  digitalWrite(motorLD, LOW);
  digitalWrite(motorLD, LOW);
  pinMode(interruptPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
  pinMode(interruptPinL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
  Serial.println("Setup Complete");
  delay(4500);
}

void loop() {
  compTime = millis();
  while(millis() - compTime <= 60000){
    moveForward(2300, 93,70);
    moveBackward(2300,113,90);
    moveForward(2300, 108, 70);
    moveBackward(2300, 115,90);
    //delay(1000);
  }
  while(millis() - compTime >= 60000){
    Serial.println("Congrats");
  }
  // put your main code here, to run repeatedly:
}

void addRotR(){
    totalRRot += 1;
}
void addRotL(){
    totalLRot += 1;
}
