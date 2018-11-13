//Force-feedback Interface using a Motorized Fader and H-bridge

#define potPin    A0
#define pinPWMB   3           
#define pinBIN1   9    
#define pinBIN2   8 

//Motor 1 - NOT USED
int pinAIN1 = 20; //Direction 9
int pinAIN2 = 30; //Direction
int pinPWMA = 10; //Speed 3


int pinSTBY = 10; //10 4last

static boolean turnCW = 0;  //for motorDrive function - clockwise 
static boolean turnCCW = 1; //for motorDrive function - reverse - counter-clockwise
static boolean motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
//not needed
static boolean motor2 = 1;  //for motorDrive, motorStop, motorBrake functions


 
void setup() {
    Serial.begin(9600);
    pinMode(potPin, INPUT);
    /*pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);*/

    pinMode(pinBIN1, OUTPUT); //initialize the #1h -bridge pin as an output

pinMode(pinBIN2, OUTPUT); //initialize the #2h -bridge pin as an output

pinMode(pinPWMB, OUTPUT); //initialize the #7h-bridge pin as an output

pinMode(pinSTBY, OUTPUT);
}
 
void loop() {
    Lumpy();
    //SpringA();
    //SpringB();
}
 
void Lumpy(){
    int val = analogRead(potPin);
    for(int i = 0; i<1024; i+= 200){
       if(val >  i-50 && val < i+50){
          if(val > i+3){
              
              motorDrive(motor2, turnCCW, 120);

              //digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
              //analogWrite(enA, 120);
          }else if(val < i-3){
            motorDrive(motor2, turnCW, 120);

              //digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
              //analogWrite(enA, 120);
          }else{
              motorBrake(motor2); //or mototStop
              //analogWrite(enA, LOW);
          }
       }
    }
}
 
void SpringA(){


  
    int val = analogRead(potPin);
    
    if(val > 10){
      int val2 = map(val, 10, 1024, 100, 255);
      motorDrive(motor2, turnCCW, val2);
      

        //digitalWrite(in1, LOW);
        //digitalWrite(in2, HIGH);
        //analogWrite(enA, map(val, 10, 1024, 100, 255));
    }else{
      motorBrake(motor2); //or mototStop
        //analogWrite(enA, LOW);
    }
}
 
void SpringB(){
    int val = analogRead(potPin);
    if(val > 550){
      int val3 = map(val, 10, 1024, 100, 255);
      motorDrive(motor2, turnCCW, val3);
        //digitalWrite(in1, LOW);
        //digitalWrite(in2, HIGH);
        //analogWrite(enA, map(val, 550, 1024, 120, 255));
    }else if(val < 450){
      int val4 = map(val, 450, 0, 120, 255);
      motorDrive(motor2, turnCW, val4);
        //digitalWrite(in1, HIGH);
        //digitalWrite(in2, LOW);
        //analogWrite(enA, map(val, 450, 0, 120, 255));
    }else{
       motorBrake(motor2); //or mototStop
        //analogWrite(enA, LOW);
    }
}

 /*
void SlideToValue(int targetValue){
  int val = analogRead(potPin);
  if(abs(val - targetValue) > 20){
      if(val > targetValue){
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
      }else{
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
      }
      analogWrite(enA, max(min(abs(val - targetValue), 255), 200));
  }else{
      // Turn off motor
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);  
      analogWrite(enA, 0);
  }
}
*/

//
//--------------------------- FUNCTIONS ----------------------------
//

void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed) {
  /*
  This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  boolean pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)
 
  //Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  
  if (motorDirection == turnCW)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the speed
  
  if(motorNumber == motor1) {
    
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorSpeed);
    
  }
  
  else {
    
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);  //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorSpeed);
    
  }
   
  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);

}

void motorBrake(boolean motorNumber) {

// This "Short Brake"s the specified motor, by setting speed to zero

  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
   
}


void motorStop(boolean motorNumber) {
  
  //This stops the specified motor by setting both IN pins to LOW
  
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  
  else {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  } 
  
}


void motorsStandby() {
  
// This puts the motors into Standby Mode

  digitalWrite(pinSTBY, LOW);
  
}

void serialCom (int value, float value1) {

  Serial.print(value, DEC);
  Serial.print(" ");
  Serial.print(value1, DEC);
  Serial.print("\r");
  delay(10);
}

