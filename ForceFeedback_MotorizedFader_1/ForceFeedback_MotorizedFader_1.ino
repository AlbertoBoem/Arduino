/* 
Force-feedback Interface using a Motorized Fader and H-bridge
*/

//Motor 1
int pinAIN1 = 20; //Direction 9
int pinAIN2 = 30; //Direction
int pinPWMA = 10; //Speed 3

//Motor 2
int pinBIN1 = 9; //Direction
int pinBIN2 = 8; //Direction
int pinPWMB = 3; //Speed 4
// ----

//Standby
int pinSTBY = 10; //10 4last

//Constants to help remember the parameters
static boolean turnCW = 0;  //for motorDrive function - clockwise 
static boolean turnCCW = 1; //for motorDrive function - reverse - counter-clockwise
static boolean motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
//not needed
static boolean motor2 = 1;  //for motorDrive, motorStop, motorBrake functions

int inputPin = A5; //A7
int potVal = 0;
int potValb = 0;
int locationB = 0;
int diff = 0;

float stiff = 1 ;  //1 very fast - 35 very slow come back
int forze = 0;

//Define constants
//const int redwire = 10; //pin 1 on h-bridge
//const int yellowwire = 11; //pin 2 on h-bridge
//const int greenwire = 12; //pin7 on h-bridge
const int analogPin=A0; //pin that the wiper is connected to //A1
int reference = 0; //position goal on the slider. Where I want the slider to end up.
int error = 0; //difference between reference and location. This will be computed later.
int location = 0; //current location fo slider. This will be read later. 

int started = 0;
int serialvalue;



void setup() {
  
Serial.begin(9600); //initialize serial communication at 9600 bits/sec
//Serial.setTimeout(10);
/*
//---------------
   #ifdef TCCR2B  //pwm pin 10-9 arduino mega 2560
   TCCR2B = TCCR2B & 0b11111000 | 0x01;
   #endif
   //sets Arduino Mega's pin 10 and 9 to frequency 31250.
   //code typically inserted in setup()


  // Here we make the ADC run faster:
  // - defines for setting and clearing register bits
  #ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
  #endif
  #ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  #endif
  // - set prescale to 16, to increase the "ADC sampling rate" from 9600Hz to 77kHz
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  //-------------
*/
pinMode(analogPin, INPUT);
pinMode(inputPin, INPUT);


pinMode(pinBIN1, OUTPUT); //initialize the #1h -bridge pin as an output

pinMode(pinBIN2, OUTPUT); //initialize the #2h -bridge pin as an output

pinMode(pinPWMB, OUTPUT); //initialize the #7h-bridge pin as an output

pinMode(pinSTBY, OUTPUT);

} 

void loop() {

stiff = 5;
  
  
potVal = analogRead(analogPin);
potValb = potVal;

reference = potVal;




//float stiff1 = analogRead(analogPin);
//stiff = map(stiff1, 0, 255, 0, );

//read position 
location = 200; //analogRead(analogPin);




int diffB;

diff = locationB - location;
diffB = diff;
diff = map(diff, -10, 10, 0, 255); //-6 6


//compute error
error = reference-location; //the difference between where you want to be and where you are

Serial.write(error);

forze = (-stiff) * diff;  //Hook's Law (?)

int forze2 = forze;

//compute output based on error
if(error>10){
motorDrive(motor2, turnCW, forze);
}
else {
motorDrive(motor2, turnCCW, forze);
}
if(abs(error)<10){ //<10
digitalWrite(pinBIN1, HIGH);
digitalWrite(pinBIN2, HIGH);
} 

locationB = location;


serialCom(locationB, reference);

//if(Serial.available() == 0) // check to see if there's serial data in the buffer
//  {}



    
    //Serial.write((int)locationB); // print the counter01_control_nurbs_with_force
  
           
   //}


/*
if(Serial.available()) // check to see if there's serial data in the buffer
  {
    serialvalue = Serial.read(); // read a byte of serial data
    started = 1; // set the started flag to on
  }

  if(started) // loop once serial data has been received
  {
    
    Serial.write(locationB); // print the counter01_control_nurbs_with_force
  }
*/


}


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

