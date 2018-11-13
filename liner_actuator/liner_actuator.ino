//test with 3 Kumina actuators
//move one step (0.002mm)
//start: press 'a'
//move down: press 'w'
//move up: press 's'
//every step is around 0.5mm (0-2500 = 5mm , 0-5000 = 10mm)
bool dir = false;
bool step = false;
bool sw = false;
unsigned long stime;
unsigned long ntime;
unsigned long IntTime;
unsigned long delayTime = 0;
unsigned long pulse = 0;
unsigned long totalPulse = 0;
unsigned long maxPulse = 15000;
unsigned long maxRange = 250;// = 250 maxRange * 0.002 mm
//1step0.002

int ranges = 0;

void play(){
 stime = micros();
 IntTime = stime-ntime;
 if(IntTime>maxRange-delayTime){ //150
   ntime = stime;
   if(step){
     digitalWrite(3, dir);
     digitalWrite(2, HIGH);
     digitalWrite(5, dir);
     digitalWrite(4, HIGH);
     digitalWrite(7, dir);
     digitalWrite(6, HIGH);
     digitalWrite(9, dir);
     digitalWrite(8, HIGH);
     step=false;
   }else{  
     digitalWrite(3, dir);
     digitalWrite(2, LOW);
     digitalWrite(5, dir);
     digitalWrite(4, LOW);
     digitalWrite(7, dir);
     digitalWrite(6, LOW);
     digitalWrite(9, dir);
     digitalWrite(8, LOW);
     pulse++;
     if(pulse<(maxRange/2)) //125
       delayTime+=2;
     else if(pulse>(maxRange/2)) //75
       delayTime-=2;
     step=true;
   }
   
 }
 if(pulse==maxRange){
   sw=false;
   delayTime=0;
   pulse=0;
 }
}

void setup() {
 // put your setup code here, to run once:
 pinMode(2, OUTPUT);
 pinMode(3, OUTPUT);
 pinMode(4, OUTPUT);
 pinMode(5, OUTPUT);
 pinMode(6, OUTPUT);
 pinMode(7, OUTPUT);
 pinMode(8, OUTPUT);
 pinMode(9, OUTPUT);
 
 Serial.begin(19200);
 delay(500);
 while(1){
   if(Serial.available()>0)
     if(Serial.read()=='a'){
      Serial.print("ok");
      Serial.println("\t");
       break;
     }
 }
 stime = micros();
 ntime = 0;
}

void loop() {
 if(Serial.read()=='w'){
   //for(int i = 0;10 > i;i++){
           //maxRange = 250;
           //maxRange = 150;
           dir=false;
           sw=true;
           ranges += maxRange;
           Serial.print(ranges);
           Serial.println("\t");
           //stepsz +=1;
           
   //}
   }
   if(Serial.read()=='s'){
     //for(int i=0;10>i;i++){
           //maxRange = 250;
           //maxRange = 150;
           dir=true;
           sw=true;
           ranges -= maxRange; 
           Serial.print(ranges);
           Serial.println("\t");
       //delay(1000);
           //stepsz -=1;
     //}
   }
   if(sw)
      play();
      
      
}
