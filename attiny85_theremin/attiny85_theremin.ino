#define sensorPin 3
#define speakerPin 4
#define ledPin 0

int sensorVal; // The analog reading from the photoresistor

long speakerFreq; // The frequency to buzz the buzzer

// You can experiment with these values:
long BUZZ_FREQ_MAX = 2500; // Maximum frequency for the buzzer

long PR_MAX = 1023; // Maximum value for the photoresistor

void setup() {
  pinMode(ledPin, OUTPUT);  
  pinMode(speakerPin, OUTPUT); // set a pin for buzzer output

}



void loop() {

    sensorVal = analogRead(sensorPin); // Values 0-1023

    speakerFreq = (sensorVal * BUZZ_FREQ_MAX) / PR_MAX;

    buzz(speakerPin, speakerFreq, 10);
    
    digitalWrite(ledPin, HIGH);

}



void buzz(int targetPin, long frequency, long length) {

    long delayValue = 1000000/frequency/2;

    long numCycles = frequency * length/ 1000;

    for (long i=0; i < numCycles; i++){

        digitalWrite(targetPin,HIGH);

        delayMicroseconds(delayValue);

        digitalWrite(targetPin,LOW);

        delayMicroseconds(delayValue);

    }

}
