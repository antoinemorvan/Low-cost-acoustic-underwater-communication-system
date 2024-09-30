// #include <Arduino.h>
#include <stdio.h>

 

HardwareTimer *MyTim = new HardwareTimer(TIM1);

 

// Define output pins
const int pin1 = PC7; // Replace with your actual pin
const int pin2 = PB5; // Replace with your actual pin
 

// Define the frequency and the phase difference
const int frequency = 40000 ; // 40 kHz square wave 40000
const int phaseDifference = 90; // 90 degrees
 

// Variables to hold the states of the outputs
volatile bool state1 = LOW;
volatile bool state2 = LOW;

// Training and message sequence
const char* trainingSequence = "DDDDDDDD";
char fullSequence[20];
const char* messageSequence = "00101101";


int sequenceIndex = 0;
int f = 0; // flag
 


// Calculate the time period and phase difference in microseconds
const int period = 1000000 / frequency; // Period in microseconds
const int phaseDelay = period * phaseDifference / 360; // Phase delay in microseconds

int cpt = 1;


void setup() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  strcpy(fullSequence, trainingSequence);
  strcat(fullSequence, messageSequence);
 

  // Set up the timer
  MyTim->setMode(1, TIMER_OUTPUT_COMPARE, NC);
  MyTim->setOverflow(period / 4, MICROSEC_FORMAT); // Set the interrupt to half the period
  MyTim->attachInterrupt(1, toggleOutputs);
  MyTim->resume();

  Serial.begin(9600);
}

 

void loop() {
  // Serial.println(bitstream[0]);
  // Serial.println(cpt%2);

  // Your main code
  
}

 

void toggleOutputs() {
  
  char currentBits[2] = {fullSequence[2*sequenceIndex], fullSequence[2*sequenceIndex + 1]};

  // A chaque nouveau symbole
  if (cpt == 1) {

    if (strcmp(currentBits,"DD")==0){
    // Serial.println("vu");
      state1 = LOW;
      state2 = LOW;
      f = 1;
      // delayMicroseconds(24);
      }

    if (strcmp(currentBits,"10")==0){
    // Serial.println("vu");
      state1 = HIGH;
      state2 = LOW;
      }

    if (strcmp(currentBits,"00")==0){
      // Serial.println("vu");
      state1 = LOW;
      state2 = LOW;
      }
    if (strcmp(currentBits,"01")==0){
      // Serial.println("vu");
      state1 = LOW;
      state2 = HIGH;
      }
    if (strcmp(currentBits,"11")==0){
      // Serial.println("vu");
      state1 = HIGH;
      state2 = HIGH;
      }
    digitalWrite(pin1, state1);
    digitalWrite(pin2, state2);

  }

  // Génère le symbole
  else {
    if (f == 1){
      f = 1;
    }
    else{  
      if (cpt%2==0){
        state2 = !state2;
        digitalWrite(pin2, state2);

      }
      else {
        state1 = !state1;
        digitalWrite(pin1, state1);

      }}
  }
  
  cpt ++;
  if (cpt == 5){
    sequenceIndex++;
    cpt = 1;
    f = 0;
    if (sequenceIndex >= (strlen(fullSequence)/2)) {
      sequenceIndex = 0; // Reset index to loop through the sequence again
      }

  }

}
