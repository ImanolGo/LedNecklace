/*
  LED NECKLACE

  Fashion necklace enhanced with glowing leds and photodiodes to 
  adjust the brightness.Made in collaboration with Elektrocouture

  The circuit uses the Adafruit Feather 32u4 Basic Proto. The Leds used are the
  Flora Neopixels, controlled using the FastLED library and connected through
  the pin 6.
  Also the brightness is controlled using a set of BPW34 photo-diodes 
  connected to pin A4 and a 10MOhm Restistor.
  
  The circuit:
  * Input : Photodiode  BPW34 connected to A0
            Neopixels connected to pin 6


  Created 16th March 2016
  By Imanol Gómez
  Modified 16th March 2016
  By Imanol Gómez

  https://github.com/ImanolGo/LedNecklace

*/


#include "FastLED.h"

#define NUM_LEDS 4
#define DATA_PIN 6

// uint16_t speed = 1; // almost looks like a painting, moves very slowly
//uint16_t speed = 20; // a nice starting speed, mixes well with a scale of 100
// uint16_t speed = 33;
 uint16_t speed = 80; // wicked fast!

//uint16_t scale = 1; // mostly just solid colors
// uint16_t scale = 4011; // very zoomed out and shimmery
uint16_t scale = 100;

// This is the array that we keep our computed noise values in
uint8_t noise[NUM_LEDS];

// The 32bit version of our coordinates
static uint16_t x;
static uint16_t y;
static uint16_t z;

// Define the array of leds
CRGB leds[NUM_LEDS];



const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int brightness = 20;

int inputPin = A4;


void setup() { 
    
   setupLeds();
   setupAnalogSmooth();
   setupSerial();
}

void setupLeds() { 
  
   FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
   LEDS.setBrightness(brightness);

   // Initialize our coordinates to some random values
    x = random16();
    y = random16();
    z = random16();
}


void setupAnalogSmooth() { 
  // initialize all the readings to 0:
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = 0;
    }
}

void setupSerial() { 
    // initialize serial communication with computer:
    Serial.begin(9600);
}



void loop() { 

    updateAnalogRead();
    updateBrightness();
    drawBreathingLeds();
    //drawTestLeds();
    sendSerial();
}

void updateBrightness() { 

  float fAverage = average;
  brightness = zmap(fAverage, 520.0, 700.0, 255.0, 0.0, false);
  brightness = clamp(brightness, 0, 255);
  LEDS.setBrightness(brightness);
}

void updateAnalogRead() { 
  
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
}

void drawBreathingLeds() { 

  float period = zmap(speed, 0, 100, 10000, 100, false);

  for(int i = 0; i < NUM_LEDS; i++) {
    int offset = scale * i;
    float val = (exp(sin(millis()/(period+offset)*PI + +offset)) - 0.36787944)*108.0;

    leds[i] = CHSV(179,150,val);
  }

  FastLED.show();
}
void drawTestLeds() 
{ 
  for(int i = 0; i < NUM_LEDS; i++) {
        // Turn the LED on, then pause
      leds[i] = CRGB::Red;
      FastLED.show();
      delay(500);
      // Now turn the LED off, then pause
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(500);
  }
}

void drawNoiseLeds() { 
   
  static uint8_t ihue=0;
  fillnoise8();
 for(int i = 0; i < NUM_LEDS; i++) {
      //leds[i] = CHSV(noise[i],255,noise[i]);
      leds[i] = CRGB(noise[i],0,0);
  }
  
  ihue+=1;

  LEDS.show();


}

void sendSerial()
{
  //Serial.println(average);
  Serial.println(brightness);
  delay(1);        // delay in between reads for stability
}


// Fill the array of 8-bit noise values using the inoise8 function.
void fillnoise8() {
  for(int i = 0; i < NUM_LEDS; i++) {
      int offset = scale * i;
      noise[i] = inoise8(x + offset,z);
  }
  z += speed;
}



float zmap(float input, float inMin, float inMax, float outMin, float outMax, bool _clamp){

      float out = (input-inMin)/(inMax-inMin)*(outMax-outMin)+outMin;

      if(_clamp){
        out = clamp(out, outMin, outMax);
      }
      
      return out;

}


float clamp(float value, float valueMin, float valueMax){

      float out = value;
      
      if(value > valueMax){
        out = valueMax;
      }

      if(value < valueMin){
        out = valueMin;
      }
      
    return out;
}


