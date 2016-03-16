/*
  LED NECKLACE

  Fashion necklace enhanced with glowing leds and photodiodes to 
  adjust the brightness.Made in collaboration with Elektrocouture

  The circuit uses the Adafruit Feather 32u4 Basic Proto. The Leds used are the
  Flora Neopixels, controlled using the FastLED library and connected through
  the pin 6.
  Also the brightness is controlled using a set of BPW34 photo-diodes 
  connected to pin A0.
  
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

#define NUM_LEDS 1
#define DATA_PIN 6

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
    
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

void loop() { 
  
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}
