#include <Arduino.h>
#include <LiteLED.h>

#define LED_TYPE          LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW  0   // if the LED is an RGBW type, change the 0 to 1
#define LED_GPIO          6   // change this number to be the GPIO pin connected to the LED
#define LED_BRIGHT        30  // sets how bright the LED is. O is off; 255 is burn your eyeballs out (not recommended)

#define VOLUMEDOWM 11
#define VOLUMEUP   12
#define MODI2      13
#define MODI1      14

// pick the colour you want from the list here and change it in setup()
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_WHITE = 0xe0e0e0;

int LEDPin = 38;

int VolumeDown = 0;
int VolumeUp = 0;
int Modi2 = 0;
int Modi1 = 0;

int VolumeDown_last = 0;
int VolumeUp_last = 0;
int Modi2_last = 0;
int Modi1_last = 0;

LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );    // create the LiteLED object; we're calling it "myLED"

void setup() {
  myLED.begin( LED_GPIO, 4 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
  myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level
  // myLED.setPixel( 0, L_GREEN, 1 );
  // myLED.setPixel( 1, L_RED, 1 );
  // myLED.setPixel( 2, L_BLUE, 1 );
  // myLED.setPixel( 3, L_WHITE, 1 );

  pinMode(LEDPin, OUTPUT);

  pinMode(VOLUMEDOWM, INPUT_PULLUP);
  pinMode(VOLUMEUP, INPUT_PULLUP);
  pinMode(MODI2, INPUT_PULLUP);
  pinMode(MODI1, INPUT_PULLUP);

  delay( 2000 );
}

void loop() {

  digitalWrite(LEDPin, HIGH);

  // myLED.brightness( 0, 1 );           // turn the LED off
  // delay( 1000 );

  delay(10);
  VolumeDown = digitalRead(VOLUMEDOWM);
  VolumeUp = digitalRead(VOLUMEUP);
  Modi2 = digitalRead(MODI2);
  Modi1 = digitalRead(MODI1);

  if((VolumeDown != VolumeDown_last) && (VolumeDown == LOW)){
    printf("taste");
    myLED.brightness( LED_BRIGHT, 1 );  // turn the LED on
    myLED.setPixel( 0, L_GREEN, 1 );
    myLED.setPixel( 1, L_RED, 1 );
    myLED.setPixel( 2, L_BLUE, 1 );
    myLED.setPixel( 3, L_WHITE, 1 );
  }

  if((VolumeUp != VolumeUp_last) && (VolumeUp == LOW)){
    myLED.brightness( LED_BRIGHT, 1 );  // turn the LED on
    myLED.setPixel( 0, L_WHITE, 1 );
    myLED.setPixel( 1, L_GREEN, 1 );
    myLED.setPixel( 2, L_RED, 1 );
    myLED.setPixel( 3, L_BLUE, 1 );

  }

  if((Modi1 != Modi1_last) && (Modi1 == LOW)){
    myLED.brightness( LED_BRIGHT, 1 );  // turn the LED on
    myLED.setPixel( 0, L_BLUE, 1 );
    myLED.setPixel( 1, L_WHITE, 1 );
    myLED.setPixel( 2, L_GREEN, 1 );
    myLED.setPixel( 3, L_RED, 1 );
    
  }

  if((Modi2 != Modi2_last) && (Modi2 == LOW)){
    myLED.brightness( LED_BRIGHT, 1 );  // turn the LED on
    myLED.setPixel( 0, L_RED, 1 );
    myLED.setPixel( 1, L_BLUE, 1 );
    myLED.setPixel( 2, L_WHITE, 1 );
    myLED.setPixel( 3, L_GREEN, 1 );
  }

  VolumeDown_last = VolumeDown;
  VolumeUp_last = VolumeUp;
  Modi2_last = Modi2;
  Modi1_last = Modi1;
 
}

