/* 
 *  Run LED light strip off of MusselBedHeater RevF board
 *  with PCA9685 PWM driver.
 *  
 */


#include <Wire.h>
#include "LED_light_strip_PWM.h"
#include "Adafruit_PWMServoDriver.h" // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#define COMMON_ANODE true
#define BUTTON1 2     // BUTTON1 on INT0, pin PD2
//-------------------------------------------------------------
// PCA9685 pulse width modulation driver chip
// Called this way, uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define PWM_OE 3 // PWM driver output enable pin (active low), connected to ATmega pin PD3
// Define the red green blue channels on the PWM driver as currently wired
#define RED 1
#define GREEN 0
#define BLUE 2
//#define LOGFADE
#define STEPSIZE 8
#define STEPDELAYUP 15
#define STEPDELAYDOWN 15

#define MINBRIGHT 0 // up to 4096 max, 0 min
#define MAXBRIGHT 4096 // up to 4096 maximum
double R, R255;  // Used for scaling brightness of LEDs
// The number of Steps between the output being on and off
const int pwmIntervals = 4096;
const int pwmIntervals255 = 255;

//--------- RGB LED setup --------------------------
// Create object for the onboard red green blue LED
ONBOARDLED onboardrgb;


struct Color{
  uint16_t r;
  uint16_t g;
  uint16_t b;
};
typedef struct Color mycolor;


int brightness = 0;

//-----------------------------------------------------
void setup() {
  Serial.begin(57600);
  Serial.println(F("Hello"));
  // Set BUTTON1 as an input
  pinMode(BUTTON1, INPUT_PULLUP);
  // PWM chip pin
  pinMode(PWM_OE, OUTPUT);
  digitalWrite(PWM_OE, LOW); // Enable PWM chip
  onboardrgb.begin(); // Setup RGB with default pins (9,6,5)

  //------------ PWM chip initialization ----------------------------------
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
  // Set all PWM channels off initially
  for (byte i = 0; i < 16; i++){
    pwm.setPWM(i, 0, 0); // Channel, on, off (relative to start of 4096-part cycle)  
  }

  // Calculate the R variable (only needs to be done once at setup)
  // These 2 values are used to help apply a scaling factor to the final
  // LED light output to achieve a nearly linear brightness output
  // See here: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
  R = (pwmIntervals * log10(2))/(log10(4096));
  R255 = (pwmIntervals255 * log10(2))/(log10(255));

}

void loop() {
     uint16_t rOut, gOut, bOut = 0;
     

    // Values for angle can vary from 0 to 360, representing location (degrees)
    // around the color wheel, going from red to orange to yellow to green to 
    // aqual to blue to purple to pink back to red
    int angle = 23;  // 22 = orangish red
    int maxbright = 3500;
    for (int v = MINBRIGHT; v < maxbright; v += STEPSIZE){
      onboardrgb.writeHSV(angle,1, v / (float)maxbright, R255);
      convertHSV(angle, 1, v / (float)maxbright, R, rOut, gOut, bOut);
      mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut);
      delay(STEPDELAYUP);
    }
    for (int v = maxbright-1; v >= MINBRIGHT; v -= STEPSIZE){
      onboardrgb.writeHSV(angle,1, v / (float)maxbright, R255);
      convertHSV(angle, 1, v / (float)maxbright, R, rOut, gOut, bOut);
      mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut);
      delay(STEPDELAYDOWN);
    }

    angle = 305;  // 300 = purple
    for (int v = MINBRIGHT; v < MAXBRIGHT; v += STEPSIZE){
      onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHT, R255);
      convertHSV(angle, 1, v / (float)MAXBRIGHT, R, rOut, gOut, bOut);
      mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut);
      delay(STEPDELAYUP);
    }
    for (int v = MAXBRIGHT-1; v >= MINBRIGHT; v -= STEPSIZE){
      onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHT, R255);
      convertHSV(angle, 1, v / (float)MAXBRIGHT, R, rOut, gOut, bOut);
      mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut);
      delay(STEPDELAYDOWN);
    }

}  // end of main loop
