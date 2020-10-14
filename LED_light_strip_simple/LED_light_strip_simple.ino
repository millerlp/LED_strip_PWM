/* 
 *  Run LED light strip off of MusselBedHeater RevF board
 *  with PCA9685 PWM driver.
 *  The light strip is a Govee H6160 strip, which has 4
 *  lines: +12V, R, G, B, arranged in a common anode arrangement
 *  and is driven by sending a PWM signal to each of the 3 color
 *  channels. The LEDs are not individually addressable, the entire
 *  strip lights up with the same color.
 *  
 *  Pins PC1 and PC2 on the board can be connected to ground to 
 *  enable alternate light modes (halloween, xmas, red-white-blue)
 *  
 */

#include <Wire.h>
#include "LED_light_strip_PWM_lib.h"  // https://github.com/millerlp/LED_light_strip_PWM_lib
#include "Adafruit_PWMServoDriver.h" // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include "RTClib.h" // https://github.com/millerlp/RTClib
// User-settable values
#define STARTHOUR 17 // What hour of the day should lights turn on?
#define STOPHOUR  22 // What hour of the day should lights turn off?
#define LOGFADE  // Comment out to disable the logarithmic light fade
#define STEPSIZE 8  // Number of PWM steps to skip
#define STEPDELAYUP 15 // milliseconds between fade steps
#define STEPDELAYDOWN 10 // milliseconds between fade steps
#define MAXBRIGHTPURPLE 3500  // Maximum value is 4096
#define MAXBRIGHTORANGE 3500  // Maximum value is 4096
#define MAXBRIGHTWHITE 3000   // Maximum value is 4096


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



// --------------
// Leave these values alone
#define MINBRIGHT 0 // up to 4096 max, 0 min
#define MAXBRIGHT 4096 // up to 4096 maximum
double R, R255;  // Used for scaling brightness of LEDs
// The number of Steps between the output being on and off
const int pwmIntervals = 4096;  // For 12-bit PCA9685 PWM chip
const int pwmIntervals255 = 255; // For ATmega built-in PWM 
uint16_t rOut, gOut, bOut = 0; // use to pass around r,g,b PWM values

//--------- RGB LED setup --------------------------
// Create object for the onboard red green blue LED
ONBOARDLED onboardrgb;



//-------------------------------------------------------------
// Real Time Clock DS3231M  
// Create real time clock object
RTC_DS3231 rtc;
char buf[20]; // declare a string buffer to hold the time result
DateTime newtime;
DateTime oldtime; // used to track time in main loop
byte debounceTime = 20; // milliseconds to wait for debounce

bool manualRunFlag = false; // Used to manually start light cycle
volatile bool button1Flag = false;
bool timeRunFlag = false; // Used to turn on light at set times

typedef enum MODE
{
  HALLOWEEN, // Halloween mode
  XMAS, // Xmas mode
  RWB, // red-white-blue mode
  HANUKKAH, // HANUKKAH mode
} mode_t;
// mode state machine variable, this takes on the various
// values defined for the MODE typedef above. 
mode_t modeState;


//-----------------------------------------------------
void setup() {
  Serial.begin(57600);
  Serial.println(F("Hello"));
  // Set BUTTON1 as an input
  pinMode(BUTTON1, INPUT_PULLUP);
  attachInterrupt(0, buttonFunc, LOW);
  pinMode(PC1, INPUT_PULLUP); // connect PC1 to GND to change light mode
  pinMode(PC2, INPUT_PULLUP); // connect PC2 to GND to change light mode
  delay(10);
  //----------- Light mode ------------
  // Read PC1, PC2 to determine what mode to use
  bool PC1value = digitalRead(PC1);
  bool PC2value = digitalRead(PC2);
  if (PC1value && PC2value){
    // PC1 and PC2 are both high
    modeState = HALLOWEEN;
    Serial.println("Halloween");
  } else if (!PC1value && PC2value) {
    // PC1 is low, PC2 is high
    modeState = XMAS;
    Serial.println("Xmas");
  } else if (PC1value && !PC2value) {
    // PC1 is high, PC2 is low
    modeState = RWB;
    Serial.println("Red White Blue");
  } else if (!PC1value && !PC2value){
    // PC1 and PC2 are both low
    modeState = HANUKKAH;
    Serial.println("Hanukkah");
  }


  //-----------------------------------------------
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

  //------------ Real Time Clock setup ------------------------------------
  // Initialize the real time clock DS3231M
  Wire.begin(); // Start the I2C library with default options
  Wire.setClock(400000L); // Set speed to 400kHz
  rtc.begin();  // Start the rtc object with default options
  printTimeSerial(rtc.now()); // print time to serial monitor
  Serial.println();
  newtime = rtc.now(); // read a time from the real time clock
  //-----------------------
  // Check that real time clock has a reasonable time value
  if ( (newtime.year() < 2020) | (newtime.year() > 2035) ) {
    // There is an error with the clock, halt everything
    while(1){
    // Flash the error led to notify the user
    // This permanently halts execution, no data will be collected
      onboardrgb.setColor(127,0,0);  // red
      delay(250);
      onboardrgb.setColor(0,127,0); // green
      delay(250);
      onboardrgb.setColor(0,0,127); // blue
      delay(250);
      onboardrgb.setColor(0,0,0);  // off
      delay(250);
    } // while loop end
  }

}

//--------------------------------------------------
void loop() {
     int angle = 0;
    // Start by checking the status of the button1 flag
    if (button1Flag){
      // Button1 was pressed
      attachInterrupt(0, buttonFunc, LOW); // restart the interrupt
      button1Flag = false; // reset the flag
      manualRunFlag = !manualRunFlag; // Switch manualRunFlag
    }

    // Update the clock time and see if it's time to be on
    newtime = rtc.now();
    if ( (newtime.hour() >= STARTHOUR) & (newtime.hour() < STOPHOUR) ) {
      // If it's within the start and stop hours of the day, turn on
      timeRunFlag = true;
    } else {
      timeRunFlag = false;
    }

    // If either manualRunFlag or timeRunFlag are true, run the lights
    if (timeRunFlag | manualRunFlag ){
      switch(modeState){
        case HALLOWEEN:
          // Do Halloween stuff
          // Values for angle can vary from 0 to 360, representing location (degrees)
          // around the color wheel, going from red to orange to yellow to green to 
          // aqual to blue to purple to pink back to red
          angle = 23;  // 22 = orangish red
          for (int v = MINBRIGHT; v < MAXBRIGHTORANGE; v += STEPSIZE){
            onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTORANGE, R255);
            convertHSV(angle, 1, v / (float)MAXBRIGHTORANGE, rOut, gOut, bOut);
            mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut, R);
            delay(STEPDELAYUP);
          }
          for (int v = MAXBRIGHTORANGE-1; v >= MINBRIGHT; v -= STEPSIZE){
            onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTORANGE, R255);
            convertHSV(angle, 1, v / (float)MAXBRIGHTORANGE, rOut, gOut, bOut);
            mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut, R);
            delay(STEPDELAYDOWN);
          }
          // Purple
          
          angle = 305;  // 300 = purple
          for (int v = MINBRIGHT; v < MAXBRIGHTPURPLE; v += STEPSIZE){
            onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTPURPLE, R255);
            convertHSV(angle, 1, v / (float)MAXBRIGHTPURPLE, rOut, gOut, bOut);
            mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut, R);
            delay(STEPDELAYUP);
          }
          for (int v = MAXBRIGHTPURPLE-1; v >= MINBRIGHT; v -= STEPSIZE){
            onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTPURPLE, R255);
            convertHSV(angle, 1, v / (float)MAXBRIGHTPURPLE, rOut, gOut, bOut);
            mysetpwm(pwm, RED, GREEN, BLUE, rOut, gOut, bOut, R);
            delay(STEPDELAYDOWN);
          }
        break;

        case XMAS:
          // Do Xmas routine
        break;

        case RWB:
          // Do red white blue routine
        break;

        case HANUKKAH:
          // Do hanukkah routine
        break;
      } // end switch (modeState)
    }

     


    // Bright white
//    for (int v = MINBRIGHT; v < MAXBRIGHTWHITE; v += STEPSIZE){
//      onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTWHITE, R255);
////      convertHSV(angle, 1, v / (float)MAXBRIGHTWHITE, R, rOut, gOut, bOut);
//      mysetpwm(pwm, RED, GREEN, BLUE, v, v, v, R);
//      delay(STEPDELAYUP);
//    }
//    for (int v = MAXBRIGHTWHITE-1; v >= MINBRIGHT; v -= STEPSIZE){
//      onboardrgb.writeHSV(angle,1, v / (float)MAXBRIGHTWHITE, R255);
////      convertHSV(angle, 1, v / (float)MAXBRIGHTWHITE, R, rOut, gOut, bOut);
//      mysetpwm(pwm, RED, GREEN, BLUE, v, v, v, R);
//      delay(STEPDELAYDOWN);
//    }

}  // end of main loop


//--------------- buttonFunc --------------------------------------------------
// If button 1 is pressed (triggering interupt), arrive here.
// Implement a basic debounce routine (wait 20ms and check button 1 again)
// if still LOW, the button press is real. Set the button1Flag and return

void buttonFunc(void){
  detachInterrupt(0); // Turn off the interrupt
  delay(debounceTime);
  if (digitalRead(BUTTON1) == LOW){
    button1Flag = true;
  }
}
