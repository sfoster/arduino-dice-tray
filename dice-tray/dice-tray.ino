#include "FastLED.h"
// include "ArduinoLowPower.h"
#include "LowPower.h"

#define LED_PIN     6
#define STATUS_PIN 13
#define BUTTON_PIN 2

#define NUM_LEDS 101
#define ROW_COUNT 9
#define COLOR_STOP_COUNT 5

#define STATE_OFF 0
#define STATE_POWERON 1
#define STATE_HOLDON 2
#define STATE_POWEROFF 3

#define FRAME_DELAY_MS 8

#define DEBUG true //set to true for debug output, false for no debug output
#define DEBUG_SERIAL \
  if (DEBUG) Serial
  
typedef struct PixelRow_struct {
  int len;
  int pixelOffset;
  int hueOffset;
} PixelRow;

typedef struct Animation_struct {
  unsigned long duration;
  int startDelay;
  CHSV* colorStops;
} Animation;

PixelRow row0 = { .len = 12, .pixelOffset = 0,                           };
PixelRow row1 = { .len = 12, .pixelOffset = row0.len,                    };
PixelRow row2 = { .len = 12, .pixelOffset = row1.pixelOffset + row1.len, };
PixelRow row3 = { .len = 12, .pixelOffset = row2.pixelOffset + row2.len, };
PixelRow row4 = { .len = 8,  .pixelOffset = row3.pixelOffset + row3.len, };
PixelRow row5 = { .len = 9,  .pixelOffset = row4.pixelOffset + row4.len, };
PixelRow row6 = { .len = 12, .pixelOffset = row5.pixelOffset + row5.len, };
PixelRow row7 = { .len = 12, .pixelOffset = row6.pixelOffset + row6.len, };
PixelRow row8 = { .len = 12, .pixelOffset = row7.pixelOffset + row7.len, };

// For a single strip of leds where each pixel represents a "row" of 1
//PixelRow row0 = { .len = 1, .pixelOffset = 0,                           };
//PixelRow row1 = { .len = 1, .pixelOffset = row0.len,                    };
//PixelRow row2 = { .len = 1, .pixelOffset = row1.pixelOffset + row1.len, };
//PixelRow row3 = { .len = 1, .pixelOffset = row2.pixelOffset + row2.len, };
//PixelRow row4 = { .len = 1,  .pixelOffset = row3.pixelOffset + row3.len, };
//PixelRow row5 = { .len = 1,  .pixelOffset = row4.pixelOffset + row4.len, };
//PixelRow row6 = { .len = 1, .pixelOffset = row5.pixelOffset + row5.len, };
//PixelRow row7 = { .len = 1, .pixelOffset = row6.pixelOffset + row6.len, };
//PixelRow row8 = { .len = 1, .pixelOffset = row7.pixelOffset + row7.len, };

PixelRow* allRows[ROW_COUNT] = { &row0, &row1, &row2, &row3, &row4, &row5, &row6, &row7, &row8 };
CRGB leds[NUM_LEDS];

CHSV hsvBlack =     CHSV(0,  0,  0);    // starting color in HSV format
CHSV hsvDarkRed =   CHSV(244,255,127);  // dark red
CHSV hsvRed =       CHSV(0,  255,255);  // red
CHSV hsvOrange =    CHSV(15, 255,255);  // orange
CHSV hsvWhite =     CHSV(255,0,255);    // white
CHSV hsvFullOn = CHSV(20, 205,255);

CHSV colorStart = hsvBlack;  // target color in HSV format
CHSV colorTarget = hsvBlack; // target color in HSV format
CHSV colorCurrent = colorStart;

// An array of references to colors; these are our color stops in the animation sequence
CHSV* colorStops[COLOR_STOP_COUNT] = { &hsvBlack, &hsvDarkRed, &hsvRed, &hsvOrange, &hsvFullOn };

// Waking up or going to sleep state
// Declare it volatile since it's incremented inside an interrupt
volatile bool waking = 0;

// Program state - either powering-on, hold-steady or powering-off
volatile byte animationState = 0;

unsigned long startTime;
unsigned long elapsedMs;
volatile unsigned long lastInterruptTime = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime = 0;            //
const unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers
const unsigned long onTimeBeforeSleepMs = 600000; // Hold at least 10mins before going back to sleep
const unsigned long shutdownDuration = 1500;
int buttonState = LOW;

void setup() {
  Serial.begin(19200);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(34);
  FastLED.setMaxRefreshRate(120);

  // The button drives wake up and can advance through the animation states
  pinMode(BUTTON_PIN, INPUT);

  // We want to blink the built-in LED
  pinMode(STATUS_PIN, OUTPUT);
  DEBUG_SERIAL.println("setup, BUTTON_PIN current value: " + (String)digitalRead(BUTTON_PIN));
  
  resetState();
  delay(100);
  FastLED.clear();
  FastLED.show();
}

// How long each row of leds is staggered from the previous in the power on animation
unsigned long rowOffsetDelay = 500;
// How long each color stop in the animation takes
unsigned long perStopDuration = 1000;
// We need a total animation duration to calculate the time and frames per color stop
// animationDuration is how long each row will take 
unsigned long animationDuration = perStopDuration * (COLOR_STOP_COUNT - 1); // do we hold on the last color??? or not (-1)
// Essentially the time for the last row to finish its start animation
unsigned long startAnimationDuration = animationDuration + (ROW_COUNT - 1) * rowOffsetDelay;

void waitForTicks(int tickCount) {
  for (; tickCount > 0; tickCount--) {
    delay(FRAME_DELAY_MS);
    // also check button state each tick
    buttonState = digitalRead(BUTTON_PIN);
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (buttonState == HIGH) {
        DEBUG_SERIAL.println("In waitForTicks, buttonState went high, animationState: " + (String)animationState);
        animationState += 1;
        lastDebounceTime = millis();
      }
    }
  }
}
/* 
 *  Set LEDs in a row to a color representing a point on a line through a set of color stops, 
 */
void fadeInRow(int rowIdx, int progressMs) {
  // how far into the animation for this row are we?
  int lastStop = COLOR_STOP_COUNT - 1;
  int stopIdx = min(lastStop, getProgress(progressMs, animationDuration, lastStop));
  int stepStartMs = perStopDuration * stopIdx;
  int targetIdx = min(lastStop, stopIdx + 1);

  // how far into this step of the animation are we?
  fract8 stepProgress = (fract8)getProgress(progressMs - stepStartMs, perStopDuration, 255);
  
  colorStart = *colorStops[stopIdx];
  colorTarget = *colorStops[targetIdx];
    
  colorCurrent = blend(colorStart, colorTarget, stepProgress, SHORTEST_HUES);
  if (rowIdx == 0) {
    // Only log out the first row to reduce noise
    // DEBUG_SERIAL.println((String)rowIdx + ": progressMs: " + (String)progressMs + "/" + (String)animationDuration + 
    //               ", stop/targetIdx: [" +(String)stopIdx+ "," +(String)targetIdx+ "], stepProgress:" + (String)stepProgress +
    //               ", colorCurrent: " + hsvToString(colorCurrent) +
    //               ", target: " + hsvToString(colorTarget));
  }
  updateRow(leds, allRows[rowIdx], colorCurrent);
}

/* 
 *  Animate each row of LEDs to a predefined "on" color and hold it there for a period
 *   - A change to the animationState (by pressing the button) will break the loop
 */
void holdOn() {
  startTime = millis();
  int ledState = LOW;
  unsigned long interval = 1000;
  updateAllRows(leds, allRows, ROW_COUNT, hsvFullOn);
  FastLED.show();
  DEBUG_SERIAL.println("holdOn, all rows should be hsvFullOn");
  digitalWrite(STATUS_PIN, ledState);
  elapsedMs = 0;
  while (animationState == STATE_HOLDON && elapsedMs < onTimeBeforeSleepMs) {
    waitForTicks((int)(interval / FRAME_DELAY_MS));
    ledState = ledState ^ 1;
    digitalWrite(STATUS_PIN, ledState);

    elapsedMs = millis() - startTime;
    DEBUG_SERIAL.println("holdOn, elapsed:" + (String)elapsedMs);
    // start blinking the status LED when there's 5 seconds remaining
    if (onTimeBeforeSleepMs - elapsedMs < 5499) {
      interval = 250;
    }
  }
  ledState = 0;
  digitalWrite(STATUS_PIN, ledState);
  DEBUG_SERIAL.println("holdOn, complete resuming main loop");
}

/* 
 *  Animate each row of LEDs to a predefined "off" color
 *   - A change to the animationState (by pressing the button) will break the loop
 */
void playShutdownAnimation() {
  colorStart = colorCurrent;
  colorTarget = hsvBlack;
  startTime = millis();
  // The loop is conditional on the animationState as another button press should skip to the next state
  while(animationState == STATE_POWEROFF) {
    elapsedMs = millis() - startTime;
    if (elapsedMs >= shutdownDuration) {
      break;
    }
    // how far into this step of the animation are we?
    fract8 stepProgress = (fract8)getProgress(elapsedMs, shutdownDuration, 255);
    colorCurrent = blend(colorStart, colorTarget, stepProgress, SHORTEST_HUES);
    DEBUG_SERIAL.println("playShutdownAnimation, elapsed:" + (String)elapsedMs + "/" + (String)shutdownDuration + ", stepProgress:" + (String)stepProgress + ", colorCurrent:" + hsvToString(colorCurrent));
    updateAllRows(leds, allRows, ROW_COUNT, colorCurrent);
    FastLED.show();
    waitForTicks(1);
  }
  DEBUG_SERIAL.println("playShutdownAnimation, complete, ending at colorCurrent:" + hsvToString(colorCurrent));
}

/* 
 *  Staggered animation which fades in each row of LEDs
 *   - The fade-in is a progression through a series of color stops
 *   - A change to the animationState (by pressing the button) will break the loop
 */
void playPowerOnAnimation() {
  startTime = millis();
  elapsedMs = 0;
  int rowIdx = 0;
  int delayOffset = 0;
  
  while(animationState == STATE_POWERON) {
    elapsedMs = millis() - startTime;
    if (elapsedMs >= startAnimationDuration) {
      DEBUG_SERIAL.println("playPowerOnAnimation, done");
      break;
    }
    // Update each row, with progress offset with the delayOffset
    for (rowIdx = 0; rowIdx < ROW_COUNT; rowIdx++) {
      delayOffset = rowIdx * rowOffsetDelay;
      if (elapsedMs >= delayOffset) {
        fadeInRow(rowIdx, min(animationDuration-1, elapsedMs - delayOffset));    
      }
    }
    FastLED.show();
    waitForTicks(1);
  }
}

void powerOn() {
  playPowerOnAnimation();
}

void resetState() {
  FastLED.clear();
  FastLED.show();
  animationState = 0;
  startTime = 0;
  elapsedMs = 0;
  waking = 0;
}

void sleepNow() {
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    waking = 0;
    delay(500); // Delay before sleep for stabilization
    DEBUG_SERIAL.println("sleepNow, re-attaching the interrupt");
    EIFR |= (1 << INTF0); // Clear interrupt flag for INT0 (digital pin 2)
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonInterrupt, RISING); // Interrupt on RISING signal

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
}

void loop() {
  // BUTTON_PIN is used to wake up and to switch states
  // this outer loop is just handling running/sleeping
  DEBUG_SERIAL.println("Going to sleep to wait for interrupt from BUTTON_PIN");
  sleepNow();

  DEBUG_SERIAL.println("Woke up, animationState:" + (String)animationState + ", button state:" + (String)digitalRead(BUTTON_PIN));

  // Only resume and handle the interrupt if sufficient time elapsed
  if (waking == 1) {
    // The interrupt woke us up
    if (digitalRead(BUTTON_PIN) == HIGH) {
      // Wait for button release to avoid multiple triggers
      while (digitalRead(BUTTON_PIN) == HIGH) {
        delay(100); // Additional debounce delay after release  
      }
    }
  
    if (animationState == STATE_POWERON) {
      DEBUG_SERIAL.println("powering on");
      powerOn();
      animationState = STATE_HOLDON;
    }
    DEBUG_SERIAL.println("After powerOn, animationState:" + (String)animationState);
    if (animationState == STATE_HOLDON) {
      // stay on
      DEBUG_SERIAL.println("hold on");
      holdOn();
      animationState = STATE_POWEROFF;
    } 
    DEBUG_SERIAL.println("After holdOn, animationState:" + (String)animationState);
    if (animationState == STATE_POWEROFF) {
      // start shutdown
      DEBUG_SERIAL.println("power down");
      playShutdownAnimation();
    }
    DEBUG_SERIAL.println("After playShutdownAnimation, animationState:" + (String)animationState);
    DEBUG_SERIAL.println("Reset state");
    // Make sure all LEDs are off
    FastLED.clear();
    FastLED.show();
    resetState();
    // We'll go back to sleep at the top of the next loop
  }
}

void onButtonInterrupt() {
  unsigned long currentTime = millis();
  DEBUG_SERIAL.println("onButtonInterrupt, elapsed: " + (String)(currentTime - lastInterruptTime));
  if (currentTime - lastInterruptTime > debounceDelay) {
    // unhook the interrupt. We'll re-attach it before powering down
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
    // Handle the interrupt - the loop will resume
    // already awake, switch inner program state
    animationState++;
    waking = 1;
    DEBUG_SERIAL.println("onButtonInterrupt, debounced, waking: 1, animationState: " + (String)animationState);
  }
  lastInterruptTime = currentTime;
  lastDebounceTime = currentTime;
}
