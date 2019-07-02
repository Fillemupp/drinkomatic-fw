/*

drinkOmatic firmware for micro controller with LEDs and Alco meter
by Jimmy Eiterjord 2018

Upload with Arduino IDE to Arduino Mega 2560 

Will be connected and controlled by drinkomatic-server through
serial port

*/

#define VERSION   "L0.2"

#define LED_DELAY          10  // milliseconds between each LED animation update
#define LED_PIN            13

#define ALCO_PIN           A0
#define ALCO_DELAY         50   // Time in ms between check for new alco values
#define ALCO_FILTER_P      0.20 // Factor P for filter on new values
#define ALCO_REPORT_THRESH 2    // Difference requried to report

#define RGB_LED_PIN        5
#define RGB_LEDS_CIRCLE1   35
#define RGB_LEDS_CIRCLE2   12
#define RGB_LED_COUNT     (RGB_LEDS_CIRCLE1 + RGB_LEDS_CIRCLE2)

#define INBUFLEN  200

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel leds = Adafruit_NeoPixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
long progbar_max = 0;
long progbar_current = 0;
int progbar_previous_led = 0;
int progress = 0;

int state = 0;
int changedState = 1;
#define STANDBY 0
#define RUNNING 1
#define FINISHED 2

void setup() {
  Serial.begin(115200);
  Serial.println("#");
  Serial.println("# Startup pumpfirmware");
  Serial.flush();

  Serial.print("V:");
  Serial.println(VERSION);

  leds.begin();
  // Clear LEDs
  for (int i=0; i<RGB_LED_COUNT; i++) {
    leds.setPixelColor(i, 0);
  }
  leds.show();

  changeState(STANDBY);
}

char inBuffer[INBUFLEN];
int inBufferP = 0;

void handleCommand(char* command) {
  Serial.print("# Command \"");
  Serial.print(command);
  Serial.println("\"");
  Serial.flush();

  // Check for version command
  if (command[0] == 'V') {
    Serial.print("V:");
    Serial.println(VERSION);
    Serial.flush();
  }

  // Check for stop command
  else if (command[0] == 'S') {
    Serial.println("# Stopping all motors");
    changeState(STANDBY);
    Serial.println("S:OK");
    Serial.flush();
  }

  // Check for alko sensor command
  if (command[0] == 'A') {
    int alcoraw = analogRead(ALCO_PIN);
    Serial.println("# Reading alcohol sensor");
    Serial.print("A:");
    Serial.println(alcoraw);
    Serial.flush();
  }

  // Check for progress command
  else if (command[0] == 'P') {
    Serial.print("# Progress update ");
    command++;
    if (command[0] == ':') {
      command++;
      char* subcommand = strtok(command, "\n");
      if (subcommand != 0) {
        progress = atoi(subcommand);
        Serial.print(progress);
        if (progress == 0) {
          changeState(STANDBY);
        } else if (progress == 100) {
          changeState(FINISHED);
        } else {
          changeState(RUNNING);
        }
      }
    } else {
      Serial.print("BAD FORMAT");
    }
    Serial.println(); 
    Serial.flush();
  }

  // Unknown command
  else {
    Serial.println("# Unknown command");
    Serial.flush();
  }
}

void handleCom() {
  char inByte = 0;
  if (Serial.available() > 0) {
    inByte = Serial.read();
    inBuffer[inBufferP++] = inByte;
    if (inBufferP > INBUFLEN-1) {
      inBufferP = 0;
      inBuffer[inBufferP] = 0;
    }
    if (inByte == 10) {
      inBuffer[inBufferP-1] = 0;
      handleCommand(inBuffer);
      inBufferP = 0;
      inBuffer[inBufferP] = 0;
    }
  }
}

void changeState(int newState) {
   if (state != newState) {
     state = newState;
     changedState = 1;
   }
}

long lastLEDevent = 0;
int ledr = 250;
int ledg = 0;
int ledb = 100;
void handleStates() {
    switch (state) {

      case STANDBY:
        // If just changed to this state, initialize
        if (changedState) {
          // Set outer circle green
          /*
          for (int i=0; i<RGB_LEDS_CIRCLE4; i++)
            leds.setPixelColor(i, 0,255,0);
          for (int i=0; i<RGB_LEDS_CIRCLE3; i++)
            leds.setPixelColor(i+RGB_LEDS_CIRCLE4, 255,0,0);
          for (int i=0; i<RGB_LEDS_CIRCLE2; i++)
            leds.setPixelColor(i+RGB_LEDS_CIRCLE4+RGB_LEDS_CIRCLE3, 0,0,255);
          for (int i=0; i<RGB_LEDS_CIRCLE1; i++)
            leds.setPixelColor(i+RGB_LEDS_CIRCLE4+RGB_LEDS_CIRCLE3+RGB_LEDS_CIRCLE2, 255,0,255);
          */
          leds.show();
          state = STANDBY;
          changedState = 0;
        }

        if (millis() - lastLEDevent > LED_DELAY) {
          lastLEDevent = millis();
          ledr = (ledr + 5) % 255;
          ledg = (ledg + 1) % 255;
          ledb = (ledb + 2) % 255;
          for (int i=0; i<RGB_LEDS_CIRCLE1; i++)
            leds.setPixelColor(i, (ledr+i)%255,(ledg+i)%255,(ledb+i)%255);
          leds.show();
        }
        break;

      case RUNNING:
        // If just changed to this state, initialize
        if (changedState) {
          for (int i=0; i<RGB_LED_COUNT; i++)
            leds.setPixelColor(i, 0x000000);
          leds.show();
          changedState = 0;
        }

        if (millis() - lastLEDevent > LED_DELAY) {
          lastLEDevent = millis();
          // If pumps are on, then update progress bar
          int progbar_current_led = RGB_LEDS_CIRCLE1 -
            (int)((long)(RGB_LEDS_CIRCLE1 * (long)(100-progress)) / (long)100);
          if (progbar_current_led != progbar_previous_led) {
            progbar_previous_led = progbar_current_led;
            for (int i = 0; i < RGB_LEDS_CIRCLE1; i++) {
              if (i <  progbar_current_led) {
                leds.setPixelColor(i, 0x0000FF); // Set LEDs R G B
              } else {
                leds.setPixelColor(i, 0); // Set LEDs R G B to 0
              }
            }
          }
          leds.show();
        }
        break;

      case FINISHED:
        // If just changed to this state, initialize
        if (changedState) {
          for (int i=0; i<RGB_LEDS_CIRCLE1; i++)
            leds.setPixelColor(i, 0,255,0);
          leds.show();
          changedState = 0;
        }

        changeState(STANDBY);
       
        break;
    }
}

long handleAlcoLast = 0;
float alcov = 0;
float alcovprev = 0;
void handleAlcoSensor() {
  if (millis() > handleAlcoLast + ALCO_DELAY ) {
    handleAlcoLast = millis();
    // Read alco sensor
    int alcoraw = analogRead(ALCO_PIN);
    // Approach towards the current value slowly
    alcov = alcov + (float(alcoraw) - alcov) * ALCO_FILTER_P;
    // Report if different from before
    if (abs(alcov - alcovprev) > ALCO_REPORT_THRESH) {
      alcovprev = alcov;
      Serial.print("A:");
      Serial.print(int(alcov));
      Serial.print(":");
      Serial.print(alcoraw);
      Serial.println();      
      Serial.flush();
    }
  }
}

void handleHeartBeat() {
  // Flash heartbeat slow on LED board
  if (millis() % 800 < 100)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);
}

void loop () {
  handleHeartBeat();
  handleAlcoSensor();
  handleCom();
  handleStates();
}

