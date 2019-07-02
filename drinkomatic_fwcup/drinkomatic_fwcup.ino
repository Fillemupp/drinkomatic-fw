/*

drinkOmatic firmware for micro controller inside cup holder 
by Jimmy Eiterjord 2018

Upload with Arduino IDE to Arduino Nano 328 

Will be connected and controlled by drinkomatic-server through
serial port

*/

#define VERSION   "C0.2"

#define LED_DELAY          10  // milliseconds between each LED animation update

#define RGB_LED_PIN        5
#define RGB_LEDS_CIRCLE1   35
#define RGB_LEDS_CIRCLE2   12
#define RGB_LED_COUNT     (RGB_LEDS_CIRCLE1 + RGB_LEDS_CIRCLE2)

#define INBUFLEN  200

// Includes for RFID
#include <SPI.h>
#include <RFID.h>
#define SDA_DIO 9 // RFID define the DIO used for the SDA (SS)
#define RESET_DIO 8 // RFID define RST (reset) pin
#define RFID_DELAY 100 // Time in ms between check for RFID cards
RFID RC522(SDA_DIO, RESET_DIO);  // Create an instance of the RFID library
long handleRFIDLast = 0;
int handleRFIDisCardCount = 0; // Use counter to handle incorrect isCard flip to zero while card is present

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

  // Enable RFID reader
  SPI.begin();
  RC522.init();
  
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

  // Check for progress command
  else if (command[0] == 'P') {
    Serial.print("# Progress update ");
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
    Serial.println(); 
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
          // Show green small circle when RFID card is present
          if (handleRFIDisCardCount > 0) {
            for (int i=0; i<RGB_LEDS_CIRCLE2; i++)
              leds.setPixelColor(i+RGB_LEDS_CIRCLE1, 0, 255, 0);
          } else {
            for (int i=0; i<RGB_LEDS_CIRCLE2; i++)
              leds.setPixelColor(i+RGB_LEDS_CIRCLE1, 0, 0, 0);
          }            
          leds.show();
        }
        break;

      case RUNNING:
        // If just changed to this state, initialize
        if (changedState) {
          for (int i=0; i<RGB_LED_COUNT; i++)
            leds.setPixelColor(i, 0,0,0);
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
                leds.setPixelColor(i, 0, 0, 255); // Set LEDs R G B
              } else {
                leds.setPixelColor(i, 0, 0, 0); // Set LEDs R G B
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

        if (handleRFIDisCardCount == 0) {
          changeState(STANDBY);
        }
       

        break;
    }
}

void handleRFID() {
  if (millis() > handleRFIDLast + RFID_DELAY ) {
    handleRFIDLast = millis();
    bool isCard = RC522.isCard();
    if (isCard) {
      if (handleRFIDisCardCount == 0) {
        RC522.readCardSerial();
        Serial.print("R:");
        for(int i=0;i<5;i++) {
          Serial.print(RC522.serNum[i],HEX);
        }
        Serial.println();
        Serial.flush();
      }
      handleRFIDisCardCount = 2;
    } else {
      if (handleRFIDisCardCount == 2) {
        handleRFIDisCardCount = 1;
      } else if (handleRFIDisCardCount == 1) {
        handleRFIDisCardCount = 0;
        Serial.println("R:NONE");
        Serial.flush();
      }
    }
  }
}

void loop () {
  handleCom();
  handleRFID();
  handleStates();
}

