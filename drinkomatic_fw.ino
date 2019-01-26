/*

drinkOmatic firmware by Jimmy Eiterjord 2018

Upload with Arduino IDE to Arduino Mega 2560 with RAMPS 1.4 shell

Will be connected and controlled by drinkomatic-server through
serial port

*/

#define MOTORSPEED 100

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define RGB_LED_PIN        5
#define RGB_LED_COUNT      60
#define RGB_LEDS_CIRCLE4   24

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define LASER_PIN          8

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN          13   // ANALOG NUMBERING
#define TEMP_1_PIN          14   // ANALOG NUMBERING

#define INBUFLEN  200

#define MOTOR_COUNT 24

#include "TimerOne.h"

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel leds = Adafruit_NeoPixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

long progbar_max = 0;
int stepSpeed = 400;
int stepSpeedMin = 400;

typedef struct {
  long steps;
  int speed, pin_step, pin_dir,pin_enable;
} motor_status_t;

// RAMPS 1.4 motor config
motor_status_t motor_status[MOTOR_COUNT] = {
  {0, 0, 37, 39, 32}, // X0 M0
  {0, 0, 37, 39, 41}, // Y0 M1
  {0, 0, 37, 39, 43}, // Z0 M2
  {0, 0, 37, 39, 45}, // 00 M3
  {0, 0, 37, 39, 47}, // 10 M4

  {0, 0, 3, 23, 33},  // X1 M5
  {0, 0, 3, 23, 25},  // Y1 M6
  {0, 0, 3, 23, 27},  // Z1 M7
  {0, 0, 3, 23, 29},  // 01 M8
  {0, 0, 3, 23, 31},  // 11 M9

  {0, 0, 54, 55, 38}, // X2 M10  Direct hat PCB
  {0, 0, 60, 61, 56}, // Y2 M11  Direct hat PCB
//  {0, 0, 46, 48, 62}, // Z2 M12  Direct hat PCB. Not working on PCB. Not connected
  {0, 0, 26, 28, 24}, // 02 M12  Direct hat PCB
  {0, 0, 36, 34, 30}, // 12 M13  Direct hat PCB

  {0, 0, 58, 57, 53}, // X3 M14
  {0, 0, 58, 57, 52}, // Y3 M15
  {0, 0, 58, 57, 51}, // Z3 M16
  {0, 0, 58, 57, 49}, // 03 M17
  {0, 0, 58, 57, 50}, // 13 M18

  {0, 0, 59, 64, 63}, // X4 M19
  {0, 0, 59, 64, 44}, // Y4 M20
  {0, 0, 59, 64, 66}, // Z4 M21
  {0, 0, 59, 64, 42}, // 04 M22
  {0, 0, 59, 64, 40}, // 14 M23
};

void setup() {
  Serial.begin(115200);
  Serial.println("#");
  Serial.println("# Startup pumpfirmware v0.1");

  for (int i=0; i<MOTOR_COUNT;i++) {
    motor_status[i].steps = 0;
    motor_status[i].speed = 0;

    pinMode(motor_status[i].pin_step , OUTPUT);
    pinMode(motor_status[i].pin_dir , OUTPUT);
    pinMode(motor_status[i].pin_enable , OUTPUT);

    digitalWrite(motor_status[i].pin_step , LOW);
    digitalWrite(motor_status[i].pin_dir , LOW);
    digitalWrite(motor_status[i].pin_enable , HIGH);
  }

  // Set pin for LASER
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(LED_PIN  , OUTPUT);

  leds.begin();
  clearLEDs();
  leds.show();
  ledStandby();

  Timer1.initialize(stepSpeed);         // initialize timer1, set 1 ms period
  Timer1.attachInterrupt(handleMotors);  // attach hanldeMotors() as timer overflow interrupt

}

char inBuffer[INBUFLEN];
int inBufferP = 0;

void clearLEDs()
{
  for (int i=0; i<RGB_LED_COUNT; i++)
  {
    leds.setPixelColor(i, 0);
  }
}

long motorStartTime;

void handleCommand(char* command) {
  Serial.print("# Command \"");
  Serial.print(command);
  Serial.println("\"");

  // Check for motor command
  if (command[0] == 'M') {
    command++;
    // Read each command pair
    char* subcommand = strtok(command, "&");
    while (subcommand != 0)
    {
      // Split the command in two values
      char* separator1 = strchr(subcommand, ':');
      if (separator1 != 0)
      {
        *separator1 = 0;
        ++separator1;

        char* separator2 = strchr(separator1, ':');
        if (separator2 != 0)
        {
          *separator2 = 0;
          ++separator2;

          int motor = atoi(subcommand);
          long steps = atol(separator1);
          int speed = atoi(separator2);

          Serial.print("# Motor motor:");
          Serial.print(motor);
          Serial.print(" steps:");
          Serial.print(steps);
          Serial.print(" speed:");
          Serial.print(speed);
          Serial.println();

          Serial.println("OK");
          motorStartTime = millis();

          if (motor < MOTOR_COUNT) {
            motor_status[motor].steps = steps;
            motor_status[motor].speed = speed;
            stepSpeed = speed;
            if (stepSpeed < stepSpeedMin)
              stepSpeed = stepSpeedMin;
            Timer1.setPeriod(stepSpeed);         // Set timer interrupt with motor speed

            // Updtae progress bar maximum
            if (steps > progbar_max) {
              progbar_max = steps;
            }

            // Set motor direction
            if (motor_status[motor].speed > 0) {
              digitalWrite(motor_status[motor].pin_dir , LOW);
            } else {
              digitalWrite(motor_status[motor].pin_dir , HIGH);
            }

            // Enable motor driver if steps are available
            if (motor_status[motor].steps > 0) {
               digitalWrite(motor_status[motor].pin_enable , LOW);
             } else {
              digitalWrite(motor_status[motor].pin_enable , HIGH);
            }

          }
        }

      }
      // Find the next command in input string
      subcommand = strtok(0, "&");
    }

  }  // Check for stop command
  else if (command[0] == 'S') {
    command++;

    Serial.println("# Stopping all motors");

    for (int i=0; i<MOTOR_COUNT;i++) {
      motor_status[i].steps = 0;
      motor_status[i].speed = 0;

      digitalWrite(motor_status[i].pin_step , LOW);
      digitalWrite(motor_status[i].pin_dir , LOW);
      digitalWrite(motor_status[i].pin_enable , HIGH);
    }

    Serial.println("OK");

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

int progbar_previous_led = 0;
long progbar_current = 0;
int interruptState = 0;
void handleMotors() {
  if (interruptState == 0) {
    // Set step signal high for all activated motors
    interruptState = 1; // Change to other state for next interrupt
    for (int motor = 0; motor<MOTOR_COUNT; motor++) {
      if (motor_status[motor].steps > 0) {
        digitalWrite(motor_status[motor].pin_step , HIGH);
      }
    }
  }   else {
    // Set step signal low for all activated motors
    interruptState = 0; // Change to other state for next interrupt
    progbar_current = 0;
    for (int motor = 0; motor<MOTOR_COUNT; motor++) {
      if (motor_status[motor].steps > 0) {
        digitalWrite(motor_status[motor].pin_step , LOW);
        motor_status[motor].steps--;

        // Update current progress bar position
        if (motor_status[motor].steps > progbar_current) {
          progbar_current = motor_status[motor].steps;
        }

        // Disable motor driver when zero reached
        if (motor_status[motor].steps == 0) {
           digitalWrite(motor_status[motor].pin_enable , HIGH);
           long motorStopTime = millis();
           Serial.print("# Time in millis: ");
           Serial.println(motorStopTime-motorStartTime);
        }
      }
    }

    // Flash laser when pumps are running
    if (progbar_current > 0) {
      if (millis() % 300 < 100)
        digitalWrite(LASER_PIN, HIGH);
      else
        digitalWrite(LASER_PIN, LOW);
    } else {
      digitalWrite(LASER_PIN, HIGH);
    }

  }

  // Linear increase of speed
  if (stepSpeed > stepSpeedMin) {
    stepSpeed--;
    Timer1.setPeriod(stepSpeed);         // Set timer interrupt with motor speed
  }

//  handleCom();

}

int ledpos = 0;
void handleLeds() {
  // If pumps are on, then update progress bar
  int progbar_current_led = RGB_LEDS_CIRCLE4 -
    (int)((long)(RGB_LEDS_CIRCLE4 * (long)progbar_current) / (long)progbar_max);
//  if (progbar_current > 0) {
    if (progbar_current_led != progbar_previous_led) {
      progbar_previous_led = progbar_current_led;
      // Is end reached?
      if (progbar_current_led == RGB_LEDS_CIRCLE4 + 1) {
        // Yes, show standby green
        for (int i=0; i<RGB_LEDS_CIRCLE4; i++) {
          leds.setPixelColor(i, 0,255,0);
        }
      } else {
        // No, update blue progress bar
        for (int i = 0; i < RGB_LED_COUNT; i++) {
          if (i <  progbar_current_led) {
            leds.setPixelColor(i, 0, 0, 255); // Set LEDs R G B
          } else {
            leds.setPixelColor(i, 0, 0, 0); // Set LEDs R G B
          }
        }
      }
      leds.show();
    }
//  } else {
//    ledStandby();
//    progbar_max = 0;
//  }
  /*
  ledpos++;
  leds.setPixelColor(ledpos, ledpos+millis());
  if (ledpos > RGB_LED_COUNT) {
    ledpos = 0;
  }
  */
  /*
  for (int i=0; i<RGB_LED_COUNT; i++)
  {
    leds.setPixelColor(i, 255,0,255);
  }
  */
//  leds.show();
}

void ledStandby() {
  // Set outer circle green
  for (int i=0; i<RGB_LEDS_CIRCLE4; i++)
  {
    leds.setPixelColor(i, 0,255,0);
  }
  leds.show();
}

void heartBeat() {
  if (millis() % 800 < 100)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);
}

void loop () {
  heartBeat();
  handleCom();
//  handleLeds();
}
