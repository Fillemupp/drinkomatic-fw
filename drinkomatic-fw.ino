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

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define LASER_PIN          8

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN          13   // ANALOG NUMBERING
#define TEMP_1_PIN          14   // ANALOG NUMBERING

#define INBUFLEN  200

#define MOTOR_COUNT 5

#include <TimerOne.h>
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
  {0, 0, 54, 55, 38}, // X
  {0, 0, 60, 61, 56}, // Y
  {0, 0, 46, 48, 62}, // Z
  {0, 0, 26, 28, 24}, // E
  {0, 0, 36, 34, 30}, // Q
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

  Timer1.initialize(1000);
  Timer1.attachInterrupt(handleMotors); // blinkLED to run every 0.15 seconds

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
int step = 0;
void handleMotors() {

  if (step == 0) {
    // Set step signal on enabled motors
    step = 1;
    for (int motor = 0; motor<MOTOR_COUNT; motor++) {
      if (motor_status[motor].steps > 0) {
        digitalWrite(motor_status[motor].pin_step , HIGH);
      }
    }
  } else {
    // Clear step signal on enabled motors and count steps
    step = 0;
    long progbar_current = 0;
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



  /*
  // If pumps are on, then update progress bar
  int progbar_current_led = 24-(int)((long)(24 * (long)progbar_current) / (long)progbar_max);
  if (progbar_current > 0) {
    if (progbar_current_led != progbar_previous_led) {
      progbar_previous_led = progbar_current_led;
      for (int i = 0; i < RGB_LED_COUNT; i++) {
        if (i <  progbar_current_led) {
          leds.setPixelColor(i, 0, 0, 255); // Set LEDs R G B
        } else {
          leds.setPixelColor(i, 0, 0, 0); // Set LEDs R G B
        }
      }
      leds.show();   // ...but the LEDs don't actually update until you call this.
    }
  } else {
    ledStandby();
    progbar_max = 0;
  }
  */

  // Linear increase of speed
  if (stepSpeed > stepSpeedMin)
    stepSpeed--;

}

int ledpos = 0;
void handleLeds() {
  ledpos++;
  leds.setPixelColor(ledpos, 100);
  if (ledpos > RGB_LED_COUNT) {
    ledpos = 0;
  }

  for (int i=0; i<RGB_LED_COUNT; i++)
  {
    leds.setPixelColor(i, 255,0,255);
  }
  leds.show();
}

void ledStandby() {
  for (int i=0; i<51; i++)
  {
    leds.setPixelColor(i, 0,0,0);
  }
  for (int i=52; i<60; i++)
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
