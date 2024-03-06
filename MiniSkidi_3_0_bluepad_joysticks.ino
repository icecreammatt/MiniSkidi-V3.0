/* ProfessorBoots
   John Cheroske 1/6/2024
   MiniSkidi 3.0

   Thank you to the following people for contributing to this sketch
   -TomVerr99 "Excellent Job organizing what was a very messy intial sketch"
   -CrabRC "I dont even know where to start, but thank you for making numerous improvemnts/suggestions
   across both mechanical designs and software."
   -Fortinbra "Always willing to provide the discord group with a good meme or two, as well as lend a helping hand
   in multiple ways."

  Some tidbits to check

  -Install the esp32 boards manager into the arduino IDE"
  Programming Electronics Academy has a good tutorial: https://youtu.be/py91SMg_TeY?si=m1OWPBPlK-QHJ2Xx"
  -Select "ESP32 Dev Module" under tools>Board>ESP32 Arduino before uploading sketch.
  -The following include statements with comments "by -----" are libraries that can be installed
  directly inside the arduino IDE under Sketch>Include Library>Manage Libraries
*/
#include <Arduino.h>

#include <ESP32Servo.h> // by Kevin Harrington
//#include <ESPAsyncWebSrv.h> // by dvarrel
//#include <PS4Controller.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <Bluepad32.h>
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];


//#if defined(ESP32)
//#include <AsyncTCP.h> // by dvarrel
//#include <WiFi.h>
//#elif defined(ESP8266)
//#include <ESPAsyncTCP.h> // by dvarrel
//#endif


// defines
#define bucketServoPin  23
#define auxServoPin 22
#define lightPin1 18
#define lightPin2 5
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0

#define RIGHT_MOTOR 1
#define LEFT_MOTOR 0
#define ARM_MOTOR 2

#define FORWARD 1
#define BACKWARD -1

// global constants

//extern const char* htmlHomePage PROGMEM;
//const char* ssid     = "ProfBoots MiniSkidi OG";

// global variables

Servo bucketServo;
Servo auxServo;

bool horizontalScreen;//When screen orientation is locked vertically this rotates the D-Pad controls so that forward would now be left.
bool removeArmMomentum = false;
bool light = false;
int aux_pos = 105;
int bucket_pos = 110;

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins =
{
  {25, 26},  //RIGHT_MOTOR Pins (IN1, IN2)
  {33, 32},  //LEFT_MOTOR  Pins
  {21, 19}, //ARM_MOTOR pins
};


void rotateMotor(int motorNumber, int motorDirection)
{
  if (motorDirection == FORWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
  else if (motorDirection == BACKWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  }
  else
  {
    if (removeArmMomentum)
    {
      digitalWrite(motorPins[ARM_MOTOR].pinIN1, HIGH);
      digitalWrite(motorPins[ARM_MOTOR].pinIN2, LOW);
      delay(10);
      digitalWrite(motorPins[motorNumber].pinIN1, LOW);
      digitalWrite(motorPins[motorNumber].pinIN2, LOW);
      delay(5);
      digitalWrite(motorPins[ARM_MOTOR].pinIN1, HIGH);
      digitalWrite(motorPins[ARM_MOTOR].pinIN2, LOW);
      delay(10);
      removeArmMomentum = false;
    }
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
}

void moveCar(ControllerPtr ctl, int inputValue)
{
    switch (inputValue)
    {

      case UP:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);
        Serial.printf("Moving forward\n");
        break;

      case DOWN:
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);

        Serial.printf("Moving backward\n");
        break;

      case LEFT:
        //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);

        Serial.printf("Moving left\n");
        break;

      case RIGHT:
        //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        Serial.printf("Moving right\n");
        break;

      case STOP:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;

      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD);

        Serial.printf("Moving arm up\n");
        break;

      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD);
        removeArmMomentum = true;
        Serial.printf("Moving arm down\n");
        break;

      default:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
    }
}

void bucketTilt(int bucketServoValue)
{
  bucketServo.write(bucketServoValue);
}
void auxControl(int auxServoValue)
{
  auxServo.write(auxServoValue);
}
void lightControl()
{
  Serial.println("Toggling lights!");
  if (!light)
  {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
    light = true;
  }
  else
  {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
    light = false;
  }
}

//void handleRoot(AsyncWebServerRequest *request)
//{
//  request->send_P(200, "text/html", htmlHomePage);
//}

//void handleNotFound(AsyncWebServerRequest *request)
//{
//  request->send(404, "text/plain", "File Not Found");
//}



void setUpPinModes()
{

  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
  }
  //moveCar(STOP);
  bucketServo.attach(bucketServoPin);
  auxServo.attach(auxServoPin);
  auxControl(aux_pos);
  bucketTilt(bucket_pos);

  pinMode(lightPin1, OUTPUT);
  pinMode(lightPin2, OUTPUT);
}


void setup(void)
{
  setUpPinModes();
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    // TODO: Print pressed keys
    Serial.printf("idx=%d\n", ctl->index());
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}
void increment_bucket(ControllerPtr ctl,int increment)
{
  int button_delay = 1;
  bucket_pos += increment;
  if(bucket_pos < 150 && bucket_pos > 0)
  {
  bucketTilt(bucket_pos);
  //delay(button_delay);
  }
  else
  {
    //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
    bucket_pos -= increment;
  }
}

void increment_claw(ControllerPtr ctl, int increment)
{
  aux_pos += increment;
  if(aux_pos < 110 && aux_pos > 40)
  {
    auxControl(aux_pos);
    //delay(button_delay);
  }
  else
  {
    //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
    aux_pos -= increment;
  }
}


void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    int move_command = STOP;

    int minimal_control_input_bucket = 200;
    int right_x = ctl->axisRX();       // (-511 - 512) right X axis
    int right_y =ctl->axisRY();       // (-511 - 512) right Y axis
    if (abs(right_y)>=minimal_control_input_bucket)
    {
        if (right_y > 0) move_command = ARMUP;
        if (right_y < 0) move_command = ARMDOWN;
    }

    // THis codeblock moves the bucket via joysticks
    if (abs(right_x)>=minimal_control_input_bucket)
    {
      int n_steps = right_x/200; // Scale to appropriate speed
      Serial.println("Incrementing bucket ");
      Serial.println(n_steps);

      // Slow down the bucket movement using millis()
      unsigned long currentTime = millis();
      static unsigned long prevTime = currentTime;
      if (currentTime - prevTime >= 15) { // Adjust the interval as needed
          increment_bucket(ctl, -1 * n_steps);
          prevTime = currentTime;
      }

    }

    // This code block moves the claw via R2 and L2
    int throttle = ctl->throttle();
    int brake = ctl->brake();
    if (abs(throttle)>=minimal_control_input_bucket)
    {
      int n_steps = throttle/400;
      increment_claw(ctl,-1*n_steps);
    }
    if (abs(brake)>=minimal_control_input_bucket)
    {
      int n_steps = brake/700;
      increment_claw(ctl,n_steps);
    }

    if(ctl->b())
    {
      increment_bucket(ctl,-1);
      return;
    }
    if(ctl->a())
    {
      increment_bucket(ctl,1);
      return;
    }
    if(ctl->x())
    {
      increment_claw(ctl,1);
      return;
    }
    if(ctl->y())
    {
      increment_claw(ctl,-1);
      return;
    }
    if(ctl->miscButtons() == 0x04)
    {
      lightControl();
      delay(200);
      return;
    }

    int dpad_command = ctl->dpad();


    int left_x = ctl->axisX();        // (-511 - 512) left X Axis
    int left_y = ctl->axisY();        // (-511 - 512) left Y axis

    if (dpad_command != 0x00)
    {
      if (dpad_command == 0x04) move_command = RIGHT;
      else if (dpad_command == 0x02) move_command = DOWN;
      else if (dpad_command == 0x01) move_command = UP;
      else if (dpad_command == 0x08) move_command = LEFT;

    }
    int minimal_control_input = 200;
    if (abs(left_x)>=minimal_control_input)
    {
      Serial.println(left_x);
    }
    if (abs(left_y)>=minimal_control_input && left_y >=0) move_command = DOWN;
    if (abs(left_y)>=minimal_control_input && left_y <=0) move_command = UP;
    if (abs(left_x)>=minimal_control_input && left_x <=0) move_command = LEFT;
    if (abs(left_x)>=minimal_control_input && left_x >=0) move_command = RIGHT;

    int l1_l2_command = ctl->buttons();
    if(l1_l2_command ==0x0010) move_command = ARMUP;
    if(l1_l2_command ==0x0020) move_command = ARMDOWN;

    moveCar(ctl,move_command);
    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    //dumpGamepad(ctl);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void loop()
{

    bool dataUpdated = BP32.update();
    if (dataUpdated)
    {
        processControllers();
    }
    else
    {
      vTaskDelay(1);
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    //delay(150);


}
