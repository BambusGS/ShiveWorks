// © Jakub Jandus 2023

#include <Arduino.h>    // a library for the Arduino platform
#include "FastLED.h"    // a library for controlling LEDs
#include "ESP32Servo.h" // a library for controlling servos
#include "OneButton.h"  // a library for the push button
#include "MPU6050.h"    // a library for the MPU6050 accelerometer and gyroscope
#include "esp_task_wdt.h"

#include "mqtt.h" // a library for connecting to an MQTT broker and wirelessly controlling the shiver
#include "NTP.h"  // a library for getting the current time from an NTP server and keeping it updated

//------------------------------------//----pins

const int pin_servo = 19;
const int pin_ledA = 17;
const int pin_ledB = 16;
const int pin_button = 4;

//------------------------------------//---variables
// segment status, more in readme.md
enum SegmentStatus
{
  Initializing,
  Connected,
  Pairing,
  Paired,
  Downloading,
  Ready,
  Running,
  Estop,
  Fault
};

SegmentStatus currSegmentStatus = Initializing;

OneButton button(pin_button, false);

CRGB leds[2]; // an array of 2 LEDs

Servo servo;
// servo is in degrees, where 0° is 500uS and 180° is 2500uS
// limit the servo to 45 to 135 degrees physically so as not to damage the segment
const int servoMin = 40;
const int servoMax = 140;
void moveServo(int angleMQTT)
{

  int angle = constrain(angleMQTT, 0, 255); // constrain the angle to 0-255
  angle = map(angle, 0, 255, servoMin, servoMax);
  servo.write(angle);
}

//------------------------------------//---MISC function stubs

void beginPairing(); // do not remove as the function is defined lower in the program, but needs to be called before it is defined
void resetActuationVars();
void actuate();

//------------------------------------//---UI

void buttonClick()
{
  if (currSegmentStatus != Pairing && currSegmentStatus != Estop && currSegmentStatus != Fault && currSegmentStatus != Running)
    beginPairing();
  else
    currSegmentStatus = Fault; // change to Connected possibly
}

void buttonDoubleClick()
{
  // cycle through RGB with 1s delay
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  FastLED.show();
  delay(1200);
  leds[0] = CRGB::Green;
  leds[1] = CRGB::Blue;
  FastLED.show();
  delay(1200);
  leds[0] = CRGB::Blue;
  leds[1] = CRGB::Red;
  FastLED.show();
  delay(1200);

  // turn off LEDs
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  FastLED.show();
  delay(250);

  // sweep servo range up
  for (int i = 0; i < 255; i++)
  {
    moveServo(i);
    delay(40);
  }

  // sweep servo from max to middle
  for (int i = 255; i > 127; i--)
  {
    moveServo(i);
    delay(20);
  }

  // return to the middle position
  moveServo(127);
  delay(250);

  // quick test
  moveServo(0);
  delay(500);
  moveServo(255);
  delay(500);
  moveServo(127);
  delay(250);

  Serial.println("Self-test complete");
  sendSegmentStatus("Self-test complete");
  delay(250);
}

void buttonLongPress()
{
  ESP.restart(); // restart the ESP32
}

//----

// display different colors depending on the segment status
const int blinkInterval = 500;
uint32_t lastLEDupdate = 0; // the last time in milliseconds the LEDs were updated
bool isBlinking = false, isCurrentlyOnOff = false;
void updateLED(SegmentStatus segStat)
{
  CRGB color;
  // save resources by only updating the LEDs when the status changes
  // set the color of the LEDs depending on the current segment status
  switch (segStat)
  {
  case Initializing:
    color = CRGB::Blue;
    isBlinking = false;
    break;
  case Connected:
    color = CRGB::Blue;
    isBlinking = true;
    break;
  case Pairing:
    color = CRGB::Purple;
    isBlinking = true;
    break;
  case Paired:
    color = CRGB::Purple;
    isBlinking = false;
    break;
  case Downloading:
    color = CRGB::Yellow;
    isBlinking = true;
    break;
  case Ready:
    color = CRGB::Green;
    isBlinking = true;
    break;
  case Running:
    color = CRGB::Green;
    isBlinking = false;
    break;
  case Estop:
    color = CRGB::Red;
    isBlinking = false;
    break;
  case Fault:
    color = CRGB::DarkRed;
    isBlinking = true;
    break;
  default:
    break;
  }

  // if the LEDs are blinking, check the interval and set them to black if it's time
  if (isBlinking)
  {
    // set the LEDs to black if they're currently supposed to be off in the blinking part
    if (isCurrentlyOnOff)
      color = CRGB::Black;

    // check if it's time to update the LEDs blinking
    if (millis() - lastLEDupdate > blinkInterval)
    {
      lastLEDupdate = millis();             // update the last time the LEDs were updated
      isCurrentlyOnOff = !isCurrentlyOnOff; // toggle the LEDs on/off variable
    }
  }

  // update the LEDs
  leds[0] = color;
  leds[1] = color;
  FastLED.show();
}

//----

// convert the segment status to a string that can be sent over MQTT
String getSegmentStatusString()
{
  String statusString;
  switch (currSegmentStatus)
  {
  case Initializing:
    statusString = "Initializing";
    break;
  case Connected:
    statusString = "Connected";
    break;
  case Pairing:
    statusString = "Pairing";
    break;
  case Paired:
    statusString = "Paired";
    break;
  case Downloading:
    statusString = "Downloading";
    break;
  case Ready:
    statusString = "Ready";
    break;
  case Running:
    statusString = "Running";
    break;
  case Estop:
    statusString = "Estop";
    break;
  case Fault:
    statusString = "Fault";
    break;
  default:
    break;
  }
  return statusString;
}

//------------------------------------//----setup
bool hasInitializedHW = false;
void initializeHW()
{
  pinMode(pin_button, INPUT_PULLDOWN);
  // initialize the button routines
  button.attachClick(buttonClick);
  button.attachDoubleClick(buttonDoubleClick);
  button.attachLongPressStart(buttonLongPress);
  delay(250);

  // led initialization
  FastLED.addLeds<WS2812B, pin_ledA, GRB>(leds, 1).setCorrection(TypicalLEDStrip); // initialize the LEDs to the FastLED library
  FastLED.addLeds<WS2812B, pin_ledB, GRB>(leds, 1, 1).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(200); // set the brightness of the LEDs
  // set the LEDs to black on startup
  leds[0] = CRGB::White;
  leds[1] = CRGB::White;
  FastLED.show();
  delay(250);

  // servo initialization
  servo.attach(pin_servo); // attach the servo to the pin
  // servo is in degrees, where 0° us 500uS and 180° is 2500uS
  // limit the servo to 45 to 135 degrees physically
  moveServo(127); // set the servo to the middle position
  delay(250);

  hasInitializedHW = true;
  delay(250);
}

//---------------------

bool hasInitializedMQTT = false;
void setup()
{
  Serial.begin(115200); // initialize the serial monitor
  Wire.begin(21, 22);   // initialize the I2C bus with SDA on pin 21 and SCL on pin 22
  delay(250);
  if (mpu6050_init() != ESP_OK)
  {
    Serial.println("Error initializing MPU6050");
  }

  initializeHW();       // initialize the LEDs, button, and servo
  // try to connect to the wifi 5 times
  while (!wifi_reconnect(false) && getConnectionAttemptsWiFi() < 5)
  {
    delay(200);                   // a safety delay to prevent the ESP32 from crashing
    updateLED(currSegmentStatus); // update the LED state
  }
  WiFi.status() == WL_CONNECTED ? currSegmentStatus = Initializing : currSegmentStatus = Fault;
  if (currSegmentStatus == Fault) // early exit if the wifi connection fails
  {
    WiFi.disconnect(); // disconnect from the wifi
    Serial.println("Could not connect to WiFi");
    return;
  }
  delay(250);

  // try to connect to the MQTT broker 5 times, code will not proceed if something does not connect
  while (!mqtt_reconnect() && getConnectionAttemptsMQTT() < 5)
  {
    delay(200);                   // a safety delay to prevent the ESP32 from crashing
    updateLED(currSegmentStatus); // update the LED state
  }
  hasInitializedMQTT = true;
  client.connected() == true ? currSegmentStatus = Connected : currSegmentStatus = Fault;
  if (currSegmentStatus == Fault) // early exit if the wifi connection fails
    return;
  delay(250);

  // subscribe to the general overseer command topic
  client.subscribe(overseerCommandPath, 1);

  // subscribe to the command topic for this segment
  client.subscribe(getSegmentCommandPath().c_str(), 1); // c_str() converts the String to a char array

  // subscribe to the data topic for this segment
  client.subscribe(getSegmentDataPath().c_str(), 1); // c_str() converts the String to a char array

  // synchronize time with NTP
  NTPSetup() == true ? currSegmentStatus = Connected : currSegmentStatus = Fault;
  if (currSegmentStatus == Fault) // early exit if the wifi connection fails
    return;
  delay(250);

  // update the segment's status
  sendSegmentStatus("Connected");
}

//------------------------------------//---MQTT message handling

// a generic function that represents the overseer acknowledging receipt of a message
bool mqttAck = false;
bool checkMQTTAcknowledged()
{
  return mqttAck;
};

// using global variables, and variables defined in mqtt.cpp to handle MQTT messages
static int64_t actuationStartTime = 0, nextActuationTime = 0;
static uint32_t timeOffset = 0;

static uint16_t actuationTimestampNext = 0, actuationTimeStampLast = 0;
static uint8_t actuationValueNext = 127, actuationValueLast = 127;
static uint16_t nextActuationTriByteIndex = 0;

bool handleMQTTmessage()
{
  // check if the message is a command
  if (receivedTopic == overseerCommandPath)
  {
    String commandStr = (char *)receivedMessage;
    if (commandStr == "stop")
    {
      currSegmentStatus = Estop; // detaches the servo from the microcontroller
    }
    else if (commandStr.startsWith("start"))
    {
      if (currSegmentStatus == Ready)
      {
        int colonIndex = commandStr.lastIndexOf(":");
        // convert the epoch milliseconds received; first string to const char*; second const char* to long long (int64_t)
        actuationStartTime = atoll(commandStr.substring(colonIndex + 1).c_str());
        currSegmentStatus = Running;
      }
    }
    else if (commandStr == "reset")
    {
      for (int i = 0; i < receivedMessageLength; i++)
      {
        receivedMessage[i] = 0; // clear the buffer byte by byte
      }

      for (int i = 0; i < receivedDataLength; i++)
      {
        receivedData[i] = 0; // clear the buffer byte by byte
      }
      resetActuationVars();
      currSegmentStatus = Paired;
    }
    else if (commandStr.startsWith("move")) // a manual move command {{move::angle}}
    {
      // find the second colon in the command
      int colonIndex = commandStr.lastIndexOf(":");
      // get the angle from the command
      int angle = commandStr.substring(colonIndex + 1).toInt();
      // move the servo to the angle
      moveServo(angle);
      delay(250);
    }
    return true;
  }

  // command payload will always be a string, while data payload will be a byte array
  if (receivedTopic == getSegmentCommandPath())
  {
    String commandStr = (char *)receivedMessage;
    // if the received message payload is "status_report", send the status report
    if (commandStr == "status_report")
    {
      sendSegmentStatus(getSegmentStatusString().c_str());
    }

    if (commandStr == "ack") // overseer's acknowledgement of a message received
    {
      mqttAck = true;
    }

    if (commandStr == "restart") // restart the ESP32
    {
      ESP.restart();
    }

    if (commandStr == "reset") // clear the buffer
    {
      for (int i = 0; i < receivedMessageLength; i++)
      {
        receivedMessage[i] = 0; // clear the buffer byte by byte
      }

      for (int i = 0; i < receivedDataLength; i++)
      {
        receivedData[i] = 0; // clear the buffer byte by byte
      }

      resetActuationVars();
      currSegmentStatus = Paired;
    }

    if (commandStr.startsWith("move")) // a manual move command {{move::angle}}
    {
      // find the second colon in the command
      int colonIndex = commandStr.lastIndexOf(":");
      // get the angle from the command
      int angle = commandStr.substring(colonIndex + 1).toInt();
      // move the servo to the angle
      moveServo(angle);
      delay(250);
    }
    return true;
  }

  if (receivedTopic == getSegmentDataPath())
  {
    moveServo(receivedData[2]); // pre-move to the starting position
    currSegmentStatus = Ready;
    return true;
  }
  // if the topic is not recognized, return false
  return false;
}

//------------------------------------//---state machine

void beginPairing()
{
  // send the ID to the overseer
  String message = "pairing::" + String(ESP.getEfuseMac(), HEX);
  client.publish(overseerReturnPath, message.c_str());
  sendSegmentStatus("Pairing");
  currSegmentStatus = Pairing;
}

// reset variables after a soft reset command
void resetActuationVars()
{
  actuationStartTime = 0;
  nextActuationTime = 0;
  timeOffset = 0;

  actuationTimestampNext = 0;
  actuationTimeStampLast = 0;
  actuationValueNext = 127;
  actuationValueLast = 127;
  nextActuationTriByteIndex = 0;

  if (!servo.attached())
  {
    servo.attach(pin_servo);
  }
}

// all global variables are defined a section above
void nextActuationValue()
{
  actuationTimeStampLast = actuationTimestampNext;
  actuationValueLast = actuationValueNext;

  // data format is always 3 bytes in little endian format:
  //    2 bytes representing the timestamp as uint16_t
  //    1 byte representing the material value as uint8_t
  byte byteOne = receivedData[nextActuationTriByteIndex * 3 + 0];
  byte byteTwo = receivedData[nextActuationTriByteIndex * 3 + 1];
  byte byteThree = receivedData[nextActuationTriByteIndex * 3 + 2];

  actuationTimestampNext = (byteTwo << 8) + byteOne;
  actuationValueNext = byteThree;
  nextActuationTriByteIndex++;

  if (nextActuationTriByteIndex * 3 >= receivedDataLength) // we have reached the end of actuation data
  {
    nextActuationTriByteIndex = 0;
    timeOffset += actuationTimestampNext;
    actuationTimestampNext = 0;
    // Serial.println(timeOffset);
  }
  nextActuationTime = actuationStartTime + timeOffset + actuationTimestampNext; // lookahead timestamp
  // Serial.println(timeOffset + actuationTimestampNext);
  // Serial.println(actuationTimestamp);
  // Serial.println(actuationValue);
}

void actuate()
{
  if (nextActuationTime == 0) // first program run
  {
    nextActuationValue();
    actuationTimeStampLast = actuationTimestampNext;
    actuationValueLast = actuationValueNext;
    moveServo(actuationValueLast);
  }

  else if (NTP.millis() >= nextActuationTime) // actuate to the next position when the time comes
  {
    nextActuationValue();
    moveServo(actuationValueLast);
  }
}

//------------------------------------//---loop
// the loop performance is ~ 400 Hz
void loop()
{
  int16_t ax, ay, az, gx, gy, gz;
  if (mpu6050_read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
    Serial.printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d\r\n",
                  ax, ay, az, gx, gy, gz);
    // Optionally, process the sensor data (filtering, integration, etc.)
  } else {
    Serial.println("Failed to read sensor data.");
  }
  delay(5);
  button.tick();                // update the button state
  if (!client.loop())           // update the ESP32 MQTT client data and communication
    currSegmentStatus = Fault;  // if the client loop fails or is disconnected, set the status to fault
  updateLED(currSegmentStatus); // update the LED state

  //---- check if a message has been received and handle accordingly
  if (messageReceived)
  {
    messageReceived = false;
    handleMQTTmessage();
  }

  //----state machine

  switch (currSegmentStatus)
  {
  case Initializing:
    // do nothing, but display a color
    break;

  case Connected:
    // if we connect to MQTT and the segment is already paired, it should receive an "ack" message
    if (checkMQTTAcknowledged())
    {
      sendSegmentStatus("Paired");
      currSegmentStatus = Paired;
      mqttAck = false;
    }
    break;

  case Pairing:
    if (checkMQTTAcknowledged())
    {
      sendSegmentStatus("Paired");
      currSegmentStatus = Paired;
      mqttAck = false;
    }
    break;

  case Paired:
    // just check messages, but display a color
    break;

  case Downloading:
    // do stuff
    break;

  case Ready:
    // wait for start command
    break;

  case Running:

    actuate();

    break;

  case Estop:       // emergency stop state: keep connected, but don't physically move
    servo.detach(); // detaches the servo from the microcontroller
    // do nothing, but display red - microcontroller can still be reset by holding down the button
    break;

  case Fault:
    servo.detach();
    // do nothing, but display red - microcontroller can still be reset by holding down the button
    break;

  default:
    break;
  }
}