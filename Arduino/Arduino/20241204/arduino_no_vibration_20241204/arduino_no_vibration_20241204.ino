#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Initialize two IMUs with different I2C addresses
Adafruit_BNO055 imu1 = Adafruit_BNO055(55, 0x28); // IMU1 address 0x28
Adafruit_BNO055 imu2 = Adafruit_BNO055(56, 0x29); // IMU2 address 0x29

const int wPin = 3;
const int sPin = 4;
const int aPin = 5;
const int dPin = 6;

const int upPin = 7;
const int downPin = 8;
const int lPin = 9;
const int rPin = 10;

unsigned long previousMillis = 0; // Last switch time
const unsigned long interval = 500; // Vibration switch interval (milliseconds)
int currentPin = wPin; // Currently vibrating pin
bool vibrationActive = false; // Vibration active flag

// Define state machine states
enum State { IDLE, CLOCKWISE, COUNTERCLOCKWISE };
State currentState = IDLE; // Initialize to IDLE state

void setup(void)
{
  Serial.begin(115200); // Set serial baud rate to 115200
  while (!Serial);
  Serial.println("Initializing dual IMU posture sensors...");

  // Initialize IMU1
  if (!imu1.begin())
  {
    Serial.println("Unable to initialize IMU1! Check connections and I2C address.");
    while (1);
  }

  // Initialize IMU2
  if (!imu2.begin())
  {
    Serial.println("Unable to initialize IMU2! Check connections and I2C address.");
    while (1);
  }

  delay(1000);

  // Use external crystal for better accuracy
  imu1.setExtCrystalUse(true);
  imu2.setExtCrystalUse(true);

  Serial.println("Starting to output posture data...");
  pinMode(wPin, OUTPUT);
  pinMode(sPin, OUTPUT);
  pinMode(aPin, OUTPUT);
  pinMode(dPin, OUTPUT);

  pinMode(upPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(lPin, OUTPUT);
  pinMode(rPin, OUTPUT);
}

void loop(void)
{
  // Get sensor events from IMU1 and IMU2
  sensors_event_t event1;
  imu1.getEvent(&event1);
  sensors_event_t event2;
  imu2.getEvent(&event2);

  // Extract and print Euler angles
  float yaw1 = event1.orientation.x;
  float pitch1 = event1.orientation.y;
  float roll1 = event1.orientation.z;
  float yaw2 = event2.orientation.x;
  float pitch2 = event2.orientation.y;
  float roll2 = event2.orientation.z;

  Serial.print(yaw1, 4);
  Serial.print(",");
  Serial.print(pitch1, 4);
  Serial.print(",");
  Serial.print(roll1, 4);
  Serial.print(",");
  Serial.print(yaw2, 4);
  Serial.print(",");
  Serial.print(pitch2, 4);
  Serial.print(",");
  Serial.println(roll2, 4);

  if (Serial.available() > 0)
  {
    char command = Serial.read(); // Read a character as a command
    switch (command)
    {
      case 'w':
      case 's':
      case 'a':
      case 'd':
      case 'u':
      case 'x':
      case 'l':
      case 'r':
      case 'n':
        deactivateAllVibrators(); // Stop all vibrators when executing other commands
        vibrationActive = false;
        currentState = IDLE; // Switch to IDLE state
        executeSingleVibration(command); // Activate single vibrator based on command
        break;

      case 'c': // Clockwise rotation vibration
        if (currentState != CLOCKWISE) // Switch only if not already in CLOCKWISE state
        {
          deactivateAllVibrators(); // Reset current vibration
          vibrationActive = true;
          currentState = CLOCKWISE; // Switch to CLOCKWISE state
          currentPin = wPin; // Reset currentPin to start position
        }
        break;

      case 'e': // Counterclockwise rotation vibration
        if (currentState != COUNTERCLOCKWISE) // Switch only if not already in COUNTERCLOCKWISE state
        {
          deactivateAllVibrators(); // Reset current vibration
          vibrationActive = true;
          currentState = COUNTERCLOCKWISE; // Switch to COUNTERCLOCKWISE state
          currentPin = wPin; // Reset currentPin to start position
        }
        break;
    }
  }

  // Execute vibration logic based on current state
  switch (currentState)
  {
    case CLOCKWISE:
      rotateClockwise();
      break;

    case COUNTERCLOCKWISE:
      rotateCounterclockwise();
      break;

    case IDLE:
    default:
      // Do nothing in IDLE state
      break;
  }
}

// Activate single vibrator
void executeSingleVibration(char command)
{
  switch (command)
  {
    case 'w':
      activateVibrator(wPin, true);
      break;
    case 's':
      activateVibrator(sPin, true);
      break;
    case 'a':
      activateVibrator(aPin, true);
      break;
    case 'd':
      activateVibrator(dPin, true);
      break;
    case 'u':
      activateVibrator(upPin, true);
      break;
    case 'x':
      activateVibrator(downPin, true);
      break;
    case 'l':
      activateVibrator(lPin, true);
      break;
    case 'r':
      activateVibrator(rPin, true);
      break;
    case 'n':
      deactivateAllVibrators();
      vibrationActive = false;
      break;
  }
}

// Clockwise vibration function
void rotateClockwise()
{
    analogWrite(wPin, 0);
    analogWrite(dPin, 0);
}

// Counterclockwise vibration function
void rotateCounterclockwise()
{
 
    analogWrite(wPin, 0);
    analogWrite(aPin, 0);
  
}

// Activate single vibrator
void activateVibrator(int pin, bool state)
{
  analogWrite(pin, state ? 0 : 0); // PWM signal
}

// Stop all vibrators
void deactivateAllVibrators()
{
  analogWrite(wPin, 0);
  analogWrite(sPin, 0);
  analogWrite(aPin, 0);
  analogWrite(dPin, 0);

  analogWrite(upPin, 0);
  analogWrite(downPin, 0);
  analogWrite(lPin, 0);
  analogWrite(rPin, 0);
}
