#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// ======================== PIN DEFINITIONS ========================
// Motor driver pins (H-Bridge inputs)
#define M1A 18
#define M1B 19
#define M2A 22
#define M2B 23

#define NUM_SENSORS 8

// ======================== PWM CONFIGURATION ========================
// Using ESP32 LEDC hardware PWM for motor speed control
int PWM_FREQ = 10000;       // 10kHz PWM
int PWM_RESOLUTION = 8;     // 8-bit resolution (0–255)

int CH_M1A = 0;
int CH_M1B = 1;
int CH_M2A = 2;
int CH_M2B = 3;

int baseSpeed = 100; // Maximum PWM value for full speed

float Kp = 0.02;  // Proportional gain for line following control
int lineLostThreshold = 400;  // Threshold for considering the line lost (sum of sensor values)

bool estopLatched = false; // Emergency stop latch 
bool wasConnected = false;  // Track Bluetooth connection state

int rightTrim = -10;  // Trim value to adjust for any bias in the motors (positive means right motor is faster, negative means left motor is faster)
int leftTrim  = 0;  // Left motor is baseline, right motor is adjusted by rightTrim. Adjust these values to achieve straight movement when both motors are given the same command.

enum State {
  WAITING_FOR_CONNECTION,
  RUNNING,
  ESTOPPED
};

/*
  Processed drive intent.
*/
struct DriveCommand {
  int throttle;
  int steering;
  int left;
  int right;
};

DriveCommand driveCommand;

/*
  Final PWM output for one motor.
*/
struct MotorOut {
  int pwmA;
  int pwmB;
};

enum StopMode {
  COAST,   // Motor free-spins when stopped
  BRAKE    // Motor actively brakes when stopped
};

MotorOut leftMotor;
MotorOut rightMotor;

// Final confirmed left-to-right order
uint8_t sensorPins[NUM_SENSORS] = {
  25, 26, 27, 14, 4, 16, 17, 5
};

QTRSensors qtr;
uint16_t values[NUM_SENSORS];

MotorOut mapMotorDir(int cmd, bool forwardIsA, StopMode stop) {
  MotorOut m;

  int magnitude = abs(cmd);
  if (magnitude > 255) magnitude = 255;

  // Which pin means "forward" for THIS motor?
  // If forwardIsA=true: forward=A, reverse=B
  // If forwardIsA=false: forward=B, reverse=A
  int forwardPin = forwardIsA ? 0 : 1; // 0 => A, 1 => B
  int reversePin = forwardIsA ? 1 : 0;

  if (cmd > 0) {
    // forward
    m.pwmA = (forwardPin == 0) ? magnitude : 0;
    m.pwmB = (forwardPin == 1) ? magnitude : 0;
  } else if (cmd < 0) {
    // reverse
    m.pwmA = (reversePin == 0) ? magnitude : 0;
    m.pwmB = (reversePin == 1) ? magnitude : 0;
  } else {
    // stop
    if (stop == COAST) {
      m.pwmA = 0;
      m.pwmB = 0;
    } else { // BRAKE
      m.pwmA = 255;
      m.pwmB = 255;
    }
  }

  return m;
}

void computeDriveCommand(int error) {
  driveCommand.throttle = baseSpeed;  // Constant forward throttle
  driveCommand.steering = Kp * error;

  driveCommand.left = driveCommand.throttle + driveCommand.steering;
  driveCommand.right = driveCommand.throttle - driveCommand.steering;

  driveCommand.left  += leftTrim;
  driveCommand.right += rightTrim;

  driveCommand.left  = constrain(driveCommand.left,  -255, 255);
  driveCommand.right = constrain(driveCommand.right, -255, 255);
}

void writeMotorOutputs(const MotorOut left, const MotorOut right) {

  // M1 = RIGHT motor
  ledcWrite(CH_M1A, right.pwmA);
  ledcWrite(CH_M1B, right.pwmB);

  // M2 = LEFT motor
  ledcWrite(CH_M2A, left.pwmA);
  ledcWrite(CH_M2B, left.pwmB);
}

int sensorSum(const uint16_t v[], int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += v[i];
  return (int)sum;
}

void handleStopCommands() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();

    if (c == 'S' || c == 's') estopLatched = true;   // STOP
    if (c == 'R' || c == 'r') estopLatched = false;  // RESUME
  }
}

void setup() {
  SerialBT.begin("ESP32_LineFollower"); // Bluetooth device name
  delay(1000);

  Serial.println("\n=== QTR CALIBRATION ===");

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setTimeout(5000);

  Serial.println("Calibrating for 3 seconds...");
  Serial.println("Move the robot slowly across BLACK and WHITE.");

  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(15);
  }

  digitalWrite(LED_BUILTIN, LOW);

  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  ledcSetup(CH_M1A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M1B, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(M1A, CH_M1A);
  ledcAttachPin(M1B, CH_M1B);
  ledcAttachPin(M2A, CH_M2A);
  ledcAttachPin(M2B, CH_M2B);

  Serial.println("Calibration complete.\n");
}

void loop() {

  bool connected = SerialBT.hasClient();

  if (!connected && wasConnected) {
    estopLatched = true; // Latch estop if connection is lost
  }

  wasConnected = connected;

  if (!SerialBT.hasClient()) {
    // No Bluetooth connection, stop motors and wait
    writeMotorOutputs(mapMotorDir(0, false, COAST), mapMotorDir(0, true , COAST));
    return;
  }

  handleStopCommands();

  if (estopLatched) {
    writeMotorOutputs(mapMotorDir(0, false, COAST), mapMotorDir(0, true , COAST));
    return;
  }

  qtr.readCalibrated(values);
  uint16_t position = qtr.readLineBlack(values);

  int sum = sensorSum(values, NUM_SENSORS);

  static unsigned long last = 0;
  if (millis() - last > 200) {
    last = millis();
    SerialBT.print("sum="); SerialBT.print(sum);
    SerialBT.print(" pos="); SerialBT.print(position);
    SerialBT.print(" vals=");
    for (int i = 0; i < NUM_SENSORS; i++) {
      SerialBT.print(values[i]);
      SerialBT.print(i == NUM_SENSORS - 1 ? "" : ",");
    }
  SerialBT.println();
}

  if (sum < lineLostThreshold) {
    // stop motors (coast)
    leftMotor  = mapMotorDir(0,  /*forwardIsA=*/false, COAST); // LEFT forward is B
    rightMotor = mapMotorDir(0, /*forwardIsA=*/true,  COAST); // RIGHT forward is A
    writeMotorOutputs(leftMotor, rightMotor);
    return; // skip rest of loop
  }

  int error = position - 3500;

  computeDriveCommand(error);
  leftMotor  = mapMotorDir(driveCommand.left,  /*forwardIsA=*/false, BRAKE); // LEFT forward is B
  rightMotor = mapMotorDir(driveCommand.right, /*forwardIsA=*/true,  BRAKE); // RIGHT forward is A

  
  writeMotorOutputs(leftMotor, rightMotor);
}