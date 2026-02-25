#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// ======================== PINS ========================
#define M1A 18
#define M1B 19
#define M2A 22
#define M2B 23

#define NUM_SENSORS 8
uint8_t sensorPins[NUM_SENSORS] = {25, 26, 27, 14, 4, 16, 17, 5};

QTRSensors qtr;
uint16_t values[NUM_SENSORS];

// ======================== PWM ========================
const int PWM_FREQ       = 10000;
const int PWM_RESOLUTION = 8;
const int CH_M1A = 0, CH_M1B = 1, CH_M2A = 2, CH_M2B = 3;

// ======================== TUNING ========================
// Bluetooth commands:
//   P<val>   Kp              e.g. P0.018
//   D<val>   Kd              e.g. D1.500
//   B<val>   baseSpeed       e.g. B55
//   DB<val>  deadband        e.g. DB150
//   PT<val>  pivotMs         e.g. PT350
//   PB<val>  postBumpMs      e.g. PB150
//   PW<val>  pivotPwm        e.g. PW90
//   FW<val>  bumpPwm         e.g. FW65
//   CC<val>  cornerCount     e.g. CC5
//   CT<val>  cornerThreshold e.g. CT700
//   AL<val>  alpha 0.0-1.0   e.g. AL0.40
//   V        print all values

float Kp              = 0.018f;
float Kd              = 1.500f;
int   baseSpeed       = 55;
int   deadband        = 150;

int   pivotMs         = 350;
int   postBumpMs      = 150;
int   pivotPwm        = 90;    // One wheel forward at this, other wheel backward at this
int   bumpPwm         = 65;
int   cornerCount     = 5;
int   cornerThreshold = 700;

float alpha = 0.40f;

int leftTrim  = 0;
int rightTrim = -10;

// ======================== STATE MACHINE ========================
enum RobotState : uint8_t { DISARMED, FOLLOW, PIVOT, BUMP };
RobotState robotState = DISARMED;
unsigned long stateStartMs = 0;

bool   armed        = false;
String rxBuf        = "";

int   lastError     = 0;
float filteredSteer = 0.0f;
int   lastErrorSign = 0;
int   cornerDir     = 1;

unsigned long lastBtPrintMs = 0;

// ======================== HELPERS ========================
static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline void changeState(RobotState s) {
  robotState   = s;
  stateStartMs = millis();
}

int countBlack(uint16_t* vals) {
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if ((int)vals[i] > cornerThreshold) n++;
  return n;
}

// ======================== MOTOR HELPERS ========================
struct MotorOut { int pwmA, pwmB; };

// Full bidirectional — positive = forward, negative = reverse
MotorOut mapMotorDir(int cmd, bool forwardIsA) {
  MotorOut m;
  int mag = clampInt(abs(cmd), 0, 255);
  if (cmd > 0) {
    m.pwmA = forwardIsA ? mag : 0;
    m.pwmB = forwardIsA ? 0   : mag;
  } else if (cmd < 0) {
    m.pwmA = forwardIsA ? 0   : mag;
    m.pwmB = forwardIsA ? mag : 0;
  } else {
    m.pwmA = m.pwmB = 0;
  }
  return m;
}

void writeMotors(MotorOut left, MotorOut right) {
  ledcWrite(CH_M1A, right.pwmA);  // M1 = RIGHT
  ledcWrite(CH_M1B, right.pwmB);
  ledcWrite(CH_M2A, left.pwmA);   // M2 = LEFT
  ledcWrite(CH_M2B, left.pwmB);
}

void stopMotors() {
  MotorOut z = {0, 0};
  writeMotors(z, z);
}

// Follow: clamp to 0..255 — no reverse during line following
void setFollowSpeeds(int leftSpd, int rightSpd) {
  leftSpd  = clampInt(leftSpd,  0, 255);
  rightSpd = clampInt(rightSpd, 0, 255);
  writeMotors(
    mapMotorDir(leftSpd,  false),
    mapMotorDir(rightSpd, true)
  );
}

// True in-place pivot: one wheel forward, opposite wheel backward
// dir=+1 → turn right: left forward, right backward
// dir=-1 → turn left:  right forward, left backward
void doPivotMotors(int dir) {
  int leftCmd  =  dir * pivotPwm;   // +dir = forward for left
  int rightCmd = -dir * pivotPwm;   // -dir = backward for right
  writeMotors(
    mapMotorDir(leftCmd,  false),
    mapMotorDir(rightCmd, true)
  );
}

void driveForward(int spd) {
  setFollowSpeeds(spd, spd);
}

// ======================== BLUETOOTH ========================
void btPrint(const String& s) { SerialBT.println(s); }

void printVars() {
  btPrint(
    "Kp="  + String(Kp, 4)     +
    " Kd=" + String(Kd, 4)     +
    " B="  + String(baseSpeed)  +
    " DB=" + String(deadband)   +
    " AL=" + String(alpha, 3)   +
    " PT=" + String(pivotMs)    +
    " PB=" + String(postBumpMs) +
    " PW=" + String(pivotPwm)   +
    " FW=" + String(bumpPwm)    +
    " CC=" + String(cornerCount) +
    " CT=" + String(cornerThreshold)
  );
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s == "A") { armed = true;  stopMotors(); changeState(FOLLOW);   btPrint("ARMED");   return; }
  if (s == "S") { armed = false; stopMotors(); changeState(DISARMED); btPrint("STOPPED"); return; }
  if (s == "V") { printVars(); return; }

  if (s.startsWith("DB")) { deadband        = s.substring(2).toInt();   btPrint("DB=" + String(deadband));        return; }
  if (s.startsWith("PT")) { pivotMs         = s.substring(2).toInt();   btPrint("PT=" + String(pivotMs));         return; }
  if (s.startsWith("PB")) { postBumpMs      = s.substring(2).toInt();   btPrint("PB=" + String(postBumpMs));      return; }
  if (s.startsWith("PW")) { pivotPwm        = s.substring(2).toInt();   btPrint("PW=" + String(pivotPwm));        return; }
  if (s.startsWith("FW")) { bumpPwm         = s.substring(2).toInt();   btPrint("FW=" + String(bumpPwm));         return; }
  if (s.startsWith("CC")) { cornerCount     = s.substring(2).toInt();   btPrint("CC=" + String(cornerCount));     return; }
  if (s.startsWith("CT")) { cornerThreshold = s.substring(2).toInt();   btPrint("CT=" + String(cornerThreshold)); return; }
  if (s.startsWith("AL")) { alpha           = s.substring(2).toFloat(); btPrint("AL=" + String(alpha, 3));        return; }

  char   cmd = s.charAt(0);
  String val = s.substring(1);
  switch (cmd) {
    case 'P': Kp        = val.toFloat(); btPrint("Kp="   + String(Kp, 4));    break;
    case 'D': Kd        = val.toFloat(); btPrint("Kd="   + String(Kd, 4));    break;
    case 'B': baseSpeed = val.toInt();   btPrint("base=" + String(baseSpeed)); break;
    default:  btPrint("ERR: unknown cmd"); break;
  }
}

void pollBluetooth() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) { handleCommand(rxBuf); rxBuf = ""; }
    } else {
      if (rxBuf.length() < 64) rxBuf += c;
    }
  }
}

// ======================== SETUP ========================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  ledcSetup(CH_M1A, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(M1A, CH_M1A);
  ledcSetup(CH_M1B, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(M1B, CH_M1B);
  ledcSetup(CH_M2A, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(M2A, CH_M2A);
  ledcSetup(CH_M2B, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(M2B, CH_M2B);
  stopMotors();

  SerialBT.begin("ESP32_PD");
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setTimeout(2500);

  digitalWrite(LED_BUILTIN, HIGH);
  btPrint("CALIBRATING...");

  unsigned long start = millis();
  while (millis() - start < 4000) qtr.calibrate();

  digitalWrite(LED_BUILTIN, LOW);
  btPrint("DONE. Send A to arm.");
  changeState(DISARMED);
}

// ======================== LOOP ========================
void loop() {
  pollBluetooth();

  if (!armed || robotState == DISARMED) {
    stopMotors();
    delay(5);
    return;
  }

  unsigned long now     = millis();
  unsigned long elapsed = now - stateStartMs;

  // ----------------------------------------------------------------
  // STATE: PIVOT — true in-place spin (one fwd, one rev)
  // ----------------------------------------------------------------
  if (robotState == PIVOT) {
    doPivotMotors(cornerDir);
    if (elapsed >= (unsigned long)pivotMs) {
      btPrint("PIVOT DONE");
      changeState(BUMP);
    }
    return;
  }

  // ----------------------------------------------------------------
  // STATE: BUMP — short forward nudge to land on the new line
  // ----------------------------------------------------------------
  if (robotState == BUMP) {
    driveForward(bumpPwm);
    if (elapsed >= (unsigned long)postBumpMs) {
      lastError     = 0;
      filteredSteer = 0.0f;
      btPrint("RESUMING");
      changeState(FOLLOW);
    }
    return;
  }

  // ----------------------------------------------------------------
  // STATE: FOLLOW
  // ----------------------------------------------------------------
  int position = (int)qtr.readLineBlack(values);
  int error    = position - 3500;
  int bc       = countBlack(values);

  // Track approach direction (ignore noise near centre)
  if (error < -200)     lastErrorSign = -1;
  else if (error > 200) lastErrorSign = +1;

  // Corner detection
  if (bc >= cornerCount) {
    cornerDir = lastErrorSign;
    if (cornerDir == 0) cornerDir = 1;
    btPrint("CORNER dir=" + String(cornerDir) + " bc=" + String(bc));
    changeState(PIVOT);
    return;
  }

  // Deadband — within ±deadband treat error as zero, motors run equal speed
  int effectiveError = (abs(error) <= deadband) ? 0 : error;

  // PD
  int   derivative = effectiveError - lastError;
  lastError = effectiveError;

  float rawSteer = (Kp * (float)effectiveError) + (Kd * (float)derivative);

  filteredSteer = alpha * rawSteer + (1.0f - alpha) * filteredSteer;

  float steer = filteredSteer;
  if (steer >  200.0f) steer =  200.0f;
  if (steer < -200.0f) steer = -200.0f;

  // Clamp to 0 minimum — line following never reverses, just slows
  int leftSpd  = clampInt((int)((float)baseSpeed + steer) + leftTrim,  0, 255);
  int rightSpd = clampInt((int)((float)baseSpeed - steer) + rightTrim, 0, 255);

  setFollowSpeeds(leftSpd, rightSpd);

  // BT debug every 200ms
  if (now - lastBtPrintMs > 200UL) {
    lastBtPrintMs = now;
    btPrint("e=" + String(error) + " s=" + String(steer, 1) +
            " L=" + String(leftSpd) + " R=" + String(rightSpd));
  }

  delay(10);
}
