/*
    Name:       260523_2Wheel_Robot.ino
    Created:	5/23/2026 2:29:17 PM
    Author:     FRANK_XPS_9530\Frank
*/

#include "MPU6050_6Axis_MotionApps20.h"
//#include <FXUtil.h>
#include "FXUtil.h"
extern "C"
{
#include "FlashTxx.h"
//#include <FlashTxx.h> //06/01/26 mvd to Arduino/Libraries
}

MPU6050 mpu;

bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float gl_IMUHdgDeg = 0.0;
float headingOffset = 0.0;

const uint16_t MAX_GETPACKET_LOOPS = 100;

#define DMP_STARTUP_STABILITY_THRESHOLD 0.1f

// ====================== YOUR EXACT PIN NAMES ======================
const int LEFT_IN1 = 10;
const int LEFT_IN2 = 9;
const int RIGHT_IN1 = 12;
const int RIGHT_IN2 = 11;

// ====================== MOTOR PARAMETERS ======================
#define MOTOR_BIAS 5.0f
#define MOTOR_DEADBAND 80.0f
#define TELEMETRY_INTERVAL_MSEC 100

const int MOTOR_SPEED_FULL = 200;
const int MOTOR_SPEED_MAX = 255;
const int MOTOR_SPEED_HALF = 127;
const int MOTOR_SPEED_QTR = 75;
const int MOTOR_SPEED_LOW = 50;
const int MOTOR_SPEED_OFF = 0;
const int TURN_START_SPEED = MOTOR_SPEED_QTR;

// drive wheel direction constants
const bool TURNDIR_CCW = true;
const bool TURNDIR_CW = false;

// Motor direction variable
bool bIsForwardDir = true;

// ====================== SPIN TURN PARAMETERS ======================
const int TURN_RATE_UPDATE_INTERVAL_MSEC = 30;

// PID gains - tuned for 2-wheel robot (your final 10° values 2026-05-23)
#define TURN_RATE_BASE_KP     1.0
#define TURN_RATE_BASE_KI     0.2
#define TURN_RATE_BASE_KD     0.8
#define SMALL_TURN_KP_MULT    7.0
#define SMALL_TURN_KI_MULT    2.5

double TurnRate_Kp = TURN_RATE_BASE_KP;
double TurnRate_Ki = TURN_RATE_BASE_KI;
double TurnRate_Kd = TURN_RATE_BASE_KD;

const float HDG_FULL_MATCH_VAL = 0.99;
const float HDG_MIN_MATCH_VAL = 0.6;

double Prev_HdgDeg = 0.0;
double TurnRatePIDOutput;

// ====================== SERIAL PORT SUPPORT ======================
#define active_serial getActiveStream()
Stream& getActiveStream()
{
  if (Serial)
    return Serial;
  return Serial2;
}

// ====================== DMP HELPERS ======================
bool UpdateIMUHdgValDeg()
{
  uint8_t flag = GetCurrentFIFOPacket(fifoBuffer, packetSize, MAX_GETPACKET_LOOPS);

  if (flag != 0)
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    gl_IMUHdgDeg = -(ypr[0] * 180.0f / M_PI) - headingOffset;
    return true;
  }
  return false;
}

uint8_t GetCurrentFIFOPacket(uint8_t* data, uint8_t length, uint16_t max_loops)
{
  mpu.resetFIFO();
  delay(1);

  uint16_t fifoCount = mpu.getFIFOCount();
  uint16_t GetPacketLoopCount = 0;

  while (fifoCount < packetSize && GetPacketLoopCount < max_loops)
  {
    GetPacketLoopCount++;
    fifoCount = mpu.getFIFOCount();
    delay(2);
  }

  if (GetPacketLoopCount >= max_loops)
    return 0;

  mpu.getFIFOBytes(data, packetSize);
  return 1;
}

// ====================== MOTOR CONTROL ======================
void StopBothMotors()
{
  active_serial.printf("In StopBothMotors() - clean coast stop (no whine)\n");

  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void SetLeftMotorDirAndSpeed(bool bIsFwd, int speed)
{
#ifndef NO_MOTORS
  if (bIsFwd)
  {
    speed -= MOTOR_BIAS;
    speed = constrain(speed, MOTOR_SPEED_LOW, MOTOR_SPEED_MAX);
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, speed);
  }
  else
  {
    speed -= MOTOR_BIAS;
    speed = constrain(speed, MOTOR_SPEED_LOW, MOTOR_SPEED_MAX);
    analogWrite(LEFT_IN1, speed);
    analogWrite(LEFT_IN2, 0);
  }
#endif
}

void SetRightMotorDirAndSpeed(bool bIsFwd, int speed)
{
#ifndef NO_MOTORS
  speed += MOTOR_BIAS;
  speed = constrain(speed, MOTOR_SPEED_LOW, MOTOR_SPEED_MAX);

  if (bIsFwd)
  {
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, speed);
  }
  else
  {
    analogWrite(RIGHT_IN1, speed);
    analogWrite(RIGHT_IN2, 0);
  }
#endif
}

void MoveReverse(int leftspeednum, int rightspeednum)
{
  if (leftspeednum == 0 && rightspeednum == 0)
  {
    StopBothMotors();
  }
  else
  {
    SetLeftMotorDirAndSpeed(false, leftspeednum);
    SetRightMotorDirAndSpeed(false, rightspeednum);
  }
}

void MoveAhead(int leftspeednum, int rightspeednum)
{
  if (leftspeednum == 0 && rightspeednum == 0)
  {
    StopBothMotors();
  }
  else
  {
    SetLeftMotorDirAndSpeed(true, leftspeednum);
    SetRightMotorDirAndSpeed(true, rightspeednum);
  }
}

void RunBothMotorsBidirectional(int leftspeed, int rightspeed)
{
  if (leftspeed < 0)
    SetLeftMotorDirAndSpeed(false, -leftspeed);
  else
    SetLeftMotorDirAndSpeed(true, leftspeed);

  if (rightspeed < 0)
    SetRightMotorDirAndSpeed(false, -rightspeed);
  else
    SetRightMotorDirAndSpeed(true, rightspeed);
}

// ====================== SPIN TURN ======================
bool SpinTurn(bool b_ccw, float numDeg, float degPersec)
{
  active_serial.printf("In SpinTurn(%s, %2.2f, %2.2f)\n", b_ccw == TURNDIR_CCW ? "CCW" : "CW", numDeg, degPersec);
  active_serial.printf("TurnRatePID started with Kp/Ki/Kd = %2.1f,%2.1f,%2.1f, SampleTime(mSec) = %d\n",
    TurnRate_Kp, TurnRate_Ki, TurnRate_Kd, TURN_RATE_UPDATE_INTERVAL_MSEC);

  if (!dmpReady)
  {
    active_serial.printf("DMP Failure - returning FALSE\n");
    return false;
  }

  UpdateIMUHdgValDeg();

  int retries = 0;
  while ((gl_IMUHdgDeg == 180.0f || gl_IMUHdgDeg == 0.0f) && retries < 5)
  {
    active_serial.printf("Got 180.00 or 0.00 exactly (%2.3f) from IMU - retrying %d...\n", gl_IMUHdgDeg, retries);
    UpdateIMUHdgValDeg();
    retries++;
    delay(100);
  }

  float timeout_sec = 2 * numDeg / degPersec;
  timeout_sec = (timeout_sec < 1) ? 20.0f : timeout_sec;

#ifdef MPU6050_CCW_INCREASES_YAWVAL
  float tgt_deg = b_ccw ? gl_IMUHdgDeg + numDeg : gl_IMUHdgDeg - numDeg;
#else
  float tgt_deg = b_ccw ? gl_IMUHdgDeg - numDeg : gl_IMUHdgDeg + numDeg;
#endif

  if (tgt_deg < -180) tgt_deg += 360;
  if (tgt_deg > 180)  tgt_deg -= 360;

  active_serial.printf("Init hdg = %4.2f deg, Turn = %4.2f deg, tgt = %4.2f deg, timeout = %4.2f sec\n\n",
    gl_IMUHdgDeg, numDeg, tgt_deg, timeout_sec);

  float curHdgMatchVal = 0.0f;
  float prevHdgMatchVal = 0.0f;
  float matchSlope = 0.0f;

  Prev_HdgDeg = gl_IMUHdgDeg;

  int startSpd = (degPersec <= 45) ? TURN_START_SPEED : 2 * TURN_START_SPEED;

  SetLeftMotorDirAndSpeed(!b_ccw, startSpd);
  SetRightMotorDirAndSpeed(b_ccw, startSpd);

  elapsedMillis sinceLastTimeCheck = 0;
  elapsedMillis sinceLastComputeTime = 0;

  double lastError = 0;
  double lastInput = 0;
  double lastIval = 0;
  double lastDerror = 0;

  float loop_Kp = (numDeg >= 45) ? TurnRate_Kp : SMALL_TURN_KP_MULT * TurnRate_Kp;
  float loop_Ki = (numDeg >= 45) ? TurnRate_Ki : SMALL_TURN_KI_MULT * TurnRate_Ki;
  float loop_Kd = TurnRate_Kd;
  active_serial.printf("loop_Kp/Ki/Kd = %2.1f,%2.1f,%2.1f\n", loop_Kp, loop_Ki, loop_Kd);

  active_serial.printf("Msec\tHdg\tPrvHdg\tdHdg\tRate\ttgtDPS\terr\tKp*err\tIval\tKd*Derr\tspeed\tMatch\tSlope\n");

  bool bDoneTurning = false;
  bool bTimedOut = false;
  bool bResult = true;

  while (!bDoneTurning && !bTimedOut)
  {
    if (sinceLastComputeTime >= TURN_RATE_UPDATE_INTERVAL_MSEC)
    {
      sinceLastComputeTime -= TURN_RATE_UPDATE_INTERVAL_MSEC;

      UpdateIMUHdgValDeg();

      double dHdg = gl_IMUHdgDeg - Prev_HdgDeg;
      if (dHdg > 180) dHdg -= 360;
      else if (dHdg < -180) dHdg += 360;

      double rate = fabs(1000 * dHdg / TURN_RATE_UPDATE_INTERVAL_MSEC);

      TurnRatePIDOutput = PIDCalcs(rate, degPersec, TURN_RATE_UPDATE_INTERVAL_MSEC,
        lastError, lastInput, lastIval, lastDerror,
        loop_Kp, loop_Ki, loop_Kd);

      int speed = (int)TurnRatePIDOutput;
      if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
      if (speed < MOTOR_SPEED_LOW) speed = MOTOR_SPEED_LOW;

      SetLeftMotorDirAndSpeed(!b_ccw, speed);
      SetRightMotorDirAndSpeed(b_ccw, speed);

      curHdgMatchVal = GetHdgMatchVal(tgt_deg, gl_IMUHdgDeg);
      matchSlope = curHdgMatchVal - prevHdgMatchVal;

      active_serial.printf("%lu\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%d\t%2.2f\t%2.2f\n",
        millis(), gl_IMUHdgDeg, Prev_HdgDeg, dHdg, rate, degPersec,
        lastError, loop_Kp * lastError, lastIval, loop_Kd * lastDerror,
        speed, curHdgMatchVal, matchSlope);

      Prev_HdgDeg = gl_IMUHdgDeg;

      bDoneTurning = (curHdgMatchVal >= HDG_FULL_MATCH_VAL ||
        (prevHdgMatchVal >= HDG_MIN_MATCH_VAL && matchSlope <= -0.01));

      prevHdgMatchVal = curHdgMatchVal;

      bTimedOut = (sinceLastTimeCheck > timeout_sec * 1000);

      if (bTimedOut)
      {
        active_serial.printf("timed out with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", gl_IMUHdgDeg, tgt_deg, curHdgMatchVal);
        bResult = false;
        break;
      }
      if (bDoneTurning)
      {
        active_serial.printf("Completed turn with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", gl_IMUHdgDeg, tgt_deg, curHdgMatchVal);
        bResult = true;
        break;
      }
    }
  }

  StopBothMotors();
  delay(1000);
  return bResult;
}

double PIDCalcs(double input, double setpoint, uint16_t sampleTime,
  double& lastError, double& lastInput, double& lastIval, double& lastDerror,
  double Kp, double Ki, double Kd)
{
  double error = setpoint - input;
  lastIval += (Ki * error);
  lastIval = constrain(lastIval, MOTOR_SPEED_OFF, MOTOR_SPEED_MAX);

  double dErr = error - lastError;
  double output = Kp * error + lastIval - Kd * dErr;

  lastError = error;
  lastInput = input;
  lastDerror = dErr;

  return output;
}

float GetHdgMatchVal(float tgt_deg, float cur_deg)
{
  float tgthdg = (tgt_deg < 0) ? tgt_deg + 360 : tgt_deg;
  float curhdg = (cur_deg < 0) ? cur_deg + 360 : cur_deg;
  return fabs(1.0f - fabs(tgthdg - curhdg) / 180.0f);
}

// ====================== COMMAND HANDLER ======================
void CheckForUserInput()
{
  const int bufflen = 3;
  char buff[bufflen];
  byte incomingByte = 0;
  int numchars = 0;

  if (active_serial.available() > 0)
  {
    numchars = active_serial.readBytesUntil('\n', buff, sizeof(buff));
    incomingByte = buff[0];
    active_serial.printf("I received %d chars, first char %c\n", numchars, incomingByte);
  }

  if (incomingByte != 0)
  {
    switch (incomingByte)
    {
    case 'U':
    case 'u':
      active_serial.println(F("Start Program Update - Send new HEX file!"));
      delay(500);

      uint32_t buffer_addr, buffer_size;
      if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
      {
        active_serial.println("Failed to init buffer");
        return;
      }

      while (active_serial.available()) active_serial.read();

      update_firmware(&Serial2, &Serial2, buffer_addr, buffer_size);
      firmware_buffer_free(buffer_addr, buffer_size);
      REBOOT;
      break;

    case 'C':
    case 'c':
      active_serial.println(F("ENTERING COMMAND MODE:"));
      active_serial.println(F("0 = 180 deg CCW Turn"));
      active_serial.println(F("1 = 180 deg CW Turn"));
      active_serial.println(F("A = Back to Auto Mode"));
      active_serial.println(F("S = Stop"));
      active_serial.println(F("F = Forward"));
      active_serial.println(F("R = Reverse"));
      active_serial.println(F(""));
      active_serial.println(F("       Faster"));
      active_serial.println(F("\t8"));
      active_serial.println(F("Left 4\t5  6 Right"));
      active_serial.println(F("\t2"));
      active_serial.println(F("       Slower"));

      StopBothMotors();
      int speed = 0;
      bool bAutoMode = false;

      while (active_serial.available()) active_serial.read();

      while (!bAutoMode)
      {
        if (active_serial.available() > 0)
        {
          numchars = active_serial.readBytesUntil('\n', buff, sizeof(buff));
          incomingByte = buff[0];
          while (active_serial.available()) active_serial.read();
        }

        if (incomingByte != 0)
        {
          switch (incomingByte)
          {
          case 'U':
          case 'u':
            active_serial.println(F("Start Program Update - Send new HEX file!"));
            delay(500);

            uint32_t buffer_addr2, buffer_size2;
            if (firmware_buffer_init(&buffer_addr2, &buffer_size2) == 0)
            {
              active_serial.println("Failed to init buffer");
              return;
            }

            while (active_serial.available()) active_serial.read();

            update_firmware(&Serial2, &Serial2, buffer_addr2, buffer_size2);
            firmware_buffer_free(buffer_addr2, buffer_size2);
            REBOOT;
            break;

          case '0':
            active_serial.println(F("CCW 180 deg Turn"));
            SpinTurn(true, 180, 90);
            MoveAhead(speed, speed);
            break;
          case '1':
            active_serial.println(F("CW 180 deg Turn"));
            SpinTurn(false, 180, 90);
            break;
          case '4':
            active_serial.println(F("CCW 10 deg Turn"));
            SpinTurn(true, 10, 30);
            if (bIsForwardDir) MoveAhead(speed, speed); else MoveReverse(speed, speed);
            break;
          case '6':
            active_serial.print("CW 10 deg Turn\n");
            SpinTurn(false, 10, 30);
            if (bIsForwardDir) MoveAhead(speed, speed); else MoveReverse(speed, speed);
            break;
          case '8':
            speed += 50;
            speed = (speed >= MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : speed;
            active_serial.printf("Speeding up: speed now %d\n", speed);
            if (bIsForwardDir) MoveAhead(speed, speed); else MoveReverse(speed, speed);
            break;
          case '2':
            speed -= 50;
            speed = (speed < 0) ? 0 : speed;
            active_serial.printf("Slowing down: speed now %d\n", speed);
            if (bIsForwardDir) MoveAhead(speed, speed); else MoveReverse(speed, speed);
            break;
          case '5':
            active_serial.println(F("Stopping Motors!"));
            StopBothMotors();
            speed = 0;
            break;
          case 'A':
          case 'a':
            StopBothMotors();
            active_serial.println(F("Re-entering AUTO mode"));
            bAutoMode = true;
            break;
          case 'R':
          case 'r':
            active_serial.println(F("Setting both motors to reverse"));
            bIsForwardDir = false;
            MoveReverse(speed, speed);
            break;
          case 'F':
          case 'f':
            active_serial.println(F("Setting both motors to forward"));
            bIsForwardDir = true;
            MoveAhead(speed, speed);
            break;
          default:
            active_serial.println(F("In Default Case: Stopping Motors!"));
            StopBothMotors();
          }
          incomingByte = 0;
        }
      }
      break;
    }
  }
}

// ====================== SETUP / LOOP ======================
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(2000);

  active_serial.println("MPU6050 DMP6 + Motor Turn Test - Teensy 4.1 (cleaned 2026-05-23 20:45)");
  active_serial.println("Type e.g. C then 0/1/4/6/8/2/5/A/F/R/U in Serial Monitor");

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection())
  {
    active_serial.println("MPU6050 connection failed!");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-125);
  mpu.setYAccelOffset(1798);
  mpu.setZAccelOffset(1545);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(22);
  mpu.setZGyroOffset(70);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    active_serial.println("DMP initialized - Waiting for heading to stabilize");
  }
  else
  {
    active_serial.print("DMP Initialization failed (code ");
    active_serial.print(devStatus);
    active_serial.println(")");
    while (1);
  }

  delay(1000);

  const int STABLE_READINGS_REQUIRED = 5;
  int stableCount = 0;
  float Prev_HdgDeg_local;
  UpdateIMUHdgValDeg();
  Prev_HdgDeg_local = gl_IMUHdgDeg;

  while (stableCount < STABLE_READINGS_REQUIRED)
  {
    active_serial.print(".");
    if (abs(gl_IMUHdgDeg - Prev_HdgDeg_local) <= DMP_STARTUP_STABILITY_THRESHOLD)
      stableCount++;
    else
      stableCount = 0;

    Prev_HdgDeg_local = gl_IMUHdgDeg;
    delay(100);
    UpdateIMUHdgValDeg();
  }

  active_serial.println(".");
  active_serial.println("Zeroing heading to current stable orientation at startup");
  headingOffset = gl_IMUHdgDeg;
  gl_IMUHdgDeg = 0.0;

  float StartSec = millis() / 1000.0f;
  active_serial.printf("MPU6050 Ready at %2.2f Sec\n", StartSec);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  UpdateIMUHdgValDeg();
  CheckForUserInput();
}