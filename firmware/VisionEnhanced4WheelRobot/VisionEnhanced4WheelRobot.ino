//#include "MPU6050.h"
//MPU6050 mpu;

//08/04/26 copied MPU6050 DMP6 stuff from WallE3_Git
#include <Wire.h>
//#include "FlashTxx.h"		// TLC/T3x/T4x flash primitives

extern "C"
{
#include "FlashTxx.h"
}

#include "FXUtil.h"

#include <elapsedMillis.h>
#include "MPU6050_6Axis_MotionApps612.h"  //01/18/22 changed to use the \I2CDevLib\Arduino\MPU6050\ version
#include "I2C_Anything.h" //needed for sending float data over I2C
#include "timelib.h" //added 01/01/22 for charge monitoring support
#include <math.h>
//#include "enums.h" //'local' header file for navigation 'state' enums

#pragma region DEFINES
//02/29/16 hardware defines
//#define HDG_ONLY //added 06/11/23
//#define NO_MOTORS
//#define NO_MPU6050 //added 01/23/22
//#define IR_HOMING_ONLY
//#define NO_FRONT_LIDAR
//#define NO_VL53L0X //01/08/22 now used for VL53L0X hardware
//#define NO_IRDET //added 04/05/17 for daytime in-atrium testing (too much ambient IR)
//#define DISTANCES_ONLY //added 11/14/18 to just display distances in infinite loop
//#define NO_STUCK //added 03/10/19 to disable 'stuck' detection
//#define BATTERY_DISCHARGE //added 03/04/20 to discharge battery safely
//#define PID_TUNING_TELEMETRY_ONLY //added 07/22/23 to facilitate PID tuning
//#define CHG_STATION_PID_TUNING_TEST //added 11/16/23 to document this testing block & then commented out -
//#define FRONTBACKMOTION_TESTING_ONLY //added 11/16/23

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) //added 01/26/23 to extract pgm filename from full path
#pragma endregion DEFINES

//11/07/2020 moved all I2C Address declarations here
#pragma region I2C_ADDRESSES
#define IRDET_I2C_ADDR 0x08
#define MPU6050_I2C_ADDR 0x68
//const uint16_t VL53L0X_I2C_SLAVE_ADDRESS = 0x20; ////Teensy 3.5 VL53L0X ToF LIDAR controller
#define VL53L0X_I2C_SLAVE_ADDRESS 0x20 //Teensy 3.5 VL53L0X ToF LIDAR controller - chg to #define 02/01/23
//uint8_t IR_HOMING_MODULE_SLAVE_ADDR = 8;  //uint8_t type reqd here for Wire.requestFrom() call
#define IR_HOMING_MODULE_SLAVE_ADDR 8  //uint8_t type reqd here for Wire.requestFrom() call - chg to #define 02/01/23
#define GARMIN_LIDAR_I2C_ADDR 0x62  //added 02/25/23 for Garmin LIDAR-Lite V4/LED on main Teensy Wire2
#pragma endregion I2C_ADDRESSES

//#pragma region PRE_SETUP
MPU6050 mpu(MPU6050_I2C_ADDR);
uint32_t buffer_addr, buffer_size;



#pragma region TIME INTERVALS
const uint16_t MSEC_PER_IR_HOMING_ADJ = 150; //06/27/22 100ms is too fast for GetFrontDistCm()
//const int MSEC_PER_DIST_UPDATE = 100; //10/02/22 rev to speed up rate indep of front LIDAR
const int MSEC_PER_DIST_UPDATE = 50; //10/02/22 rev to speed up rate indep of front LIDAR
const uint16_t WALL_TRACK_UPDATE_INTERVAL_MSEC = MSEC_PER_DIST_UPDATE;//10/02/22 rev to make these two intervals the same
//const uint16_t FRONT_DISTANCE_UPDATE_INTERVAL_MSEC = 250; //10/02/22 added to separate out slow front LIDAR from faster VL53L0X sensors
const uint16_t FRONT_DISTANCE_UPDATE_INTERVAL_MSEC = MSEC_PER_DIST_UPDATE; //03/15/23 Garmin LIDAR can easily keep up with other sensors
const uint16_t TURN_RATE_UPDATE_INTERVAL_MSEC = 30; //30 mSec is as fast as it can go
const uint16_t PRINT_INTERVAL_MSEC = 100; //added 09/25/23

elapsedMillis MsecSinceLastAdj; //added 05/24/22 for ParallelOrientation() routine
elapsedMillis MsecSinceLastIRHomingAdj; //01/07/22 used for #ifdef IR_HOMING_ONLY block
elapsedMillis MsecSinceLastDistUpdate; //01/07/22 used for local dist update loops
elapsedMillis MsecSinceLastTurnRateUpdate;//heading/rate based turn support
elapsedMillis mSecSinceLastWallTrackUpdate;
elapsedMillis MsecSinceLastFrontDistUpdate; //10/02/22 slower update rate needed for front LIDAR
elapsedMillis mSecSinceLastTelemetryUpdate; //09/25/23 added to reduce telemetry printout frequency
#pragma endregion TIME INTERVALS

#pragma region ADC CONSTANTS
const float ADC_REF_VOLTS = 3.3; //teensy default for analog inputs
const int MAX_AD_COUNT = 1023;
const float AmpsPerVolt = 1.00; //default 10K Rs
const float VoltsPerCount = ADC_REF_VOLTS / MAX_AD_COUNT;
const float VOLTAGE_TO_CURRENT_RATIO = 1.f; //Used for both 'Total' and 'Run' sensors
#pragma endregion ADC CONSTANTS

#pragma region TELEMETRYSTRINGS
const char* IRHomingTelemStr = "Time\tBattV\tFin1\tFin2\tSteer\tPID_Out\t\tLSpd\tRSpd\tFrontD\tRearD";
const char* IRHomingTelemStrNoPings = "Time\tBattV\tFin1\tFin2\tSteer\tPID_Out\t\tLSpd\tRSpd\n";

//07/17/23 split WallFollowTelemHdrStr & WallFollowTelemStr into Left/Right versions
const char* WallFollowTelemStr = "%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.2f\t%2.2f\t%2.0f\t%2.1f\t%d\t%d\t%s\t%s\n";//05/12/23 added L/R motor speeds
const char* LeftWallFollowTelemHdrStr = "\nSec\tLCen\tRCen\tDeg\tLF\tLR\tLStr\tFront\tRear\tFVar\tRVar\tLSpd\tRSpd\tACODE\tTRKDIR\n";//07/17/23 added Front/Rear dists
const char* RightWallFollowTelemHdrStr = "\nSec\t\tLCen\tRCen\t\tDeg\tRF\tRR\tRStV\tFront\tRear\tFVar\tRVar\tLSpd\tRSpd\tACODE\tTRKDIR\n";//10/13/24 chg 'Rstr' to 'RstV', added '\t's
const char* LeftWallFollowTelemStr = "%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.2f\t%d\t%2.0f\t%2.0f\t%2.0f\t%d\t%d\t%s\t%s\n";//07/17/23 added Front/Rear dists, rem Rt steerval
const char* RightWallFollowTelemStr = "%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.2f\t%d\t%2.0f\t%2.0f\t%2.0f\t%d\t%d\t%s\t%s\n";//07/17/23 added Front/Rear dists, rem Rt steerval
const char* ChargingTelemStr = "ChgSec\tBattV\tTotalI\tRunI\tChgI\tbChging\n"; //rev 01/30/21

const char* PIDTuneHdrStr = "\nSec\tCen\tF\tR\tSteer\tcorrD\tadjF\tkP\tkI\tkD\tOut\tLSpd\tRSpd\n";
const char* PIDTuneTelemStr = "%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%d\t%d\n";//07/22/23 Wall track PID tuning telemetry.

#pragma endregion Mode-specific telemetry header strings

#pragma region BATTCONSTS
//03/10/15 added for battery charge level monitoring
//const float LOW_BATT_THRESH_VOLTS = 7.4; //50% chg per http://batteryuniversity.com/learn/article/lithium_based_batteries
//const float LOW_BATT_THRESH_VOLTS = 6.4; //05/02/22 for debug
//const float LOW_BATT_THRESH_VOLTS = 7.0; 
//const float LOW_BATT_THRESH_VOLTS = 7.8; //05/07/22 for debug
const float LOW_BATT_THRESH_VOLTS = 8.2; //11/03/23 for debug
//const long BATT_CHG_TIMEOUT_SEC = 36000; //10 HRS
const uint16_t BATT_CHG_TIMEOUT_SEC = 12; //12/28/20 for test only 
const float DEAD_BATT_THRESH_VOLTS = 6; //added 01/24/17
//const float DEAD_BATT_THRESH_VOLTS = 8.4; //03/21/22 temp number for testing.  added 01/24/17
const float FULL_BATT_VOLTS = 8.4; //added 03/17/18. Chg to 8.4 03/05/20
const uint16_t MAX_DEAD_BATT_DETS = 3; //added 06/12/23 to suppress spurious dead battery detections
const uint16_t MINIMUM_CHARGE_TIME_SEC = 10; //added 04/01/18
const float FULL_BATT_CURRENT_THRESHOLD = 0.5; //amps chg to 0.5A 03/02/19 per https://www.fpaynter.com/2019/03/better-battery-charging-for-wall-e2/
const uint16_t IR_HOMING_TELEMETRY_SPACING_MSEC = 200; //added 04/23/20

//battery fuel guage constants
const float _20PCT_BATT_VOLTS = DEAD_BATT_THRESH_VOLTS + 0.2f * (FULL_BATT_VOLTS - DEAD_BATT_THRESH_VOLTS);//6.48V
const float _40PCT_BATT_VOLTS = DEAD_BATT_THRESH_VOLTS + 0.4f * (FULL_BATT_VOLTS - DEAD_BATT_THRESH_VOLTS);//6.96V
const float _60PCT_BATT_VOLTS = DEAD_BATT_THRESH_VOLTS + 0.6f * (FULL_BATT_VOLTS - DEAD_BATT_THRESH_VOLTS);//7.44V
const float _80PCT_BATT_VOLTS = DEAD_BATT_THRESH_VOLTS + 0.8f * (FULL_BATT_VOLTS - DEAD_BATT_THRESH_VOLTS);//7.92V
const float _90PCT_BATT_VOLTS = DEAD_BATT_THRESH_VOLTS + 0.9f * (FULL_BATT_VOLTS - DEAD_BATT_THRESH_VOLTS);//8.16V
const float ZENER_VOLTAGE_OFFSET = 5.96; //03/14/22 measured zener voltage
#pragma endregion Battery Constants


#pragma region PIN ASSIGNMENTS
#pragma region MOTOR PINS
//11/04/21 Now using Pololu VNH5019 drivers for all 4 motors
const uint16_t InA_Left = 22;
const uint16_t InB_Left = 21;
const uint16_t Spd_Left = 23;

//08/05/26 pin 35 isn't PWMable on Teensy 4.1, but pin 33 is
//const uint16_t InA_Right = 34;
//const uint16_t InB_Right = 33;
//const uint16_t Spd_Right = 35;
const uint16_t InA_Right = 34;
const uint16_t InB_Right = 35;
const uint16_t Spd_Right = 33;
#pragma endregion MOTOR PINS

#pragma region CHG SUPP PINS
//12/19/21 updated here, schematic, and spreadsheet
const uint16_t CHG_CONNECT_PIN = A0; //output of photo resistor looking at TP5100 CHG LED
const uint16_t BATT_MON_PIN = A1; //connected to 5VLDO regulator module
const uint16_t TOT_CURR_PIN = A2; //connected to 1NA619 between charge plug and battery
const uint16_t RUN_CURR_PIN = A3; //connected to 1NA619 between battery and rest of robot

//state-of-charge indicator LEDs
const uint16_t CHG_FIN_LED_PIN = 32; //purple
const uint16_t _80PCT_LED_PIN = 31; //blue
const uint16_t _60PCT_LED_PIN = 30; //green
const uint16_t _40PCT_LED_PIN = 29; //orange
const uint16_t _20PCT_LED_PIN = 28; //red
const uint16_t CHG_CONNECT_LED_PIN = 27;//brown
#pragma endregion CHG SUPP PINS

#pragma region MISCELLANEOUS_PINS
//Second Deck Pins
const uint16_t RED_LASER_DIODE_PIN = 5;//Laser pointer

//02/23/23 replace Pulsed Light LIDAR with Garmin LIDAR-Lite V4/LED @ 0x62
//const uint16_t LIDAR_MODE_PIN = 3; //LIDAR MODE pin (continuous mode)
//const uint16_t VL53L0X_TEENSY_RESET_PIN = 4; //pulled low for 1 mSec in Setup()
const uint16_t LIDAR_SCL_PIN = 3;//SCL2 Purple
const uint16_t LIDAR_SDA_PIN = 4;//SDA2 Orange
const uint16_t VL53L0X_TEENSY_RESET_PIN = 26; //pulled low for 1 mSec in Setup() Gray

//Miscellaneous pins
const uint16_t HOMING_PID_COMPUTE_CALL_PIN = CHG_CONNECT_LED_PIN; //using same LED for two indications
const uint16_t SOS_PWM_PIN = 2;//03/19/22 corection: connected to speaker for dead-battery SOS

//03/08/22 added for misc duration measurements
const uint16_t DURATION_MEASUREMENT_PIN1 = 6;
const uint16_t DURATION_MEASUREMENT_PIN2 = 7;
#pragma endregion MISCELLANEOUS_PINS
#pragma endregion PIN ASSIGNMENTS

#pragma region CHG SUPP PARAMETERS
//12/25/21 now using analog input, so threshold scaled by analog ref voltage
const uint16_t CHG_CONNECTED_AVG_THRESHOLD = 0.2 * MAX_AD_COUNT; //integer truncation OK
const uint16_t CHG_DISCONNECTED_AVG_THRESHOLD = 0.8 * MAX_AD_COUNT;//integer truncation OK
unsigned long chgStartMsec;//added 02/24/17
float TotalAmps; //moved here from loop() 01/02/22
float RunAmps; //moved here from loop() 01/02/22
#pragma endregion CHG SUPP PARAMETERS		

#pragma region MOTOR_PARAMETERS
//drive wheel speed parameters
const int MOTOR_SPEED_FULL = 200; //range is 0-255
const int MOTOR_SPEED_MAX = 255; //range is 0-255
const int MOTOR_SPEED_HALF = 127; //range is 0-255
const int MOTOR_SPEED_QTR = 75; //added 09/25/20
const int MOTOR_SPEED_LOW = 50; //added 01/22/15
const int MOTOR_SPEED_OFF = 0; //range is 0-255
const int MOTOR_SPEED_CAPTURE_OFFSET = 75; //added 06/21/20 for offset capture
const int TURN_START_SPEED = MOTOR_SPEED_QTR; //added 11/14/21

//drive wheel direction constants
const boolean FWD_DIR = true;
const boolean REV_DIR = !FWD_DIR;
const bool TURNDIR_CCW = true;
const bool TURNDIR_CW = false;

//Motor direction variables
bool gl_bIsForwardDir = true; //default is foward direction

#pragma endregion Motor Parameters

#pragma region MPU6050_SUPPORT
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU. Used in Homer's Overflow routine
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int GetPacketLoopCount = 0;
int OuterGetPacketLoopCount = 0;

//MPU6050 status flags
bool bMPU6050Ready = true;
bool dmpReady = false;  // set true if DMP init was successful
volatile float IMUHdgValDeg = 0; //updated by UpdateIMUHdgValDeg()//11/02/20 now updated in ISR
const uint16_t MAX_GETPACKET_LOOPS = 100; //10/30/19 added for backup loop exit condition in GetCurrentFIFOPacket()
uint8_t GetCurrentFIFOPacket(uint8_t* data, uint8_t length, uint16_t max_loops = MAX_GETPACKET_LOOPS); //prototype here so can define a default param
bool bFirstTime = true;
//#define MPU6050_CCW_INCREASES_YAWVAL //added 12/05/19 commented out 11/22/21 as now the MPU6050 module is mounted 'Z-up'
#pragma endregion MPU6050 Support

#pragma region HEADING_AND_RATE_BASED_TURN_PARAMETERS
float Prev_HdgDeg = 0; //02/01/23 - this should be a local variable in SpinTurn()
float TurnRatePIDOutput; //02/01/23 - this should be a local variable in SpinTurn()

//06/04/22 from WallE3_SpinTurnTuning.ino
float TurnRate_Kp = 0.7;//02/03/23 updated Kp from 1.0 to 0.7 per https://www.fpaynter.com/2022/12/walle3-spin-turn-revisited/
float TurnRate_Ki = 0.3;
float TurnRate_Kd = 0.0;//02/03/23 updated Kd from 0.1 to 0.0 per https://www.fpaynter.com/2022/12/walle3-spin-turn-revisited/

//ported from FourWD_PulseTurnRateTest.ino

const float HDG_NEAR_MATCH_VAL = 0.8; //slow the turn down here
const float HDG_FULL_MATCH_VAL = 0.99; //stop the turn here //rev 06/01/21
const float HDG_MIN_MATCH_VAL = 0.6; //added 09/08/18: don't start checking slope until turn is well started
const float DEFAULT_TURN_RATE_DEGPERSEC = 45.f; //06/06/21 updated
const uint16_t HEADING_HISTORY_ARRAY_SIZE = 50; //06/12/23 added for 'spinning' condx detection
float gl_HdgHistoryArray[HEADING_HISTORY_ARRAY_SIZE];

//02/16/22 fwd decl reqd for fcns using default param
//09/16/23 added RunBothMotorsMsec with default params
bool SpinTurn(bool b_ccw, float numDeg, float degPersec = DEFAULT_TURN_RATE_DEGPERSEC);

//06/01/24 chg int gl_Leftspeednum to uint16_t leftspeednum, int gl_Rightspeednum to uint16_t rightspeednum
//void RunBothMotorsMsec(bool bisFwd, int timeMsec = 500, int gl_Leftspeednum = MOTOR_SPEED_HALF, int gl_Rightspeednum = MOTOR_SPEED_HALF);
void RunBothMotorsMsec(bool bisFwd, int timeMsec = 500, uint16_t leftspeednum = MOTOR_SPEED_HALF, uint16_t rightspeednum = MOTOR_SPEED_HALF);
bool RollingTurn(bool b_ccw, bool b_fwd, float numDeg, float Kp, float Ki, float Kd, float degPersec = DEFAULT_TURN_RATE_DEGPERSEC);
//bool IsChargerConnected(bool curState = false);

#pragma endregion HEADING_AND_RATE_BASED_TURN_PARAMETERS


#pragma region GLOBAL_VARIABLES
float gl_batteryVoltage;
uint16_t gl_Leftspeednum = MOTOR_SPEED_HALF;
uint16_t gl_Rightspeednum = MOTOR_SPEED_HALF;


int16_t gl_FinalLeftSpeed = 0;
int16_t gl_FinalRightSpeed = 0;

//02/15/22 added from FourWD_WallE2_V12.ino
uint16_t gl_FrontCm = 0;
float gl_Rearvar = 0;//chg to float 03/23/22
bool gl_bChgConnect = false;

//Sensor data values
//03/01/22 rev to store all dists in cm vs mm
float gl_RearCm; //added 10/24/20
Stream* gl_pSerPort = 0; //09/26/22 made global so can use everywhere.
float gl_ElapsedSec = 0; //03/18/23
elapsedMillis gl_ElapsedRunMillisec = 0; //03/18/23

uint16_t gl_NumDeadBattDets = 0; //added 06/12/23 to suppress spurious dead battery detections

//added 07/22/23 for PID tuning telemetry
float gl_PIDLastIval;
float gl_LeftOffsetFactor;
float gl_RightOffsetFactor;
float gl_PID_Kp_error;
float gl_PID_Kd_dErr;
float gl_PID_Outval;

//added 09/09/23 for 'choose better side' support
float gl_AvgHeadingDeg;

//added 10/08/23 for QuickSort()
const uint16_t QUICKSORT_ARRAY_SIZE = 36;
uint16_t FrontD[QUICKSORT_ARRAY_SIZE];
float Hdg[QUICKSORT_ARRAY_SIZE];
float RightD[QUICKSORT_ARRAY_SIZE];
float LeftD[QUICKSORT_ARRAY_SIZE];
float RightSteer[QUICKSORT_ARRAY_SIZE];
float LeftSteer[QUICKSORT_ARRAY_SIZE];
const uint16_t MIN_RUN_TO_DAYLIGHT_DIST_CM = 150;//added 10/14/23

unsigned long prevTime = 0;
float gl_IMUHdgValDeg = 0.0;
float gl_Prev_HdgDeg = 0.0;
bool gl_bMPU6050Ready = false;
#pragma endregion GLOBAL_VARIABLES

const float GYRO_Z_SENSITIVITY = 131.0;
const int STATIONARY_THRESHOLD = 120;
const unsigned long STATIONARY_TIME = 2000;

unsigned long stillStartTimeMsec = 0;
bool isStationary = false;

#define TURN_TERMINATION_ERROR_THRESHOLD 2.0

struct SimplePID
{
  float kp = 5.0;      // increased for much stronger response
  float ki = 0.008;
  float kd = 1.0;
  float integral = 0.0;
  float prevError = 0.0;
};

SimplePID pid;

//// ====================== SHARED HELPER ======================
// ====================== turnDegrees() Must be here due to default param def ======================
void turnDegrees(float turnDeg, float targetDegPerSec = 15.0)
{
  updateHeadingAndStationaryReset(); //updates gl_IMUHdgValDeg
  float startHeading = gl_IMUHdgValDeg;
  float targetHeading = startHeading + turnDeg;

  Serial.print("=== Starting turn to ");
  Serial.print(turnDeg, 1);
  Serial.print(" deg at ");
  Serial.print(targetDegPerSec, 1);
  Serial.println(" deg/s ===");

  Serial.printf("Sec\tDeg\tError\tTgtR\tActR\tPID_PWM\n");

  pid.integral = 0.0;
  pid.prevError = 0.0;

  unsigned long turnStartTime = millis();


  //fwd half-speed, 
  //digitalWrite(LEFT_IN1, HIGH);
  //analogWrite(LEFT_IN2, 128);
  //digitalWrite(RIGHT_IN1, HIGH);
  //analogWrite(RIGHT_IN2, 128);

  delay(1000);

  //digitalWrite(LEFT_IN1, LOW);
  //digitalWrite(LEFT_IN2, LOW);

  ////right motor runs backward slowly after stop
  //digitalWrite(RIGHT_IN1, LOW);
  //digitalWrite(RIGHT_IN2, LOW);

  ////right motor runs forward at half speed after stop
  //digitalWrite(RIGHT_IN1, HIGH);
  //digitalWrite(RIGHT_IN2, HIGH);


  while (true)
  {
    updateHeadingAndStationaryReset();

    float error = targetHeading - gl_IMUHdgValDeg;

    float dt = (micros() - prevTime) / 1000000.0f;
    pid.integral += error * dt;
    float derivative = (error - pid.prevError) / dt;
    float pidOutput = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    pid.prevError = error;

    // Drive motors with corrected direction
    //setMotor(LEFT_DIR_PIN, LEFT_PWM_PIN, -pidOutput);   // flipped
    //setMotor(RIGHT_DIR_PIN, RIGHT_PWM_PIN, pidOutput);   // flipped

    RunBothMotorsBidirectional(int(-pidOutput), int(pidOutput));

    static unsigned long lastTelemetry = 0;
    if (millis() - lastTelemetry >= 100)
    {
      Serial.printf("%lu\t%6.2f\t%6.2f\t%5.2f\t%5.2f\t%d\n",
        millis() / 1000, gl_IMUHdgValDeg, error, targetDegPerSec,
        (mpu.getRotationZ() / GYRO_Z_SENSITIVITY),
        (int)pidOutput);
      lastTelemetry = millis();
    }

    if (abs(error) < TURN_TERMINATION_ERROR_THRESHOLD)
    {
      Serial.println("=== Turn complete ===");
      break;
    }

    if (millis() - turnStartTime > 30000)
    {
      Serial.println("=== Turn timeout ===");
      break;
    }
  }

  // Stop motors
  //analogWrite(LEFT_PWM_PIN, 0);
  //analogWrite(RIGHT_PWM_PIN, 0);
  delay(500);
}

void setup()
{
  //Serial.begin(115200);
  //while (!Serial);
  //while (Serial.available()) Serial.read();

#pragma region PIN INITIALIZATION

  pinMode(LED_BUILTIN, OUTPUT);

#pragma region Second Deck Pins
  pinMode(RED_LASER_DIODE_PIN, OUTPUT);
  //pinMode(LIDAR_MODE_PIN, INPUT);//02/23/23 rem - now using Garmin LIDAR V4/LED at 0x62
#pragma endregion Second Deck Pins

#pragma region Charge_Support_Pins
  //current sensor pins
  pinMode(RUN_CURR_PIN, INPUT); //02/24/19 now connected to 'Run Current' 1NA619 charge current sensor
  digitalWrite(RUN_CURR_PIN, LOW); //turn off the internal pullup resistor

  pinMode(TOT_CURR_PIN, INPUT);//02/24/19 now connected to 'Total Current' 1NA619 charge current sensor
  digitalWrite(TOT_CURR_PIN, LOW); //turn off the internal pullup resistor

  //Battery Voltage Monitor pin
  pinMode(BATT_MON_PIN, INPUT);
  //digitalWrite(BATT_MON_PIN, LOW); //turn off the internal pullup resistor

  //charge connect
  pinMode(CHG_CONNECT_PIN, INPUT_PULLUP);  //goes LOW when chg cable connected
  digitalWrite(CHG_CONNECT_PIN, HIGH); //01/09/22 needed now that InitAllPins() forces it LOW

  //charge connect status display pin (this will eventually be one of the LEDs on the rear LED panel)
  pinMode(CHG_CONNECT_LED_PIN, OUTPUT);  //12/16/20 lights LED when chg cable connected
  pinMode(_60PCT_LED_PIN, OUTPUT);
  pinMode(CHG_FIN_LED_PIN, OUTPUT);
  pinMode(_80PCT_LED_PIN, OUTPUT);
  pinMode(_40PCT_LED_PIN, OUTPUT);
  pinMode(_20PCT_LED_PIN, OUTPUT);

  Serial.printf("In setup() just before EnableAllRearLEDs\n");

  EnableAllRearLEDs(false); //turns all LEDs OFF

#pragma endregion Charge_Support_Pins

#pragma region Motor_Pins
  //motor pins
  pinMode(InA_Left, OUTPUT);
  pinMode(InB_Left, OUTPUT);
  pinMode(Spd_Left, OUTPUT);

  pinMode(InA_Right, OUTPUT);
  pinMode(InB_Right, OUTPUT);
  pinMode(Spd_Right, OUTPUT);
#pragma endregion Motor_Pins

#pragma endregion PIN INITIALIZATION

#pragma region SERIAL_PORTS
  Serial.begin(115200);
  delay(2000); //10/06/21 - just use fixed delay instead

  Serial1.begin(115200);
  delay(2000); //11/20/21 use fixed delay instead of waiting

  if (Serial)
  {
    Serial.printf("Serial port active\n");
    gl_pSerPort = (Stream*)&Serial;
  }
  else if (Serial1)
  {
    Serial.printf("Serial1 port active\n");
    gl_pSerPort = (Stream*)&Serial1;
  }

  gl_pSerPort->printf("gl_pSerPort now points to active Serial (USB or Wixel)\n");

  //02/25/23 added for Garmin LIDAR-Lite V4/LED use
  Serial2.begin(115200);
  delay(2000); //11/20/21 use fixed delay instead of waiting

#pragma endregion SERIAL_PORTS

#pragma region I2C_PORTS
  //I2C bus
  Wire.begin();

  ////01/31/22 added to enable internal pullups on Wire - the I2C bus connected to 
  ////the T3.2 IR Homing processor and the MPU6050 MPU (the MPU does have internal 4.7K pullups installed)
  //CORE_PIN19_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
  //CORE_PIN18_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);

  //Wire1.begin();

  ////01/31/22 added to enable internal pullups on Wire1 - the I2C buss connected to the T3.5 VL53L0X array processor 
  //CORE_PIN37_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
  //CORE_PIN38_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);

  //Wire2.begin();

  //02/25/23 added to enable internal pullups on Wire2 - the I2C buss connected to the Garmin LIDAR-Lite V4/LED
  //03/08/23 c/o - prevents connection to Garmin LIDAR - don't know why
  //CORE_PIN3_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
  //CORE_PIN4_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
#pragma endregion I2C_PORTS

  CheckForUserInput(); //01/13/22 - here so OTA procedure can maybe start faster
  // 03/06/23 commented out - Garmin LIDAR won't connect on Wire2 with these lines active

#pragma region MPU6050
#ifndef NO_MPU6050
  gl_pSerPort->printf("\nChecking for MPU6050 IMU at I2C Addr 0x%x\n", MPU6050_I2C_ADDR);
  gl_pSerPort->println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  mpu.initialize();

  // verify connection

  float StartSec = 0; //used to time MPU6050 init
  gl_pSerPort->println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if successful)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    gl_pSerPort->printf(F("Enabling DMP...\n"));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    gl_pSerPort->println(F("DMP ready! Waiting for MPU6050 drift rate to settle..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    gl_pSerPort->printf(F("Calibrating...Retrieving Calibration Values\n\n"));
    mpu.CalibrateGyro(); //using default value of 15
    mpu.PrintActiveOffsets();

    //loop heading display until stabilized
    gl_pSerPort->printf(F("\nMsec\tHdg\n"));

    UpdateIMUHdgValDeg();
    Prev_HdgDeg = IMUHdgValDeg;
    delay(100);
    UpdateIMUHdgValDeg();

    gl_pSerPort->printf("%lu\t%2.3f\t%2.3f\n", millis(), IMUHdgValDeg, Prev_HdgDeg);
    while (abs(IMUHdgValDeg - Prev_HdgDeg) > 0.1f)
    {
      gl_pSerPort->printf("%lu\t%2.3f\n", millis(), IMUHdgValDeg);
      Prev_HdgDeg = IMUHdgValDeg;
      delay(100);
      UpdateIMUHdgValDeg();
    }

    StartSec = millis() / 1000.f;
    gl_pSerPort->printf("MPU6050 Ready at %2.2f Sec with delta = %2.3f\n", StartSec, IMUHdgValDeg - Prev_HdgDeg);
    bMPU6050Ready = true;

    delay(1000);
  }
  else //MPU6050 Init failed for some reason
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    gl_pSerPort->printf("DMP Initialization failed with code %d", devStatus);

    //08/29/21 print out battery voltage on failure
    //float batV = GetBattVoltage();
    gl_batteryVoltage = GetBattVoltage();
    gl_pSerPort->printf("Battery Voltage = %2.2f\n", gl_batteryVoltage);

    bMPU6050Ready = false;
  }
#endif // !NO_MPU6050
#pragma endregion MPU6050

  CheckForUserInput(); //01/13/22 - here so OTA procedure can maybe start faster

}

void loop()
{
  CheckForUserInput();
  delay(200); //08/17/23 put in to make sure distances are current
}


////Added 08/04/26
#pragma region MOTOR_SUPPORT
//09/08/20 modified for DRV8871 motor driver
//01/06/24 chg int gl_Leftspeednum, int gl_Rightspeednum to uint16_t leftspeednum, uint16_t rightspeednum
//void MoveReverse(int gl_Leftspeednum, int gl_Rightspeednum)
void MoveReverse(uint16_t leftspeednum, uint16_t rightspeednum)
{
  //Purpose:  Move in reverse direction continuously - companion to MoveAhead()
  //ProvEnA_Pinnce: G. Frank Paynter 09/08/18
  //Inputs:  
  //    leftspeednum = integer denoting speed (0=stop, 255 = full speed)
  //    rightspeednum = integer denoting speed (0=stop, 255 = full speed)
  //Outputs: both drive Motors energized with the specified speed
  //Plan:
  //    Step 1: Set reverse direction for both wheels
  //    Step 2: Run both Motors at specified speeds
  //Notes:
  //	01/22/20 now using Adafruit DRV8871 drivers

  //Step 1: Set reverse direction and speed for both wheels
//01/06/24 chg int gl_Leftspeednum, int gl_Rightspeednum to uint16_t leftspeednum, uint16_t rightspeednum
  //SetLeftMotorDirAndSpeed(REV_DIR, gl_Leftspeednum);
  //SetRightMotorDirAndSpeed(REV_DIR, gl_Rightspeednum);
  SetLeftMotorDirAndSpeed(REV_DIR, leftspeednum);
  SetRightMotorDirAndSpeed(REV_DIR, rightspeednum);
}

//09/08/20 modified for DRV8871 motor driver
//01/06/24 chg int gl_Leftspeednum, int gl_Rightspeednum to uint16_t leftspeednum, uint16_t rightspeednum
//void MoveAhead(int gl_Leftspeednum, int gl_Rightspeednum)
void MoveAhead(uint16_t leftspeednum, uint16_t rightspeednum)
{
  //Purpose:  Move ahead continuously
  //ProvEnA_Pinnce: G. Frank Paynter and Danny Frank 01/24/2014
  //Inputs:  
  //    leftspeednum = integer denoting speed (0=stop, 255 = full speed)
  //    rightspeednum = integer denoting speed (0=stop, 255 = full speed)
  //Outputs: both drive Motors energized with the specified speed
  //Plan:
  //    Step 1: Set forward direction for both wheels
  //    Step 2: Run both Motors at specified speeds
  //Notes:
  //	01/22/20 now using Adafruit DRV8871 drivers

  //gl_pSerPort->printf("InMoveAhead(%d,%d)\n", gl_Leftspeednum, gl_Rightspeednum);

  //Step 1: Set forward direction and speed for both wheels
//01/06/24 chg gl_Leftspeednum, gl_Rightspeednum to leftspeednum, rightspeednum
  //SetLeftMotorDirAndSpeed(true, gl_Leftspeednum);
  //SetRightMotorDirAndSpeed(true, gl_Rightspeednum);
  SetLeftMotorDirAndSpeed(true, leftspeednum);
  SetRightMotorDirAndSpeed(true, rightspeednum);
}

void StopBothMotors()
{
  StopLeftMotors();
  StopRightMotors();
}

//09/08/20 modified for DRV8871 motor driver
//11/04/21 modified for Pololu VNH5019 motor driver
void StopLeftMotors()
{
  digitalWrite(InA_Left, LOW);
  digitalWrite(InB_Left, LOW);
  analogWrite(Spd_Left, MOTOR_SPEED_OFF);
}

//11/04/21 modified for Pololu VNH5019 motor driver
void StopRightMotors()
{
  digitalWrite(InA_Right, LOW);
  digitalWrite(InB_Right, LOW);
  analogWrite(Spd_Right, MOTOR_SPEED_OFF);
}

//09/08/20 added bool bisFwd param for DRV8871 motor driver
//01/06/24 chg int gl_Leftspeednum, int gl_Rightspeednum to uint16_t leftspeednum, uint16_t rightspeednum
//void RunBothMotors(bool bisFwd, int gl_Leftspeednum, int gl_Rightspeednum)
void RunBothMotors(bool bisFwd, uint16_t leftspeednum, uint16_t rightspeednum)
{
  //Purpose: Run both Motors at left/rightspeednum speeds
  //Inputs:
  //    speednum = speed value (0 = OFF, 255 = full speed)
  //Outputs: Both Motors run for timesec seconds at speednum speed
  //Plan:
  //    Step 1: Apply drive to both wheels
  //Notes:
  //    01/14/15 - added left/right speed offset for straightness compensation
  //    01/22/15 - added code to restrict fast/slow values
  //    01/24/15 - revised for continuous run - no timing
  //  01/26/15 - speednum modifications moved to UpdateWallFollowmyMotorspeeds()
  //	12/07/15 - chg args from &int to int
  //Step 1: Apply drive to both wheels

//DEBUG!!
  //gl_pSerPort->printf("In RunBothMotors(%s, %d,%d)\n", bisFwd? "FWD":"REV", gl_Leftspeednum, gl_Rightspeednum);
//DEBUG!!

//01/06/24 chg gl_Leftspeednum, gl_Rightspeednum to leftspeednum, rightspeednum
  //SetLeftMotorDirAndSpeed(bisFwd, gl_Leftspeednum);
  //SetRightMotorDirAndSpeed(bisFwd, gl_Rightspeednum);
  SetLeftMotorDirAndSpeed(bisFwd, leftspeednum);
  SetRightMotorDirAndSpeed(bisFwd, rightspeednum);
}

void RunBothMotorsBidirectional(int leftspeed, int rightspeed)
{
  //Purpose:  Accommodate the need for independent bidirectional wheel motion
  //Inputs:
  //	leftspeed - integer denoting left wheel speed.  Positive value is fwd, negative is rev
  //	rightspeed - integer denoting right wheel speed.  Positive value is fwd, negative is rev
  //Outputs:
  //	left/right wheel motors direction and speed set as appropriate
  //Plan:
  //	Step1: Set left wheel direction and speed
  //	Step2: Set right wheel direction and speed

//Step1: Set left wheel direction and speed

  //DEBUG!!
  //gl_pSerPort->printf("In RunBothMotorsBidirectional(%d, %d)\n", leftspeed, rightspeed);
  if (leftspeed < 0)
  {
    SetLeftMotorDirAndSpeed(false, -leftspeed); //neg value ==> reverse
  }
  else
  {
    SetLeftMotorDirAndSpeed(true, leftspeed); //pos or zero value ==> fwd
  }

  //Step2: Set right wheel direction and speed
  if (rightspeed < 0)
  {
    SetRightMotorDirAndSpeed(false, -rightspeed); //neg value ==> reverse
  }
  else
  {
    SetRightMotorDirAndSpeed(true, rightspeed); //pos or zero value ==> fwd
  }
}

//09/08/20 added bool bisFwd param for DRV8871 motor driver
//06/01/24 chg int gl_Leftspeednum to uint16_t leftspeednum, int gl_Rightspeednum to uint16_t rightspeednum

//void RunBothMotorsMsec(bool bisFwd, int timeMsec, int gl_Leftspeednum, int gl_Rightspeednum)
void RunBothMotorsMsec(bool bisFwd, int timeMsec, uint16_t leftspeednum, uint16_t rightspeednum)
{
  //Purpose: Run both Motors for timesec seconds at speednum speed
  //Inputs:
  //    timesec = time in seconds to run the Motors
  //    speednum = speed value (0 = OFF, 255 = full speed)
  //Outputs: Both Motors run for timesec seconds at speednum speed
  //Plan:
  //    Step 1: Apply drive to both wheels
  //    Step 2: Delay timsec seconds
  //    Step 3: Remove drive from both wheels.
  //Notes:
  //    01/14/15 - added left/right speed offset for straightness compensation
  //    01/22/15 - added code to restrict fast/slow values
  //	11/25/15 - rev to use motor driver class object
  //	09/08/20 added bool bisFwd param for DRV8871 motor driver

//06/01/24 chg gl_Leftspeednum to leftspeednum, gl_Rightspeednum to rightspeednum
  //RunBothMotors(bisFwd, gl_Leftspeednum, gl_Rightspeednum);
  RunBothMotors(bisFwd, leftspeednum, rightspeednum);

  //Step 2: Delay timsec seconds
  delay(timeMsec);

  //Step3: Stop motors added 04/12/21
  StopBothMotors();
}

//11/25/15 added for symmetry ;-).
//06/01/24 chg speed signature from 'int' to 'uint16_t'
void SetLeftMotorDirAndSpeed(bool bIsFwd, uint16_t speed)
{
  //DEBUG!!
  //gl_pSerPort->printf("In SetLeftMotorDirAndSpeed(%s, %d)\n",
  //  (bIsFwd == true) ? "true" : "false", speed);
  //DEBUG!!

    //11/04/21 fwd for right motors is CCW when looking at shaft
  if (bIsFwd)
  {
    digitalWrite(InA_Left, LOW);
    digitalWrite(InB_Left, HIGH);
#ifndef NO_MOTORS
    analogWrite(Spd_Left, speed);
#endif // !NO_MOTORS

    //DEBUG!!
        //gl_pSerPort->printf("In TRUE block of SetLeftMotorDirAndSpeed(%s, %d)\n",
        //  (bIsFwd == true) ? "true" : "false", speed);
    //DEBUG!!
  }
  else
  {
    //DEBUG!!
        //gl_pSerPort->printf("In FALSE block of SetLeftMotorDirAndSpeed(%s, %d)\n",
        //  (bIsFwd == true) ? "true" : "false", speed);
    //DEBUG!!

    digitalWrite(InA_Left, HIGH);
    digitalWrite(InB_Left, LOW);
#ifndef NO_MOTORS
    analogWrite(Spd_Left, speed);
#endif // !NO_MOTORS
  }
}

//06/01/24 chg speed signature from 'int' to 'uint16_t'
void SetRightMotorDirAndSpeed(bool bIsFwd, uint16_t speed)
{
  //DEBUG!!
  //gl_pSerPort->printf("In SetRightMotorDirAndSpeed(%s, %d)\n",
  //  (bIsFwd == true) ? "true" : "false", speed);
  //DEBUG!!

    //11/04/21 fwd for right motors is CW when looking at shaft
#ifndef NO_MOTORS

  if (bIsFwd)
  {
    //DEBUG!!
        //gl_pSerPort->printf("In TRUE block of SetRighttMotorDirAndSpeed(%s, %d)\n",
        //  (bIsFwd == true) ? "true" : "false", speed);
    //DEBUG!!

    digitalWrite(InA_Right, HIGH);
    digitalWrite(InB_Right, LOW);
#ifndef NO_MOTORS
    analogWrite(Spd_Right, speed);
#endif // !NO_MOTORS
  }
  else
  {
    //DEBUG!!
        //gl_pSerPort->printf("In FALSE block of SetRightMotorDirAndSpeed(%s, %d)\n",
        //  (bIsFwd == true) ? "true" : "false", speed);
        //DEBUG!!

    digitalWrite(InA_Right, LOW);
    digitalWrite(InB_Right, HIGH);
#ifndef NO_MOTORS
    analogWrite(Spd_Right, speed);
#endif // !NO_MOTORS
  }

#endif // !NO_MOTORS
}

//05/05/23 added for symmetry and to simplify IsStuckAhead() code
bool AreBothMotorsForward()
{
  return IsRightMotorForward() && IsLeftMotorForward();
}

bool AreBothMotorsReversed()
{
  return IsRightMotorReversed() && IsLeftMotorReversed();
}

bool IsRightMotorForward()
{
  //gl_pSerPort->printf("in IsRightMotorForward(): digitalRead(InA_Right) = %d, digitalRead(InB_Right) = %d\n", digitalRead(InA_Right), digitalRead(InB_Right));
  return (digitalRead(InA_Right) == HIGH) && (digitalRead(InB_Right) == LOW);
}

bool IsLeftMotorForward()
{
  //gl_pSerPort->printf("In IsLeftMotorForward() with digitalRead(InA_Left) = %d, digitalRead(InB_Left) = %d\n",
  //  digitalRead(InA_Left), digitalRead(InB_Left));
  return (digitalRead(InB_Left) == HIGH) && (digitalRead(InA_Left) == LOW);
}

bool IsRightMotorReversed()
{
  //gl_pSerPort->printf("in IsRightMotorReversed(): digitalRead(InA_Right) = %d, digitalRead(InB_Right) = %d\n", digitalRead(InA_Right), digitalRead(InB_Right));
  return (digitalRead(InA_Right) == LOW) && (digitalRead(InB_Right) == HIGH);
}

bool IsLeftMotorReversed()
{
  //gl_pSerPort->printf("In IsLeftMotorReversed() with digitalRead(InA_Left) = %d, digitalRead(InB_Left) = %d\n",
  //  digitalRead(InA_Left), digitalRead(InB_Left));
  return (digitalRead(InB_Left) == LOW) && (digitalRead(InA_Left) == HIGH);
}

//05/05/23 these three functions added to cover edge case where robot is in STUCK_AHEAD situation,
//  but both motors are stopped ratherthan running in forward direction.  This occurs if robot gets 
//  stuck inside SpinTurn().  SpinTurn() will timeout and return false (with both motors stopped)
//  and this will cause program to jump to top of loop().  This *should* cause STUCK_AHEAD to be
//  detected, but it doesn't because it only checked for 'both motors forward', not 'both motors
//  stopped' (oops!).  To simplify IsStuckAhead(), I also added the 'AreBothMotorsForward() fcn
bool AreBothMotorsStopped()
{
  return IsLeftMotorStopped() && IsRightMotorStopped();
}

bool IsLeftMotorStopped()
{
  return (digitalRead(InA_Left) == LOW) && (digitalRead(InB_Left) == LOW);
}

bool IsRightMotorStopped()
{
  return (digitalRead(InA_Right) == LOW) && (digitalRead(InB_Right) == LOW);
}
#pragma endregion MOTOR_SUPPORT
//// ====================== SHARED HELPER ======================
void updateHeadingAndStationaryReset()
{
  //DEBUG!!
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0f;
  prevTime = now;

  int16_t gz = mpu.getRotationZ();
  float rate = gz / GYRO_Z_SENSITIVITY;
  gl_IMUHdgValDeg += rate * dt;
  Serial.printf("%d\t%2.3f\t%2.3f\t %2.3f\n", gz, GYRO_Z_SENSITIVITY, rate, gl_IMUHdgValDeg);

  if (abs(gz) < STATIONARY_THRESHOLD)
  {
    if (!isStationary)
    {
      stillStartTimeMsec = millis();
      isStationary = true;
    }
    if (millis() - stillStartTimeMsec > STATIONARY_TIME)
    {
      if (abs(gl_IMUHdgValDeg) > 0.5f)
      {
        Serial.print(">>> Heading auto-reset to 0.0 (was ");
        Serial.print(gl_IMUHdgValDeg, 2);
        Serial.println(")");
      }
      gl_IMUHdgValDeg = 0.0f;
    }
  }
  else
  {
    isStationary = false;
  }
}
//
//// ====================== MOTOR CONTROL (deadband + correct direction) ======================
//void setMotor(int dirPin, int pwmPin, float pidOutput)
//{
//  float absOut = abs(pidOutput);
//  if (absOut < 100.0f)   // deadband — motors need ~80-100 to start moving
//  {
//    analogWrite(pwmPin, 0);
//    return;
//  }
//
//  int pwm = constrain((int)absOut, 100, 255);
//
//  // Flipped polarity so +pidOutput = CW turn (your desired convention)
//  if (pidOutput > 0)
//  {
//    digitalWrite(dirPin, HIGH);
//    analogWrite(pwmPin, pwm);
//  }
//  else
//  {
//    digitalWrite(dirPin, LOW);
//    analogWrite(pwmPin, pwm);
//  }
//}
//
//#pragma endregion Motor Support Functions
//
//#pragma region IR_HOMING_SUPPORT
//void IRHomingNavigateToIAP()
//{
//  //Purpose:  Navigate to Initial Approach Point for charge station homing
//  //Provenance: 03/21/21
//  //Plan:
//  //	Step1: Rotate to maximum
//  //	Step1: Determine tracking direction
//  //	Step2: Call IRHomingNavigateToIAP(TRACKING_RIGHT) or IRHomingNavigateToIAP(TRACKING_LEFT)
//  //Notes:
//  //	04/16/21 rev to use new MoveToDesiredWallOffsetCm() function
//  //  02/17/22 cleaned up
//  //  05/18/22 added code to maximize IR beacon amplitude before doing anything else
//  //  05/26/22 added to use IRHomingl_RSteeringVal for initial turn dir (neg values --> CCW turn)
//  //  06/20/22 Positive steering value requires a CCW rotation for homing
//
//  UpdateAllDistances();//05/26/22 bugfix
//  UpdateIRHomingValues();//05/26/22 added for revised turn dir selection
//  gl_pSerPort->printf("OK we have a IR Beacon, with front distance = %d, LRSteer = %2.1f!\n",
//    gl_FrontCm, IRHomingl_RSteeringVal);
//
//  //DEBUG!!
//  delay(1000);
//  //DEBUG!!
//
//  gl_pSerPort->printf("In IRHomingNavigateToIAP with LF/LR/LS/RF/RR/RS = %2.1f\t%2.1f\t%2.2f\t%2.1f\t%2.1f\t%2.2f\n",
//    gl_LeftFrontCm, gl_LeftRearCm, gl_LeftSteeringVal, gl_RightFrontCm, gl_RightRearCm, gl_RightSteeringVal);
//
//  //Step1: Go to parallel orientation
//  WallTrackingCases trkside = (gl_LeftCenterCm > gl_RightCenterCm) ? TRACKING_RIGHT : TRACKING_LEFT;
//
//  if (trkside == TRACKING_RIGHT)
//  {
//    IRHomingNavigateToIAP(TRACKING_RIGHT);
//  }
//  else
//  {
//    IRHomingNavigateToIAP(TRACKING_LEFT);
//  }
//}
//
//void IRHomingNavigateToIAP(WallTrackingCases trkdir)
//{
//  //Purpose:  Navigate to Initial Approach Point for charge station homing
//  /*05/23/22 New plan.  I noticed today that the charger was detected while the
//   robot was tracking the wall opposite the charger, NOT the one parallel to the
//   charger, and this screwed up the 'navigate to IAP' algorithm. To handle this case
//   I plan to first stop the robot fine tune for parallel orientation and record the
//   heading and front/side differences.  Then I'll rotate it to maximize the IR signal,
//   using 'TurnToHomingBeacon()' and record this heading.  If the heading difference.
//   From the two headings and the front/side distances, I should be able to figure out
//   where the robot is relative to the charging station and take appropriate action to
//   line the robot up for homing.
//   */
//   //Inputs:
//   //  trkdir = WallTrackingCases enum denoting wall currently being tracked
//   //Outputs:
//   //  Robot at charging station beam IAP, pointed toward charger
//   //Plan:
//   //	Step1: Fine tune for parallel orientation, record side/front distances & hdg
//   //  Step2: Rotate robot to maximize the IR signal, and record hdg & front dist
//   //  Step3: If the robot is on the opposite or same wall
//   //     Step3a: Turn 90deg & go forward or backward to calculated IAP offset
//   //     Step3b: Turn back to re-acquire beacon signal
//   //     Step3c: If necessary, fine-tune orientation w/r/t beacon boresight
//   //  Step4: If the robot is on the opposite wall from the charging station.
//   //     Step4a: Turn back to parallel (can use previous heading change)
//   //     Step4b: go forward or backward to calculated IAP offset
//   //     Step4c: Turn back to re-acquire beacon signal
//   //     Step4d: If necessary, fine-tune orientation w/r/t beacon boresight
//   //Notes:
//   //	04/16/21 rev to use new MoveToDesiredFront/RearDistanceCm() functions
//   //  03/06/22 have to update distances manually - no longer updated in ISR
//   //  05/23/22 rewritten to handle both 'high side' and 'low side' approaches
//   //  05/26/22 rev call to RotateToParallelOrientation to use turn dir vs trkdir
//   //  06/20/22 A positive IR steer value should produce a CCW turn to home
//
//  UpdateIRHomingValues();//05/26/22 added for revised turn dir selection
//  gl_pSerPort->printf("In IRHomingNavigateToIAP(%s) with LRSteer = %2.1f\n",
//    WallTrackStrArray[trkdir], IRHomingl_RSteeringVal);
//
//  //Step2: Rotate robot to maximize the IR signal, and record hdg & front dist
//  TurnToHomingBeacon(IRHomingl_RSteeringVal >= 0); //06/20/22
//  UpdateAllEnvironmentParameters();
//  UpdateIMUHdgValDeg();
//
//  UpdateAllDistances();
//  uint16_t IRBeacon_frontd_cm = gl_FrontCm;
//  float IRBeacon_hdg_deg = IMUHdgValDeg;
//  uint16_t IPOffsetCm = (uint16_t)(IRBeacon_frontd_cm * 0.33f); //05/20/22 Wall-E3
//
//  gl_pSerPort->printf("\nNavToIAP: IP is at 45 cm offset, 135 cm away from charging station, a ratio of %2.1f\n", IRHOMING_DISTANCE_OFFSET_RATIO); //05/20/22 Wall-E3
//  gl_pSerPort->printf("NavToIAP: Robot is at %d from charger, so we need a %d cm offset\n\n",
//    IRBeacon_frontd_cm, IPOffsetCm);
//
//  //05/30/22 bugfix - have to do home to beacon first, then parallel
//  RotateToParallelOrientation(trkdir);
//
//  UpdateAllDistances();
//  uint16_t par_frontd_cm = gl_FrontCm; //updated in RotateToParallel
//  float par_hdg_deg = IMUHdgValDeg; //updated in RotateToParallel
//
//  gl_pSerPort->printf("\nNavToIAP(%s) with par/IR hdg deg = %2.1f/%2.1f, par/IR dist = %d/%d\n",
//    WallTrackStrArray[trkdir], par_hdg_deg, IRBeacon_hdg_deg, par_frontd_cm, IRBeacon_frontd_cm);
//
//  UpdateAllDistances(); //added 03/06/22
//
//  gl_pSerPort->printf("NavToIAP(%s) with LF/LR/LS/RF/RR/RS = %2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\n",
//    WallTrackStrArray[trkdir], gl_LeftFrontCm, gl_LeftRearCm, gl_LeftSteeringVal, gl_RightFrontCm,
//    gl_RightRearCm, gl_RightSteeringVal);
//
//  //Step3: Determine if the robot is on the opposite or same wall
//  if (abs(IRBeacon_hdg_deg - par_hdg_deg) < 30)
//  {
//    gl_pSerPort->printf("NavToIAP: Robot on same wall as charging station - proceed with normal homing\n");
//
//    //Step3a: Turn 90deg & go forward or backward to calculated IAP offset
//    float SideDistCm = (gl_RightCenterCm <= gl_LeftCenterCm) ? gl_RightCenterCm : gl_LeftCenterCm;
//    gl_pSerPort->printf("NavToIAP: Current wall offset is %2.1f cm\n", SideDistCm);
//    if (trkdir == TRACKING_RIGHT)
//    {
//      SpinTurn(true, 90); //05/01/22
//    }
//    else
//    {
//      SpinTurn(false, 90); //04/23/21 
//    }
//
//    gl_RearCm = GetRearDistCm();
//    gl_pSerPort->printf("NavToIAP: After SpinTurn(), rear dist (cm) is %2.1f\n", gl_RearCm);
//    MoveToDesiredRearDistCm(IPOffsetCm);
//    TurnToHomingBeacon(false);//IR beacon should be 90deg CW
//  }
//  else
//  {
//    gl_pSerPort->printf("\tNavToIAP: Robot on opposite wall from charging station - more work required\n");
//    //Step4a: Turn back to parallel (can use previous heading change)
//    RotateToParallelOrientation(trkdir);
//
//    //Step4b: go forward or backward to calculated IAP offset
//    //11/19/23 added SpinTurn() section - seemed to be missing
//    if (trkdir == TRACKING_RIGHT)
//    {
//      SpinTurn(true, 90); //05/01/22
//    }
//    else
//    {
//      SpinTurn(false, 90); //04/23/21 
//    }
//
//    //12/21/23 MoveToDesiredFrontDistCm() returns FALSE if robot encounters 'mirrored wall' prob
//    //MoveToDesiredFrontDistCm(IPOffsetCm);
//    if (!MoveToDesiredFrontDistCm(IPOffsetCm))
//    {
//      gl_pSerPort->printf("\tNavToIAP: MoveToDesiredFrontDistCm() failed!  Mirrored Wall problem? - Quitting!\n");
//      YellForHelp();
//    }
//
//    gl_pSerPort->printf("\tNavToIAP: Wall offset now %2.1f cm, with IRHomingValTotalAvg = %lu\n",
//      gl_RearCm, IRHomingValTotalAvg);
//    TurnToHomingBeacon(true);//IR beacon should be 90deg CCW
//  }
//
//  StopBothMotors();
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//  //IRHomingValTotalAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//  gl_pSerPort->printf("\tNavToIAP: Wall offset now %2.1f cm, with IRHomingValTotalAvg = %lu\n",
//    gl_RearCm, IRHomingValTotalAvg);
//
//  gl_pSerPort->printf("\tNavToIAP: Should be close to beacon centerline val avg = %lu, SteeringVal = %2.3f\n",
//    IRHomingValTotalAvg, IRHomingl_RSteeringVal);
//}
//
//bool IRHomeToChgStnNoPings(int initleftspeed, int initrightspeed)
//{
//  //Purpose:  Home in to charging station without distance information
//  //Inputs:
//  //Outputs:
//  //	robot homes to charging station
//  //Plan: 
//  //	Step2: Initialize PID for homing
//  //Notes:
//  //	03/19/17 rev to add initial left/right speed vals to calling sig
//  //  01/14/22 rev to use PIDCalcs vs PID library
//
//  bool result = true; //added 01/16/19 to suppress  warning
//
//  //Step2: Initialize PID for homing
//  IRHomingSetpoint = 0; //01/14/21 
//  float lastError = 0;
//  float lastInput = 0;
//  float lastIval = 0;
//  float lastDerror = 0;
//
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//  gl_pSerPort->printf("IRHomeToChgStnNoPings with detection value = %lu, Steering = %2.2f & IRHOMING Kp,Ki,Kd = (%2.0f,%2.0f,%2.0f)\n",
//    IRHomingValTotalAvg, IRHomingl_RSteeringVal, IRHomingKp, IRHomingKi, IRHomingKd);
//
//  gl_pSerPort->println(IRHomingTelemStrNoPings); //header for chg telemetry data
//
//  bool bChgConnect = false; //01/08/22 no longer using ISR, so have to generate locally
//  MsecSinceLastIRHomingAdj = 0; //could be a large value, so re-zero here
//
//  while (!IsChargerConnected(gl_bChgConnect) && gl_LastAnomalyCode != ANOMALY_STUCK_AHEAD)//11/08/23 repl gl_bChgConnect condx w !IsChargerConnected(gl_bChgConnect)
//    //while (!bChgConnect) //01/08/22 removed avoidancedistCm check
//  {
//    //05/02/20 turn on Laser
//    digitalWrite(RED_LASER_DIODE_PIN, HIGH);
//
//    //01/30/17 added to kill motors remotely using Wixel & serial port
//    CheckForUserInput();
//
//    if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//    {
//      MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//
//      UpdateIRHomingValues();
//
//      if (!isnan(IRHomingl_RSteeringVal))//skip bad values
//      {
//        //gl_pSerPort->printf("IRHomingl_RSteeringVal = %2.2f\n", IRHomingl_RSteeringVal);
//
//      //02/05/22 sampleTime removed from signature
//        IRHomingOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval,
//          lastDerror, IRHomingKp, IRHomingKi, IRHomingKd);
//
//        //reversed 01/11/22 - 01/14/22 - reversed it back again
//        //06/01/24 chg to local 'int' var to prevent unexpected behavior with negative values
//        //gl_Leftspeednum = initleftspeed + IRHomingOutput;
//        //gl_Rightspeednum = initrightspeed - IRHomingOutput;
//        int leftspeednum = initleftspeed + IRHomingOutput;
//        int rightspeednum = initrightspeed - IRHomingOutput;
//
//        //gl_pSerPort->printf("gl_Left/Rightspeednum = %d / %d\n", gl_Leftspeednum, gl_Rightspeednum);
//
//        //limit wheel speeds to valid range (0-255)
//        //gl_Leftspeednum = (gl_Leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Leftspeednum;
//        //gl_Leftspeednum = (gl_Leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Leftspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Rightspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Rightspeednum;
//        gl_Leftspeednum = (leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : leftspeednum;
//        gl_Leftspeednum = (leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : leftspeednum;
//        gl_Rightspeednum = (rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : rightspeednum;
//        gl_Rightspeednum = (rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : rightspeednum;
//
//        //DEBUG!!
//        gl_pSerPort->printf("%lu\t%2.2f\t%lu\t%lu\t%2.2f\t%2.2f\t\t%d\t%d\n",
//          millis(), GetBattVoltage(), IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingOutput,
//          gl_Leftspeednum, gl_Rightspeednum);
//
//        //UpdateRearLEDPanelForHoming(gl_Leftspeednum, gl_Rightspeednum);
//        UpdateRearLEDPanelForHoming(IRHomingl_RSteeringVal);
//
//        MoveAhead(gl_Leftspeednum, gl_Rightspeednum);
//      }
//
//      bChgConnect = IsChargerConnected(bChgConnect);
//    }
//  }
//
//  gl_pSerPort->printf("After while loop in IRHomeToChgStnNoPings\n");
//
//
//  if (bChgConnect)
//  {
//    Serial.print("Charger Connected at "); Serial.println(millis());
//    result = true; //added 01/16/19 to supress warning
//  }
//  else //added 12/28/20 for debug.
//  {
//    gl_pSerPort->printf("In IRHomeToChgStation: This code block should never execute! Stopping Program!");
//    StopBothMotors();
//    while (true)
//    {
//
//    }
//  }
//
//  return result; //added 01/16/19 to supress warning
//}
//bool IRHomeToChgStnNoPingsPID(int initleftspeed, float Kp, float Ki, float Kd)
//{
//  //Purpose:  Home in to charging station without distance information
//  //Inputs:
//  //Outputs:
//  //	robot homes to charging station
//  //Plan: 
//  //	Step2: Initialize PID for homing
//  //Notes:
//  //	03/19/17 rev to add initial left/right speed vals to calling sig
//  //  01/14/22 rev to use PIDCalcs vs PID library
//
//  bool result = true; //added 01/16/19 to suppress  warning
//
//  //Step2: Initialize PID for homing
//  IRHomingSetpoint = 0; //01/14/21 
//  float lastError = 0;
//  float lastInput = 0;
//  float lastIval = 0;
//  float lastDerror = 0;
//
//  int initrightspeed = initleftspeed;
//
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//  gl_pSerPort->printf("IRHomeToChgStnNoPings with detection value = %lu, Steering = %2.2f & IRHOMING Kp,Ki,Kd = (%2.0f,%2.0f,%2.0f)\n",
//    IRHomingValTotalAvg, IRHomingl_RSteeringVal, Kp, Ki, Kd);
//
//  gl_pSerPort->println(IRHomingTelemStrNoPings); //header for chg telemetry data
//
//  bool bChgConnect = false; //01/08/22 no longer using ISR, so have to generate locally
//  MsecSinceLastIRHomingAdj = 0; //could be a large value, so re-zero here
//
//  while (!IsChargerConnected(gl_bChgConnect) && gl_LastAnomalyCode != ANOMALY_STUCK_AHEAD)//11/08/23 repl gl_bChgConnect condx w !IsChargerConnected(gl_bChgConnect)
//    //while (!bChgConnect) //01/08/22 removed avoidancedistCm check
//  {
//    //05/02/20 turn on Laser
//    digitalWrite(RED_LASER_DIODE_PIN, HIGH);
//
//    //01/30/17 added to kill motors remotely using Wixel & serial port
//    CheckForUserInput();
//
//    if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//    {
//      MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//
//      UpdateIRHomingValues();
//
//      if (!isnan(IRHomingl_RSteeringVal))//skip bad values
//      {
//        //   //gl_pSerPort->printf("IRHomingl_RSteeringVal = %2.2f\n", IRHomingl_RSteeringVal);
//
//        // //02/05/22 sampleTime removed from signature
//        IRHomingOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval,
//          lastDerror, IRHomingKp, IRHomingKi, IRHomingKd);
//
//        //reversed 01/11/22 - 01/14/22 - reversed it back again
//        //06/01/24 chg to local 'int' var to prevent unexpected behavior with negative values
//        //gl_Leftspeednum = initleftspeed + IRHomingOutput;
//        //gl_Rightspeednum = initrightspeed - IRHomingOutput;
//        int leftspeednum = initleftspeed + IRHomingOutput;
//        int rightspeednum = initrightspeed - IRHomingOutput;
//
//        //gl_pSerPort->printf("gl_Left/Rightspeednum = %d / %d\n", gl_Leftspeednum, gl_Rightspeednum);
//
//        //limit wheel speeds to valid range (0-255)
//        //gl_Leftspeednum = (gl_Leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Leftspeednum;
//        //gl_Leftspeednum = (gl_Leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Leftspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Rightspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Rightspeednum;
//        gl_Leftspeednum = (leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : leftspeednum;
//        gl_Leftspeednum = (leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : leftspeednum;
//        gl_Rightspeednum = (rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : rightspeednum;
//        gl_Rightspeednum = (rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : rightspeednum;
//
//        //DEBUG!!
//        gl_pSerPort->printf("%lu\t%2.2f\t%lu\t%lu\t%2.2f\t%2.2f\t\t%d\t%d\n",
//          millis(), GetBattVoltage(), IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingOutput,
//          gl_Leftspeednum, gl_Rightspeednum);
//
//        //UpdateRearLEDPanelForHoming(gl_Leftspeednum, gl_Rightspeednum);
//        UpdateRearLEDPanelForHoming(IRHomingl_RSteeringVal);
//
//        MoveAhead(gl_Leftspeednum, gl_Rightspeednum);
//      }
//
//      bChgConnect = IsChargerConnected(bChgConnect);
//    }
//  }
//
//  gl_pSerPort->printf("After while loop in IRHomeToChgStnNoPings\n");
//
//
//  if (bChgConnect)
//  {
//    Serial.print("Charger Connected at "); Serial.println(millis());
//    result = true; //added 01/16/19 to supress warning
//  }
//  else //added 12/28/20 for debug.
//  {
//    gl_pSerPort->printf("In IRHomeToChgStation: This code block should never execute! Stopping Program!");
//    StopBothMotors();
//    while (true)
//    {
//
//    }
//  }
//
//  return result; //added 01/16/19 to supress warning
//}
//
//bool IRHomeToChgStn(int avoidancedistCm, int initleftspeed, int initrightspeed)
//{
//  //Purpose:  Home in to charging station with optional avoidance manuever
//  //Inputs:
//  //	avoidancedistCm = int denoting how far away to start avoidance maneuver
//  //Outputs:
//  //	either connected to charging station or turn away at avoidancedistCm
//  //Plan: 
//  //	Step1: Navigate to IAP
//  //	Step2: Initialize PID for homing
//  //	Step3: If front distance < avoidancedistCm, turn 90 deg away from near wall
//  //		   otherwise continue homing until connected or stuck
//  //Notes:
//  //	03/19/17 rev to add initial left/right speed vals to calling sig
//  //	08/09/20 added IsStuck() call, limited to 5 calls/sec to avoid false positives
//  //	08/10/20 now using timer ISR for this so only need to check bIsStuck state
//  //	11/08/20 moved I2C comms into timer ISR
//  //	01/23/21 added code to re-try docking if the robot gets stuck
//  //	02/07/21 rev to use ISR-managed bChgConnect vs local bChgConn
//  //	02/21/21 added new nav-to-IAP algorithm
//  //	04/03/21 added IRHomingPID.SetSampleTime(0) to make sure sample interval controlled by ISR
//  //  02/17/22 switch to homegrown PID
//  //  04/15/22 change 'Serial' to 'myTeePrint'
//
//  bool result = true; //added 01/16/19 to supress warning
//
//  //Step2: Initialize PID for homing
//  IRHomingSetpoint = 0; //01/14/21 
//  float lastError = 0;
//  float lastInput = 0;
//  float lastIval = 0;
//  float lastDerror = 0;
//
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//
//  UpdateAllEnvironmentParameters();
//
//  gl_pSerPort->printf("\tIRHomeToChgStn with detection value = %lu, Steering = %2.2f & IRHOMING Kp,Ki,Kd = (%2.0f,%2.0f,%2.0f)\n",
//    IRHomingValTotalAvg, IRHomingl_RSteeringVal, IRHomingKp, IRHomingKi, IRHomingKd);
//
//  //Step3: If front distance < avoidancedistCm, turn 90 deg away from near wall
//  //	   otherwise continue homing until connected or stuck
//  gl_pSerPort->printf("\nfront dist = %d, avoidance dist = %d, gl_bChgConnect = %d, anomaly code = %s\n",
//    gl_FrontCm, avoidancedistCm, gl_bChgConnect, AnomalyStrArray[gl_LastAnomalyCode]);
//  gl_pSerPort->printf("\n%s\n", IRHomingTelemStr); //header for chg telemetry data
//
//  MsecSinceLastIRHomingAdj = 0; //could be a large value, so re-zero here
//  digitalWrite(RED_LASER_DIODE_PIN, HIGH);  //05/02/20 turn on Laser
//  InitFrontDistArray();//06/16/22 bugfix.  prevent inadvertent STUCK detection
//
//  //06/11/23 rev to remove errcode conditions
//  //08/21/23 replaced '&& !gl_bStuckAhead' with '&& gl_LastAnomalyCode == ANOMALY_NONE'
//
//  //gl_pSerPort->printf("\tIRHomeToChgStn just before while() with anomaly code = %s\n", AnomalyStrArray[gl_LastAnomalyCode]);
//  while (!IsChargerConnected(gl_bChgConnect) && gl_FrontCm > avoidancedistCm && gl_LastAnomalyCode != ANOMALY_STUCK_AHEAD)//11/08/23 repl gl_bChgConnect condx w !IsChargerConnected(gl_bChgConnect)
//  {
//    //01/30/17 added to kill motors remotely using Wixel & serial port
//    CheckForUserInput();
//
//    if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//    {
//      MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//
//      //01/15/22 added to measure time required to do a PID computation loop
//      //DEBUG!!
//      digitalWrite(HOMING_PID_COMPUTE_CALL_PIN, HIGH);
//      //DEBUG!!
//
//      UpdateIRHomingValues();
//      UpdateAllEnvironmentParameters();
//
//      //skip bad values
//      if (!isnan(IRHomingl_RSteeringVal))
//      {
//        //gl_pSerPort->printf("IRHomingl_RSteeringVal = %2.2f\n", IRHomingl_RSteeringVal);
//
//        IRHomingOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval,
//          lastDerror, IRHomingKp, IRHomingKi, IRHomingKd);
//
//        //reversed 01/11/22 - 01/14/22 - reversed it back again
//        //06/01/24 chg to local 'int' var to prevent unexpected behavior with negative values
//        //gl_Leftspeednum = initleftspeed + IRHomingOutput;
//        //gl_Rightspeednum = initrightspeed - IRHomingOutput;
//        int leftspeednum = initleftspeed + IRHomingOutput;
//        int rightspeednum = initrightspeed - IRHomingOutput;
//
//        //gl_pSerPort->printf("gl_Left/Rightspeednum = %d / %d\n", gl_Leftspeednum, gl_Rightspeednum);
//
//        //limit wheel speeds to valid range (0-255)
//        //gl_Leftspeednum = (gl_Leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Leftspeednum;
//        //gl_Leftspeednum = (gl_Leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Leftspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Rightspeednum;
//        //gl_Rightspeednum = (gl_Rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Rightspeednum;
//        gl_Leftspeednum = (leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : leftspeednum;
//        gl_Leftspeednum = (leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : leftspeednum;
//        gl_Rightspeednum = (rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : rightspeednum;
//        gl_Rightspeednum = (rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : rightspeednum;
//
//        //DEBUG!!
//        gl_pSerPort->printf("%lu\t%2.2f\t%lu\t%lu\t%2.2f\t%2.2f\t\t%d\t%d\n",
//          millis(), GetBattVoltage(), IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingOutput,
//          gl_Leftspeednum, gl_Rightspeednum);
//
//        //UpdateRearLEDPanelForHoming(gl_Leftspeednum, gl_Rightspeednum);
//        UpdateRearLEDPanelForHoming(IRHomingl_RSteeringVal);
//
//        MoveAhead(gl_Leftspeednum, gl_Rightspeednum);
//      }
//    }
//    //gl_pSerPort->printf("\tIRHomeToChgStn at bottom of while() with anomaly code = %s\n", AnomalyStrArray[gl_LastAnomalyCode]);
//  }
//
//  if (gl_LastAnomalyCode == ANOMALY_STUCK_AHEAD) //08/21/23 trying again with anomaly code
//  {
//    gl_pSerPort->printf("Stuck in IRHomeToChargingStation() - Attempting to recover\n");
//    gl_pSerPort->printf("Msec\tLF\tLC\tLR\tRF\tRC\tRR\tIR\n");
//
//    gl_pSerPort->printf("%lu\t%d\t%d\t%d\t%d\t%d\t%d\t%2.3f\n", millis(), gl_LeftFrontCm, gl_LeftCenterCm, gl_LeftRearCm,
//      gl_RightFrontCm, gl_RightCenterCm, gl_RightRearCm, IRHomingl_RSteeringVal);
//
//    //reverse for 0.5 sec, then pause for 2 sec
//    MoveReverse(MOTOR_SPEED_HALF, MOTOR_SPEED_HALF);
//    delay(500);
//    StopBothMotors();
//    delay(2000);
//
//    while (abs(IRHomingl_RSteeringVal) > 0.1f)
//    {
//      //01/30/17 added to kill motors remotely using Wixel & serial port
//      CheckForUserInput();
//
//      if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//      {
//        MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//
//        if (!isnan(IRHomingl_RSteeringVal))
//        {
//          IRHomingOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval,
//            lastDerror, IRHomingKp, IRHomingKi, IRHomingKd);
//
//          //reversed 01/11/22 - 01/14/22 - reversed it back again
//        //06/01/24 chg to local 'int' var to prevent unexpected behavior with negative values
//        //gl_Leftspeednum = initleftspeed + IRHomingOutput;
//        //gl_Rightspeednum = initrightspeed - IRHomingOutput;
//          int leftspeednum = initleftspeed + IRHomingOutput;
//          int rightspeednum = initrightspeed - IRHomingOutput;
//
//          //gl_pSerPort->printf("gl_Left/Rightspeednum = %d / %d\n", gl_Leftspeednum, gl_Rightspeednum);
//
//          //limit wheel speeds to valid range (0-255)
//          //gl_Leftspeednum = (gl_Leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Leftspeednum;
//          //gl_Leftspeednum = (gl_Leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Leftspeednum;
//          //gl_Rightspeednum = (gl_Rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Rightspeednum;
//          //gl_Rightspeednum = (gl_Rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Rightspeednum;
//          gl_Leftspeednum = (leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : leftspeednum;
//          gl_Leftspeednum = (leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : leftspeednum;
//          gl_Rightspeednum = (rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : rightspeednum;
//          gl_Rightspeednum = (rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : rightspeednum;
//
//          //DEBUG!!
//          gl_pSerPort->printf("%lu\t%2.2f\t%lu\t%lu\t%2.2f\t%2.2f\t\t%d\t%d\n",
//            millis(), GetBattVoltage(), IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingOutput,
//            gl_Leftspeednum, gl_Rightspeednum);
//
//          //UpdateRearLEDPanelForHoming(gl_Leftspeednum, gl_Rightspeednum);
//          UpdateRearLEDPanelForHoming(IRHomingl_RSteeringVal);
//
//          MoveReverse(gl_Leftspeednum, gl_Rightspeednum);
//        }
//      }
//    }
//
//    gl_pSerPort->printf("Succeeded!  Robot should now be lined up again!  Stopping motors\n");
//    StopBothMotors();
//    delay(1000);
//    gl_pSerPort->printf("Trying again to dock with Charging Station\n");
//    delay(1000);
//
//    MsecSinceLastIRHomingAdj = 0;
//
//    gl_FrontCm = GetFrontDistCm(); //added 04/29/22
//    MsecSinceLastIRHomingAdj = 0; //added 04/29/22
//
//    while (gl_bChgConnect == LOW && gl_FrontCm > avoidancedistCm) //01/23/21 removed bIsStuck - handled inside while()
//    {
//      //05/02/20 turn on Laser
//      digitalWrite(RED_LASER_DIODE_PIN, HIGH);
//
//      //01/30/17 added to kill motors remotely using Wixel & serial port
//      CheckForUserInput();
//
//      if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//      {
//        MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//
//        //01/15/22 added to measure time required to do a PID computation loop
//        //DEBUG!!
//        digitalWrite(HOMING_PID_COMPUTE_CALL_PIN, HIGH);
//        //DEBUG!!
//
//        UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//        gl_pSerPort->printf("%lu\t%lu\t%lu\t%lu\t%2.2f\n",
//          millis(), IRFinalValue1, IRFinalValue2, IRHomingValTotalAvg, IRHomingl_RSteeringVal);
//
//        //skip bad values
//        if (!isnan(IRHomingl_RSteeringVal))
//        {
//          //gl_pSerPort->printf("IRHomingl_RSteeringVal = %2.2f\n", IRHomingl_RSteeringVal);
//
//          //02/05/22 sampleTime removed from signature
//          IRHomingOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval,
//            lastDerror, IRHomingKp, IRHomingKi, IRHomingKd);
//
//          //reversed 01/11/22 - 01/14/22 - reversed it back again
//        //06/01/24 chg to local 'int' var to prevent unexpected behavior with negative values
//        //gl_Leftspeednum = initleftspeed + IRHomingOutput;
//        //gl_Rightspeednum = initrightspeed - IRHomingOutput;
//          int leftspeednum = initleftspeed + IRHomingOutput;
//          int rightspeednum = initrightspeed - IRHomingOutput;
//
//          //gl_pSerPort->printf("gl_Left/Rightspeednum = %d / %d\n", gl_Leftspeednum, gl_Rightspeednum);
//
//          //limit wheel speeds to valid range (0-255)
//          //gl_Leftspeednum = (gl_Leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Leftspeednum;
//          //gl_Leftspeednum = (gl_Leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Leftspeednum;
//          //gl_Rightspeednum = (gl_Rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : gl_Rightspeednum;
//          //gl_Rightspeednum = (gl_Rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : gl_Rightspeednum;
//          gl_Leftspeednum = (leftspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : leftspeednum;
//          gl_Leftspeednum = (leftspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : leftspeednum;
//          gl_Rightspeednum = (rightspeednum > MOTOR_SPEED_FULL) ? MOTOR_SPEED_FULL : rightspeednum;
//          gl_Rightspeednum = (rightspeednum < MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : rightspeednum;
//
//          //DEBUG!!
//          gl_pSerPort->printf("%lu\t%2.2f\t%lu\t%lu\t%2.2f\t%2.2f\t\t%d\t%d\n",
//            millis(), GetBattVoltage(), IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingOutput,
//            gl_Leftspeednum, gl_Rightspeednum);
//
//          //UpdateRearLEDPanelForHoming(gl_Leftspeednum, gl_Rightspeednum);
//          UpdateRearLEDPanelForHoming(IRHomingl_RSteeringVal);
//
//          MoveAhead(gl_Leftspeednum, gl_Rightspeednum);
//        }
//
//        //01/15/22 added to measure time required to do a PID computation loop (w/o pings)
//        //DEBUG!!
//        digitalWrite(HOMING_PID_COMPUTE_CALL_PIN, LOW);
//        //DEBUG!!
//
//      }
//
//      gl_FrontCm = GetFrontDistCm(); //added 04/29/22
//    }
//  }
//
//  //find out why loop exited.  Could be stuck, connected, or inside avoidance dist
//  int leftdist = GetAvgLeftDistCm();
//  int rightdist = GetAvgRightDistCm();
//
//  gl_FrontCm = GetFrontDistCm(); //added 04/29/22
//  if (gl_FrontCm <= avoidancedistCm)
//  {
//    gl_pSerPort->printf("Abnormal exit from homing routine\n");
//
//    //gl_pSerPort->printf("%lu: front/left/rightdist/bIsStuckAhead = %d/%d/%d/%s\n",
//      //millis(), gl_FrontCm, leftdist, rightdist, gl_bStuckAhead ? "TRUE" : "FALSE");
//
//    //08/21/23 gl_bStuckAhead repl by AnomalyStrArray[ANOMALY_STUCK_AHEAD]
//    gl_pSerPort->printf("%lu: front/left/rightdist/bIsStuckAhead = %d/%d/%d/%s\n",
//      millis(), gl_FrontCm, leftdist, rightdist, AnomalyStrArray[ANOMALY_STUCK_AHEAD]);
//
//    InitFrontDistArray(); //added 08/12/20 to prevent multiple 'stuck' detections
//
//    //12/21/23 BackupAndTurn90Deg() can now return FALSE if robot hits 'mirrored wall' prob
//    //BackupAndTurn90Deg((leftdist > rightdist));//09/26/23 removed motor_speed & fwd/bkwd parameters
//    if (!BackupAndTurn90Deg((leftdist > rightdist)))
//    {
//      gl_pSerPort->printf("BackupAndTurn90Deg() failed after abnormal exit from IRHomeToChgStn() - Mirrored Wall issue?\n");
//    }
//
//    result = false; //added 01/16/19 to supress warning
//  }
//
//  else if (gl_bChgConnect == HIGH)
//  {
//    gl_pSerPort->print("Charger Connected at "); gl_pSerPort->println(millis());
//    result = true; //added 01/16/19 to supress warning
//  }
//  else //added 12/28/20 for debug.
//  {
//    gl_pSerPort->printf("In IRHomeToChgStation: This code block should never execute! Stopping Program!\n");
//    StopBothMotors();
//    gl_pSerPort->printf("Abnormal exit from homing routine\n");
//    //gl_pSerPort->printf("%lu: front/rear dist, left/rightdist, bIsStuckAhead/gl_bChgConnect = %d/%d, %d/%d, %s/%s\n",
//    //  millis(), gl_FrontCm, gl_RearCm, leftdist, rightdist, gl_bStuckAhead ? "TRUE" : "FALSE", gl_bChgConnect ? "TRUE" : "FALSE");
//
//    //08/21/23 gl_bStuckAhead repl by AnomalyStrArray[ANOMALY_STUCK_AHEAD]
//    gl_pSerPort->printf("%lu: front/rear dist, left/rightdist, bIsStuckAhead/gl_bChgConnect = %d/%d, %d/%d, %s/%s\n",
//      millis(), gl_FrontCm, gl_RearCm, leftdist, rightdist, AnomalyStrArray[ANOMALY_STUCK_AHEAD], gl_bChgConnect ? "TRUE" : "FALSE");
//
//
//    while (true)
//    {
//      CheckForUserInput();
//      delay(100);
//    }
//  }
//
//  return result; //added 01/16/19 to suppress compiler warning
//}
//
//void UpdateIRHomingValues()
//{
//  //Purpose: Update all the IR Homing related values
//  //Provenance:  Ported from FourWD_WallE2_V12 01/07/22
//  //Inputs:
//  //  none
//  //Outputs: 
//  //  IRFinalValue1/2 = long ints denoting left/right computed demodulator outputs
//  //  IRHomingl_RSteeringVal = float denoting left/right steering error
//  //  IRHomingValTotalAvg updated
//  //Notes:
//  //  06/28/22 added call to CalcIRHomingValueTotalAverage to update IRHomingValTotalAvg
//
//  //gl_pSerPort->printf("Requesting %d, %d, %d bytes from IRDet\n", sizeof(IRFinalValue1), sizeof(IRFinalValue2), sizeof(IRHomingl_RSteeringVal));
//  Wire.requestFrom(IR_HOMING_MODULE_SLAVE_ADDR, sizeof(IRFinalValue1) + sizeof(IRFinalValue2) + sizeof(IRHomingl_RSteeringVal));
//  I2C_readAnything(IRFinalValue1);
//  I2C_readAnything(IRFinalValue2);
//  I2C_readAnything(IRHomingl_RSteeringVal);
//
//  IRHomingValTotalAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);//added 06/28/22
//
//  //gl_pSerPort->printf("Got %lu, %lu, %2.2f from IR Detector module, IRHomingValTotalAvg = %lu\n", 
//  //  IRFinalValue1, IRFinalValue2, IRHomingl_RSteeringVal, IRHomingValTotalAvg);
//}
//
//uint32_t CalcIRHomingValueTotalAverage(uint32_t finval1, uint32_t finval2)
//{
//  //Purpose: compute new IR homing value total average
//  //Inputs:
//  //  finval1/2 = uint32_t objects representing left & right side IR sensor outputs
//  //Output:
//  //  returns new average of last IRHOMING_VALUE_ARRAY_SIZE values
//  //Plan:
//  // Step1: Add new total to top of array, bump everything down one, oldest value to bit-bucket
//  // Step2: Calc new average
//  //Notes:
//  //  05/08/22 this replaces UpdateIRHomingValueTotalAverage to (hopefully) fix probs with neg avg values
//
//
////Step1: Add new total to top of array, bump everything down one, oldest value to bit-bucket
//  uint32_t newValTotal = finval1 + finval2;
//  //gl_pSerPort->printf("In CalcIRHomingValueTotalAverage(%lu, %lu), newValTotal = %lu\n", finval1,  finval2, newValTotal);
//  for (int i = 0; i < IRHOMING_VALUE_ARRAY_SIZE - 1; i++)
//  {
//    aIRHOMINGVALTOTALS[i] = aIRHOMINGVALTOTALS[i + 1];
//    //gl_pSerPort->printf("aIRHOMINGVALTOTALS[%d] = %d\n", i, aIRHOMINGVALTOTALS[i]);
//  }
//  aIRHOMINGVALTOTALS[IRHOMING_VALUE_ARRAY_SIZE - 1] = newValTotal; //new total value
//
//  //DEBUG!!
//  //for (int i = 0; i < IRHOMING_VALUE_ARRAY_SIZE; i++)
//  //{
//  // gl_pSerPort->printf("aIRHOMINGVALTOTALS[%d] = %d\n", i, aIRHOMINGVALTOTALS[i]);
//  //}
//  //DEBUG!!
//
////Step2: Calc new average
//  uint32_t ir_tot_sum = 0;
//  for (int i = 0; i < IRHOMING_VALUE_ARRAY_SIZE; i++)
//  {
//    ir_tot_sum += aIRHOMINGVALTOTALS[i];
//  }
//
//  uint32_t ir_tot_avg = ir_tot_sum / IRHOMING_VALUE_ARRAY_SIZE; //int truncation OK
//
//  //DEBUG!!
//  //gl_pSerPort->printf("%lu: FinVal1/2 = %lu/%lu, new_avg = %lu\n",
//  //    millis(), finval1, finval2, ir_tot_avg);
//  //DEBUG!!
//
//  return ir_tot_avg;
//}
//
//void UpdateRearLEDPanelForHoming(double steerval)
//{
//  //Purpose:  Update rear panel LEDs to show homing activity
//  //Provenance: Created 05/03/17 gfp
//  //Inputs:
//  //	steerval = float representing steering value reported by IR Homing module
//  //Outputs:
//  //	Rear LED panel updated to reflect steering value
//  //Plan:
//  //	Step1:  Disable all LEDs to blank the display
//  //	Step2:  Enable the appropriate LEDs on each side
//  //Notes:
//  //  04/24/22 changed sig to just use steering value, and to use switch() vs if/else
//
//  //Step1:  Disable all LEDs to blank the display
//  EnableAllRearLEDs(false); //repl DisableAllRearPanelLEDs() 04/02/21
//
//  //gl_pSerPort->printf("In UpdateRearLEDPanelForHoming(%2.1f) just after EnableAllLEDs(false)\n", steerval);
//  //Step2:  Enable the appropriate LEDs on each side
//  int16_t steerval_int = (int16_t)(100.f * steerval); //cvt steerval to int - truncation is OK
//
//  //at this point, steerval_int is an integer in the range of -100 to +100
//  //use steps of 33 to spread over 6 LEDs 
//  switch (steerval_int)
//  {
//  case -100 ... - 67:
//    digitalWrite(CHG_CONNECT_LED_PIN, LOW);
//    break;
//  case -66 ... - 33:
//    digitalWrite(_20PCT_LED_PIN, LOW);
//    break;
//  case -32 ... 1:
//    digitalWrite(_40PCT_LED_PIN, LOW);
//    break;
//  case 2 ... 35:
//    digitalWrite(_60PCT_LED_PIN, LOW);
//    break;
//  case 36 ... 69:
//    digitalWrite(_80PCT_LED_PIN, LOW);
//    break;
//  case 70 ... 100:
//    //gl_pSerPort->printf("In UpdateRearLEDPanelForHoming(%2.1f) 70 ... 100:\n", steerval);
//    digitalWrite(CHG_FIN_LED_PIN, LOW);
//    break;
//  default:
//    gl_pSerPort->printf("In UpdateRearLEDPanelForHoming(%2.1f) default case:  This should never execute!\n", steerval);
//    break;
//  }
//}
//
////05/31/22 rewritten to turn to max before tuning
//void TurnToHomingBeacon(bool isCCW)
//{
//  //Purpose: turn to maximize the IR beacon signal strength
//  //Inputs: 
//  //	isCCW = bool object denoting which way to turn
//  //Outputs:
//  //	robot oriented toward IR homing beacon
//  //Plan:
//  //Step1: rotate robot to maximize IR Homing value total (finval1 + finval2)
//  //Step2: Initialize IRHomingPID for centering
//  //Step3: Rotate back and forth as necessary to maximize received signal strength.  Start with CW rotation
//  //				for trkside == TRACKING_RIGHT, and CCW for TRACKING_LEFT
//  //Notes:
//  //  05/19/22 rev to pause after each angle step, and watch for steering val sign change
//  //  05/26/22 rev to use boolean vs WallTrackingCases parameter for initial turn direction
//  //  05/27/22 rewrote to just use steering val and PID to home
//  //  05/28/22 now using sum of abs steervals to terminate homing
//  //  06/12/22 rewritten per test campaign
//  //  06/20/22 A positive IR steer value should produce a CCW turn to home
//
//  //DEBUG!!
//  digitalWrite(RED_LASER_DIODE_PIN, HIGH); //turn laser on
//  //DEBUG!!
//
//
////Step1: rotate robot to maximize IR Homing value total (finval1 + finval2)
//  UpdateIRHomingValues();
//
//  gl_pSerPort->printf("\nIn TurnToHomingBeacon(%s) with LRSteer = %2.1f\n",
//    isCCW ? "CCW" : "CW", IRHomingl_RSteeringVal);
//
//  uint32_t beaconAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//  float start_deg = UpdateIMUHdgValDeg();
//  uint16_t start_dist = GetFrontDistCm();
//  float curr_steer = IRHomingl_RSteeringVal;
//  float prev_steer = curr_steer;
//
//  gl_pSerPort->printf("At start: beaconAvg = %lu, deg = %2.1f with front dist = %d\n",
//    beaconAvg, start_deg, start_dist);
//
//  gl_pSerPort->printf("\nMsec\tDeg\tCurr\tPrev\tAvg\n");
//
//  isCCW ? RunBothMotorsBidirectional(-MOTOR_SPEED_LOW, MOTOR_SPEED_LOW) :
//    RunBothMotorsBidirectional(MOTOR_SPEED_LOW, -MOTOR_SPEED_LOW);
//
//  while (beaconAvg < IR_BEAM_DETECTION_THRESHOLD || curr_steer * prev_steer > 0)
//  {
//    UpdateIRHomingValues();
//    float deg = UpdateIMUHdgValDeg();
//    beaconAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//    prev_steer = curr_steer;
//    curr_steer = IRHomingl_RSteeringVal;
//
//    gl_pSerPort->printf("%lu\t%2.1f\t%2.1f\t%2.1f\t%lu\n", millis(), deg, curr_steer, prev_steer, beaconAvg);
//    delay(100);
//    CheckForUserInput();
//  }
//  StopBothMotors();
//
//  float end_deg = UpdateIMUHdgValDeg();
//  uint16_t end_dist = GetFrontDistCm();
//  gl_pSerPort->printf("%lu: Coarse tuning loop exited at %2.1fdeg with avg = %lu and frontdist = %d\n",
//    millis(), end_deg, beaconAvg, end_dist);
//
//  //Step2: Initialize IRHomingPID for centering
//  float lastError = 0;
//  float lastInput = 0;
//  float lastIval = 0;
//  float lastDerror = 0;
//
//  const float IRRotateKp = 40.f;
//  const float IRRotateKi = 5.f;
//  const float IRRotateKd = 0.f;
//
//  float IRRotateOutput;
//
//  StopBothMotors();
//
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//  //IRHomingValTotalAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//
//  gl_pSerPort->printf("\nFine Tuning: TurnToHomingBeacon(%s) with IRAvg = %lu, Steering = %2.2f & IRRotate Kp,Ki,Kd = (%2.0f,%2.0f,%2.0f)\n",
//    isCCW ? "CCW" : "CW", IRHomingValTotalAvg, IRHomingl_RSteeringVal, IRRotateKp, IRRotateKi, IRRotateKd);
//
//  //Step3: Rotate back and forth as necessary to maximize received signal strength.  Start with CW rotation
//  //				for trkside == TRACKING_RIGHT, and CCW for TRACKING_LEFT
//  gl_pSerPort->printf("\tTTHB Fine Tune Start: Got %lu, %lu, %lu, %2.2f from IR Detector module\n",
//    IRFinalValue1, IRFinalValue2, IRHomingValTotalAvg, IRHomingl_RSteeringVal);
//
//  //05/28/22 start by filling steerval array with latest data
//  float SteerValAbsSum = InitSteerValAbsSum();
//
//  gl_pSerPort->printf("\nMsec\tIRVal1\tIRVal2\tValAvg\tSteer\tspeed\n");
//  UpdateIRHomingValues();
//  SteerValAbsSum = UpdateSteerValAbsSum(IRHomingl_RSteeringVal);
//  MsecSinceLastIRHomingAdj = 0;
//
//  //05/28/22 now using sum of abs steervals to terminate
//  while (SteerValAbsSum > IRHOMING_IAP_STEERING_VALUE_THRESHOLD)
//  {
//    CheckForUserInput();
//
//    if (MsecSinceLastIRHomingAdj >= MSEC_PER_IR_HOMING_ADJ)
//    {
//      MsecSinceLastIRHomingAdj -= MSEC_PER_IR_HOMING_ADJ;
//      if (!isnan(IRHomingl_RSteeringVal))
//      {
//        UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//        //IRHomingValTotalAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//        SteerValAbsSum = UpdateSteerValAbsSum(IRHomingl_RSteeringVal);
//        UpdateAllDistances();
//
//        //gl_pSerPort->printf("Just before PIDCalcs - Got %lu, %lu, %lu, %2.2f from IR Detector module\n", IRFinalValue1, IRFinalValue2, IRHomingValTotalAvg, IRHomingl_RSteeringVal);
//        IRRotateOutput = PIDCalcs(IRHomingl_RSteeringVal, 0, lastError, lastInput, lastIval, lastDerror, IRRotateKp, IRRotateKi, IRRotateKd);
//
//        //04/29/22 guard against too-large output
//        //05/01/22 have to handle negative PID output too
//        int16_t speed = 0;
//        if (IRRotateOutput >= 0)
//        {
//          speed = (IRRotateOutput > MOTOR_SPEED_QTR) ? MOTOR_SPEED_QTR : (int16_t)IRRotateOutput;
//        }
//        else
//        {
//          speed = (IRRotateOutput < -MOTOR_SPEED_QTR) ? -MOTOR_SPEED_QTR : (int16_t)IRRotateOutput;
//        }
//
//        RunBothMotorsBidirectional(speed, -speed);//05/30/22 2210hrs
//
//        gl_pSerPort->printf("%lu\t%lu\t%lu\t%lu\t%2.2f\t%d\n",
//          millis(), IRFinalValue1, IRFinalValue2, IRHomingValTotalAvg, IRHomingl_RSteeringVal, speed);
//      }
//    }
//  }
//
//  UpdateIRHomingValues();//06/28/22 now inc IRHomingValTotalAvg update
//  //IRHomingValTotalAvg = CalcIRHomingValueTotalAverage(IRFinalValue1, IRFinalValue2);
//  gl_pSerPort->printf("\tTTHB Fine Tune End: %lu, %lu, %lu, %2.2f from IR Detector module\n\n",
//    IRFinalValue1, IRFinalValue2, IRHomingValTotalAvg, IRHomingl_RSteeringVal);
//
//  end_deg = UpdateIMUHdgValDeg();
//  end_dist = GetFrontDistCm();
//  gl_pSerPort->printf("%lu: start/end_deg = %2.1f/%2.1f, start/end_dist = %d/%d\n\n",
//    millis(), start_deg, end_deg, start_dist, end_dist);
//
//  //DEBUG!!
//  digitalWrite(RED_LASER_DIODE_PIN, LOW); //turn laser off
//  //DEBUG!!
//}
//
//float InitSteerValAbsSum()
//{
//  //Purpose: Initialize absolute IR steer values & return sum
//  float sum = 0;
//  for (size_t i = 0; i < IRHOMING_STEERVALUE_ARRAY_SIZE; i++)
//  {
//    UpdateIRHomingValues();
//    aIRHOMINGSTEERVALS[i] = abs(IRHomingl_RSteeringVal);
//    sum += aIRHOMINGSTEERVALS[i];
//    delay(100);
//  }
//
//  return sum;
//}
//
//float UpdateSteerValAbsSum(float steerval)
//{
//  //Purpose:  Update array and sum of absolute steering values with latest input
//  //Inputs: steerval = float value representing latest IRHomingl_RSteeringVal:
//  //Outputs: returns sum of absolute IR homing steering values
//  //Plan:
//  //  Step1: push all entries down 1 in array, discarding oldest
//  //  Step2: put new abs(IR homing steer value) at top
//  //  Step3: update sum of abs values
//
//  //gl_pSerPort->printf("In UpdateSteerValAbsSum()\n");
//
////Step1: push all entries down 1 in array, discarding oldest & put new abs(IR homing steer value) at top
//  float sum = 0;
//  for (size_t i = 0; i < IRHOMING_STEERVALUE_ARRAY_SIZE - 1; i++)
//  {
//    aIRHOMINGSTEERVALS[i] = aIRHOMINGSTEERVALS[i + 1];
//  }
//
//  //Step2: put new abs(IR homing steer value) at top
//  aIRHOMINGSTEERVALS[IRHOMING_STEERVALUE_ARRAY_SIZE - 1] = abs(steerval);
//
//
//
//  //Step3: update sum of abs values
//  for (size_t i = 0; i < IRHOMING_STEERVALUE_ARRAY_SIZE; i++)
//  {
//    //gl_pSerPort->printf("abs steerval[%d] = %2.2f\n", i, aIRHOMINGSTEERVALS[i]);
//    sum += aIRHOMINGSTEERVALS[i];
//  }
//  return sum;
//}
//
//void InitIRHomingAverageArray()
//{
//  gl_pSerPort->printf("%lu: Initializing IR Beam Total Value Averaging Array...", millis());
//  for (size_t i = 0; i < IRHOMING_VALUE_ARRAY_SIZE; i++)
//  {
//    aIRHOMINGVALTOTALS[i] = 0;
//  }
//  IRHomingValTotalAvg = 0;
//  gl_pSerPort->printf("Done\n", millis());
//}
//#pragma endregion IR_HOMING_SUPPORT
//
//#pragma region VL53L0X_SUPPORT
//void WaitForVL53L0XTeensy()
//{
//  gl_bVL53L0X_TeensyReady = false;
//  while (!gl_bVL53L0X_TeensyReady)
//  {
//    CheckForUserInput();//06/06/22 bugfix
//    GetRequestedVL53l0xValues(VL53L0X_READYCHECK); //this updates gl_bVL53L0X_TeensyReady
//    gl_pSerPort->printf("%lu: got %d from VL53L0X Teensy\n", millis(), gl_bVL53L0X_TeensyReady);
//    delay(100);
//  }
//
//  gl_pSerPort->printf("Teensy setup() finished at %lu mSec\n", millis());
//
//  //now try to get a VL53L0X measurement
//  //11/08/20 rev to loop until all distance sensors provide valid data
//
//  //gl_pSerPort->printf("Msec\tLFront\tLCtr\tLRear\tRFront\tRCtr\tRRear\tRear\n");
//
//  GetRequestedVL53l0xValues(VL53L0X_ALL);
//
//  //05/02/23 added a try limit
//  uint16_t sensor_tries = 0;
//  while ((gl_LeftFrontCm <= 0 || gl_LeftCenterCm <= 0 || gl_LeftRearCm <= 0
//    || gl_LeftFrontCm <= 0 || gl_LeftCenterCm <= 0 || gl_LeftRearCm <= 0
//    || gl_RearCm <= 0) && sensor_tries <= 100)
//  {
//    GetRequestedVL53l0xValues(VL53L0X_ALL);
//    delay(100);
//    sensor_tries++;
//    gl_pSerPort->printf("try %d of 100 failed\n", sensor_tries);
//  }
//
//  if (sensor_tries >= 100)
//  {
//    gl_pSerPort->printf("one or more distance sensors failed to respond - quitting!\n");
//    while (true)
//    {
//      CheckForUserInput();
//    }
//  }
//
//  gl_pSerPort->printf("VL53L0X Teensy Ready at %lu\n\n", millis());
//}
//
////01/24/21 revised to implement error reporting
//bool GetRequestedVL53l0xValues(VL53L0X_REQUEST which)
//{
//  //Purpose: Obtain VL53L0X ToF sensor data from Teensy sensor handler
//  //Inputs: 
//  //	which = VL53L0X_REQUEST value denoting which combination of value to retrieve
//  //		VL53L0X_CENTERS_ONLY -> Just the left/right center sensor values
//  //		VL53L0X_RIGHT -> All three right sensor values, in front/center/rear order
//  //		VL53L0X_LEFT -> All three left sensor values, in front/center/rear order
//  //		VL53L0X_ALL -> All seven sensor values, in left/right front/center/rear/rear order
//  //		VL53L0X_REAR_ONLY -> added 10/24/20 Just the rear sensor reading
//
//  //Outputs: 
//  //	Requested sensor values, obtained via I2C from the VL53L0X sensor handler
//  //	Returns TRUE if data retrieval successful, otherwise FALSE
//  //Plan:
//  //	Step1: Send request to VL53L0X handler
//  //	Step2: get the requested data
//  //Notes:
//  // Copied from FourWD_WallE2_V4.ino's IsIRBeamAvail() and adapted
//    // 08/05/20 added a VL53L0X_ALL request type
//    // 01/24/21 added error detection/reporting
//    // 01/16/22 rev to use I2C_Anything1 functions for Wire1
//    // 06/30/22   RightSteeringVal = (RF_Dist_mm - RR_Dist_mm) /100.f; //rev 06/21/20 see PPalace post
//    //            LeftSteeringVal = (LF_Dist_mm - LR_Dist_mm) /100.f; //rev 06/21/20 see PPalace post
//    // 02/16/23 limit L/R/Rear measurements jump from about 110-120 to 785.  Limit to MAX_LR_DISTANCE_CM (200cm).
//
//  //Step1: Send request to VL53L0X handler
//  //DEBUG!!
//    //gl_pSerPort->printf("Sending %d to slave\n", which);
//  //DEBUG!!
//
//  //gl_pSerPort->printf("In GetRequestedVL53l0xValues(%d)\n", (int)which);
//
//  //03/01/22 rev to cvt all readings to cm
//  uint16_t Lidar_RightFrontMM = 0;
//  uint16_t Lidar_RightCenterMM = 0;
//  uint16_t Lidar_RightRearMM = 0;
//  uint16_t Lidar_RearMM = 0;
//
//  uint16_t Lidar_LeftFrontMM = 0;
//  uint16_t Lidar_LeftCenterMM = 0;
//  uint16_t Lidar_LeftRearMM = 0;
//
//
//  Wire1.beginTransmission(VL53L0X_I2C_SLAVE_ADDRESS);
//  I2C_writeAnything((uint8_t)which, &Wire1);
//  Wire1.endTransmission();
//
//
//  //Step2: get the requested data
//  int readResult = 0;
//  int data_size = 0;
//  switch (which)
//  {
//  case VL53L0X_READYCHECK: //11/10/20 added to prevent bad reads during Teensy setup()
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, (uint16_t)(sizeof(gl_bVL53L0X_TeensyReady)));
//    readResult = I2C_readAnything(gl_bVL53L0X_TeensyReady, &Wire1);
//    break;
//
//  case VL53L0X_CENTERS_ONLY:
//    //just two data values needed here
//    data_size = 2 * sizeof(uint16_t);
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, (uint16_t)(2 * sizeof(Lidar_RightCenterMM)));
//    readResult = I2C_readAnything(Lidar_RightCenterMM, &Wire1);
//    gl_RightCenterCm = Lidar_RightCenterMM / 10.0;
//
//    if (readResult > 0)
//    {
//      I2C_readAnything(Lidar_LeftCenterMM, &Wire1);
//      gl_LeftCenterCm = Lidar_LeftCenterMM / 10.0;
//    }
//
//    //DEBUG!!
//        //gl_pSerPort->printf("VL53L0X_CENTERS_ONLY case: Got LC/RC = %d, %d\n", Lidar_LeftCenterMM, Lidar_RightCenterMM);
//    //DEBUG!!
//
//    break;
//  case VL53L0X_RIGHT:
//    //four data values needed here
//    data_size = 3 * sizeof(uint16_t) + sizeof(float);
//
//    //DEBUG!!
//        //gl_pSerPort->printf("data_size = %d\n", data_size);
//    //DEBUG!!
//
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, data_size);
//    readResult = I2C_readAnything(Lidar_RightFrontMM, &Wire1);
//    gl_RightFrontCm = Lidar_RightFrontMM / 10.0;
//
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(Lidar_RightCenterMM, &Wire1);
//      gl_RightCenterCm = Lidar_RightCenterMM / 10.0;
//    }
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(Lidar_RightRearMM, &Wire1);
//      gl_RightRearCm = Lidar_RightRearMM / 10.0;
//    }
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(gl_RightSteeringVal, &Wire1);
//    }
//
//    //DEBUG!!
//        //gl_pSerPort->printf("VL53L0X_RIGHT case: Got L/C/R/S = %d, %d, %d, %3.2f\n",
//        //	gl_RightFrontCm, Lidar_RightCenterMM, Lidar_RightRearMM, ToFSteeringVal);
//    //DEBUG!!
//
//    break;
//  case VL53L0X_LEFT:
//    //four data values needed here
//    //data_size = 3 * sizeof(int) + sizeof(float);
//    data_size = 3 * sizeof(uint16_t) + sizeof(float);
//
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, data_size);
//    readResult = I2C_readAnything(Lidar_LeftFrontMM, &Wire1);
//    gl_LeftFrontCm = Lidar_LeftFrontMM / 10.0;
//
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(Lidar_LeftCenterMM, &Wire1);
//      gl_LeftCenterCm = Lidar_LeftCenterMM / 10.0;
//    }
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(Lidar_LeftRearMM, &Wire1);
//      gl_LeftRearCm = Lidar_LeftRearMM / 10.0;
//    }
//    if (readResult > 0)
//    {
//      readResult = I2C_readAnything(gl_LeftSteeringVal, &Wire1);
//    }
//
//    //DEBUG!!
//        //gl_pSerPort->printf("VL53L0X_RIGHT case: Got L/C/R/S = %d, %d, %d, %3.2f\n",
//        //	Lidar_LeftFrontMM, Lidar_LeftCenterMM, Lidar_LeftRearMM, ToFSteeringVal);
//    //DEBUG!!
//
//    break;
//  case VL53L0X_ALL: //added 08/05/20, chg to VL53L0X_ALL 10/31/20
//    //nine data values needed here - 7 ints and 2 floats
//    data_size = 7 * sizeof(uint16_t) + 2 * sizeof(float); //10/31/20 added rear distance
//
//    //gl_pSerPort->printf("In VL53L0X_ALL case with data_size = %d\n", data_size);
//
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, data_size);
//
//    //Lidar_LeftFrontMM
//    readResult = I2C_readAnything(Lidar_LeftFrontMM, &Wire1);
//    gl_LeftFrontCm = Lidar_LeftFrontMM / 10.0;
//
//    if (readResult != sizeof(Lidar_LeftFrontMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_LeftFrontMM\n");
//    }
//
//    //Lidar_LeftCenterMM
//    readResult = I2C_readAnything(Lidar_LeftCenterMM, &Wire1);
//    gl_LeftCenterCm = Lidar_LeftCenterMM / 10.0;
//
//    if (readResult != sizeof(Lidar_LeftCenterMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_LeftCenterMM\n");
//    }
//
//    //Lidar_LeftRearMM
//    readResult = I2C_readAnything(Lidar_LeftRearMM, &Wire1);
//    gl_LeftRearCm = Lidar_LeftRearMM / 10.0;
//
//    if (readResult != sizeof(Lidar_LeftRearMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_LeftRearMM\n");
//    }
//
//    //gl_LeftSteeringVal
//    readResult = I2C_readAnything(gl_LeftSteeringVal, &Wire1);
//    if (readResult != sizeof(gl_LeftSteeringVal))
//    {
//      gl_pSerPort->printf("Error reading gl_LeftSteeringVal\n");
//    }
//
//    //Lidar_RightFrontMM
//    readResult = I2C_readAnything(Lidar_RightFrontMM, &Wire1);
//    gl_RightFrontCm = Lidar_RightFrontMM / 10.0;
//
//    if (readResult != sizeof(Lidar_RightFrontMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_RightFrontMM\n");
//    }
//
//    //Lidar_RightCenterMM
//    readResult = I2C_readAnything(Lidar_RightCenterMM, &Wire1);
//    gl_RightCenterCm = Lidar_RightCenterMM / 10.0;
//
//    if (readResult != sizeof(Lidar_RightCenterMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_RightCenterMM\n");
//    }
//
//    //Lidar_RightRearMM
//    readResult = I2C_readAnything(Lidar_RightRearMM, &Wire1);
//    gl_RightRearCm = Lidar_RightRearMM / 10.0;
//
//    if (readResult != sizeof(Lidar_RightRearMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_RightRearMM\n");
//    }
//
//    //Lidar_RearMM
//    readResult = I2C_readAnything(Lidar_RearMM, &Wire1);
//    //gl_pSerPort->printf("Lidar_RearMM = %d\n", Lidar_RearMM);
//    gl_RearCm = Lidar_RearMM / 10.0;
//    if (readResult != sizeof(Lidar_RearMM))
//    {
//      gl_pSerPort->printf("Error reading Lidar_RearMM\n");
//    }
//
//    //gl_RightSteeringVal
//    readResult = I2C_readAnything(gl_RightSteeringVal, &Wire1);
//    if (readResult != sizeof(gl_RightSteeringVal))
//    {
//      gl_pSerPort->printf("Error reading gl_RightSteeringVal\n");
//    }
//
//    //gl_pSerPort->printf("%lu: VL53l0x - %d, %d, %d, %d, %d, %d, %d\n",
//    //	millis(), 
//    //	Lidar_LeftFrontMM, Lidar_LeftCenterMM, Lidar_LeftRearMM,
//    //  Lidar_RightFrontMM, Lidar_RightCenterMM, Lidar_RightRearMM,
//    //	Lidar_RearMM);
//    break; //10/31/20 bugfix
//
//  case VL53L0X_REAR_ONLY:
//    //just ONE data value needed here
//    data_size = sizeof(uint16_t);
//    Wire1.requestFrom(VL53L0X_I2C_SLAVE_ADDRESS, (uint16_t)(sizeof(Lidar_RearMM)));
//    readResult = I2C_readAnything(Lidar_RearMM, &Wire1);
//    gl_RearCm = Lidar_RearMM / 10.0;
//
//    //DEBUG!!
//        //gl_pSerPort->printf("In  VL53L0X_REAR_ONLY case with Lidar_RearMM = %d, gl_RearCm = %2.1f\n", Lidar_RearMM, gl_RearCm);
//    //DEBUG!!
//
//    break;
//
//  default:
//    break;
//  }
//  //gl_pSerPort->printf("GetRequestedVL53l0xValues(): LR/LC/LF/RR/RC/RF/R = %d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//  //  Lidar_LeftRearMM, Lidar_LeftCenterMM, Lidar_LeftFrontMM,
//  //  Lidar_RightRearMM, Lidar_RightCenterMM, gl_RightFrontCm, Lidar_RearMM);
//
//  //02/16/23 limit L/R/Rear measurements jump from about 110-120 to 785.  Limit to 200cm.
//  gl_RightFrontCm = (gl_RightFrontCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_RightFrontCm;
//  gl_RightCenterCm = (gl_RightCenterCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_RightCenterCm;
//  gl_RightRearCm = (gl_RightRearCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_RightRearCm;
//
//  gl_LeftFrontCm = (gl_LeftFrontCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_LeftFrontCm;
//  gl_LeftCenterCm = (gl_LeftCenterCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_LeftCenterCm;
//  gl_LeftRearCm = (gl_LeftRearCm > MAX_LR_DISTANCE_CM) ? MAX_LR_DISTANCE_CM : gl_LeftRearCm;
//
//  gl_RearCm = (gl_RearCm > MAX_REAR_DISTANCE_CM) ? MAX_REAR_DISTANCE_CM : gl_RearCm;
//
//  //07/10/22 constrain L/R steering values to -1 <= val <= +1. this corresponds to a front/rear delta_d of 10cm 
//  if (abs(gl_LeftSteeringVal) > 1)
//  {
//    gl_LeftSteeringVal = (gl_LeftSteeringVal < -1) ? -1 : gl_LeftSteeringVal;
//    gl_LeftSteeringVal = (gl_LeftSteeringVal > 1) ? 1 : gl_LeftSteeringVal;
//    //gl_pSerPort->printf("GetRequestedVL53L0xValues: gl_LeftSteeringVal truncated to %2.1f\n", gl_LeftSteeringVal);
//  }
//
//  if (abs(gl_RightSteeringVal) > 1)
//  {
//    gl_RightSteeringVal = (gl_RightSteeringVal < -1) ? -1 : gl_RightSteeringVal;
//    gl_RightSteeringVal = (gl_RightSteeringVal > 1) ? 1 : gl_RightSteeringVal;
//    //gl_pSerPort->printf("GetRequestedVL53L0xValues: gl_RightSteeringVal truncated to %2.1f\n", gl_RightSteeringVal);
//  }
//
//  return readResult > 0; //this is true only if all reads succeed
//}
//#pragma endregion VL53L0X_SUPPORT
//
#pragma region CHARGE_SUPPORT_FUNCTIONS
float GetAmps(int pin_number)
{
  //Purpose:  Get current in amps
  //Inputs: 
  // pin_number = integer denoting analog pin to be used for measurement
  //	VOLTAGE_TO_CURRENT_RATIO = measured voltage to current ratio
  // MAX_AD_COUNT = int denoting max A/D reading value
  // VOLTAGE_TO_CURRENT_RATIO = int denoting conversion ratio
  //Outputs:
  //	returns total robot current (chg current plus running current)
  //Notes:
  //	02/28/18 chg name from GetBattChgAmps() to GetTotalAmps()
  //  11/24/21 chg name, add pin_number param so can use for both Itot & Irun

  int reading = analogRead(pin_number); //range is 0-1023
  float volts = ((float)reading / (float)MAX_AD_COUNT) * ADC_REF_VOLTS;
  float amps = volts * VOLTAGE_TO_CURRENT_RATIO;

  //DEBUG!!
    //gl_pSerPort->printf("GetAmps(): reading, volts, amps = %d, %3.2f, %3.2f\n", 
    //  reading, volts, amps);
  //DEBUG!!

  return amps;
}

bool IsStillCharging()
{
  //Purpose:  Determine battery charge status
  //Inputs: 
  //	Battry charging current in amps from GetBattChgAmps()
  //	Battery voltage from GetBattV()
  //Outputs:
  //	returns TRUE if battery voltage is below full charge voltage threshold 
  //	AND charging current is above full charge current threshold.  Otherwise returns FALSE

  float BattV = GetBattVoltage();
  float TotI = GetAmps(TOT_CURR_PIN);
  float RunI = GetAmps(RUN_CURR_PIN);

  //DEBUG!!
    //gl_pSerPort->printf("IsStillCharging(): BattV = %2.3f, TotI = %2.3f, RunI = %2.3f\n", BattV, TotI, RunI); 

  //DEBUG!!

  return (BattV < FULL_BATT_VOLTS && TotI - RunI > FULL_BATT_CURRENT_THRESHOLD);
}

//11/2/23 chg curState param to reference so can directly update gl_bChgConnected
//bool IsChargerConnected(bool curState)
bool IsChargerConnected(bool& curState)
{
  //Purpose: Determine if robot has connected to charger
  //Inputs:  
    //curState = reference to gl_bChgConnected object
  //Outputs:
  //	true when array total <= CHG_CONNECTED_AVG_THRESHOLD
  //	false when array total >= CHG_DISCONNECTED_AVG_THRESHOLD
  //	otherwise maintains previous state
  //Notes:
  //  12/25/21 - now using AnalogReadAveraging(8) everywhere, so no need to do running avg
  //  02/15/22 - pin now connected to photoresistor aimed at TP5100 status LED.  Goes LOW when charger connects
  //  10/02/23 - chg input param from bool to bool& to synch output & gl_bChgConnected states

  //bool retStatus = curState;11/02/23 no longer used
  uint16_t  adval = (uint16_t)analogRead(CHG_CONNECT_PIN);
  //gl_pSerPort->printf("%lu: adval = %d\n",
    //millis(), adval);

  if (adval <= CHG_CONNECTED_AVG_THRESHOLD) //low means 'connected'
  {
    curState = true;
    //retStatus = true;
    //gl_pSerPort->printf("%lu: curState = %d, adval = %d, ConThresh = %d, disConThresh = %d retsatus = %d\n",
    //  millis(), curState, adval, CHG_CONNECTED_AVG_THRESHOLD, CHG_DISCONNECTED_AVG_THRESHOLD, retStatus);
  }
  else if (adval >= CHG_DISCONNECTED_AVG_THRESHOLD)
  {
    curState = false;
    //retStatus = false;
    //gl_pSerPort->printf("%lu: curState = %d, adval = %d, ConThresh = %d, disConThresh = %d retsatus = %d\n",
    //  millis(), curState, adval, CHG_CONNECTED_AVG_THRESHOLD, CHG_DISCONNECTED_AVG_THRESHOLD, retStatus);
  }

  //gl_pSerPort->printf("%lu: curState = %d, adval = %d, ConThresh = %d, disConThresh = %d chgconnect = %d\n",
  //  millis(), curState, adval, CHG_CONNECTED_AVG_THRESHOLD, CHG_DISCONNECTED_AVG_THRESHOLD, gl_bChgConnect);
  //else return previous conn/disconnect state
  //return retStatus;
  return curState;//11/2/23 now gl_bChgConnected & ret val are synched for all three cases
}

bool MonitorChargeUntilDone()
{
  //Purpose:  Monitor charging status until charge is complete
  //Inputs: startMsec = millis() at the time of the function call
  //Outputs: 
  //	returns TRUE if charging completes successfully, FALSE otherwise
  //	provides mode-specific telemetry to PC via Wixel
  //Plan:
  //	Step1: Blink charger display LEDs
  //	Step1: Get current time check for sufficiently elapsed time
  //	Step2: Get charger status signals, and echo them to display LEDs
  //	Step2: Send telemetry to PC via Serial port (Wixel)
  //	Step2: Check for end-of-charge or failure (don't know what this would be yet...)
  //Notes:
  // 03/11/17 for testing, rev to return as soon as connection dropped
  // 05/21/17 rev to xmit telemetry before loop & then delay a bit before entering loop
  // 05/21/17 abstracted status reporting code to separate function
  //  10/16/17 removed startMsec from call sig
  //	03/15/18 revised for TP5100 charger module
  //	04/01/18 rev to always stay on charge for at least MINIMUM_CHARGE_TIME_SEC sec
  //	02/24/19 rev to use new 1NA169 current sensor output
  //	02/28/19 moved ChargeTelemetryString printout here from MODE_CHARGING case
  //	01/30/21 added rear distance readout for debugging zero distance problem
  //	02/06/21 repl bChgConn with ISR-managed bChgConnect
  //	04/02/21 added code to blink LED associated with current charge level
  //  01/02/22 ported to Wall-E3

//Step1: Get current time & check for sufficient elapsed time
  int ElapsedChgTimeSec = 0;
  float ElapsedChgTimeMin = 0; //added 05/02/20

  gl_pSerPort->println(ChargingTelemStr);  //moved here from main loop MODE_CHARGING case

  bool bStillCharging = true;
  bool bChgConnect = true; //01/08/22 no longer using ISR
  EnableAllRearLEDs(false);

  while (ElapsedChgTimeSec < MINIMUM_CHARGE_TIME_SEC ||
    (bStillCharging
      && ElapsedChgTimeSec < BATT_CHG_TIMEOUT_SEC
      && bChgConnect)
    )
  {
    //04/02/21 moved to 'fast' part of loop
    float BattV = GetBattVoltage();
    float TotI = GetAmps(TOT_CURR_PIN);
    float RunI = GetAmps(RUN_CURR_PIN);
    //EnableAllRearLEDs(false);//05/22/22 bugfix - do once, not every time through
    UpdateChgStatusLEDs(BattV, bStillCharging); //updates 'fuel guage' LEDs 04/22/20 added bStillCharging to sig
    bStillCharging = IsStillCharging(); //02/24/19 - now using 1NA169 current sensor
    //bChgConnect = IsChargerConnected(bChgConnect); //01/02/22 - wasn't being checked
    bChgConnect = IsChargerConnected(gl_bChgConnect); //01/02/22 - wasn't being checked

    //05/02/20 rev to only print out 10 times/min
    if (ElapsedChgTimeSec % 6 == 0)
    {
      ElapsedChgTimeMin = (float)ElapsedChgTimeSec / 60.f;

      gl_pSerPort->printf("%3.1f\t%2.4f\t%2.4f\t%2.4f\t%2.4f\t%s\n",
        ElapsedChgTimeMin, BattV, TotI, RunI, TotI - RunI, bChgConnect ? "TRUE" : "FALSE"); //rev 02/24/19 for 1Na169 sensor
    }

    CheckForUserInput(); //added 04/02/21
    delay(1000); //one-second loop
    ElapsedChgTimeSec++;
    //bStillCharging = IsStillCharging(); //02/24/19 - now using 1NA169 current sensor
    //bChgConnect = IsChargerConnected(bChgConnect); //01/02/22 - wasn't being checked
  }

  //Step2: Check for end-of-charge or failure (don't know what this would be yet...)
    //if charging ran over time, something went wrong

  time_t t = now();
  if (!bChgConnect) //charger unplugged
  {
    Serial.printf("Charge connection dropped after %2.2f minutes at %d:%d:%d elapsed time\n", (float)(ElapsedChgTimeSec / 60.), hour(t), minute(t), second(t));
    gl_pSerPort->printf("Charge connection dropped after %2.2f minutes at %d:%d:%d elapsed time\n", (float)(ElapsedChgTimeSec / 60.), hour(t), minute(t), second(t));
    return false;
  }
  else if (ElapsedChgTimeSec < BATT_CHG_TIMEOUT_SEC)
  {
    gl_pSerPort->printf("Charging Completed Successfully in %2.2f minutes at %d:%d:%d elapsed time\n", (float)(ElapsedChgTimeSec / 60.), hour(t), minute(t), second(t));
    return true;
  }
  else
  {
    gl_pSerPort->printf("Charging timout value of %2.2f minutes expired at\n", (float)(BATT_CHG_TIMEOUT_SEC / 60.), hour(t), minute(t), second(t));
    return false;
  }
}

float GetBattVoltage()
{
  //02/18/17 get corrected battery voltage.  Voltage reading is 1/3 actual Vbatt value
  int analog_batt_reading = analogRead(BATT_MON_PIN);//analogReadAveraging(8) in setup() does internal averaging
  float calc_volts = ZENER_VOLTAGE_OFFSET + ADC_REF_VOLTS * (float)analog_batt_reading / (float)MAX_AD_COUNT;

  //DEBUG!!
  //for(int i = 0; i < 8;i++)
  //{
  //  int analog_batt_reading = analogRead(BATT_MON_PIN);//analogReadAveraging(8) in setup() does internal averaging
  //  float calc_volts = ZENER_VOLTAGE_OFFSET + ADC_REF_VOLTS * (float)analog_batt_reading / (float)MAX_AD_COUNT;
  //  gl_pSerPort->printf("a/d = %d, calc = %2.2f\n", analog_batt_reading,calc_volts);
  //  delay(100);
  //}
  //DEBUG!!
  return calc_volts;
}

//06/12/23 added for more reliable 'dead battery' detection
bool IsDeadBattery()
{
  //Purpose: prevent spurious dead battery detections
  //Inputs: 
  // gl_NumDeadBattDets = integer denoting the number of previous dead batt detections
  // MAX_DEAD_BATT_DETS = const integer denoting min number of dead batt detects before TRUE return
  //Outputs: returns TRUE if gl_NumDeadBattDets > MAX_DEAD_BATT_DETS.  Otherwise returns FALSE
  //Plan:
  //  Step1: Get current battery voltage
  //  Step2: if current battV < DEAD_BATT_THRESH_VOLTS, increment gl_NumDeadBattDets
  //  Step3: if gl_NumDeadBattDets > MAX_DEAD_BATT_DETS return TRUE.  Otherwise return FALSE

//Step1: Get current battery voltage
  float battV = GetBattVoltage();
  bool result = false;

  //Step2: if current battV < DEAD_BATT_THRESH_VOLTS, increment gl_NumDeadBattDets
  if (battV < DEAD_BATT_THRESH_VOLTS)
  {
    gl_NumDeadBattDets++;
    gl_pSerPort->printf("IsDeadBattery(): gl_NumDeadBattDets incremented - now %d\n", gl_NumDeadBattDets);
  }

  //Step3: if gl_NumDeadBattDets > MAX_DEAD_BATT_DETS return TRUE.  Otherwise return FALSE
  if (gl_NumDeadBattDets > MAX_DEAD_BATT_DETS)
  {
    result = true;
    gl_NumDeadBattDets = 0;//reset counter for next time (shouldn't *be* a next time, but...)
    gl_pSerPort->printf("Dead Battery detected with gl_NumDeadBattDets = %d\n", gl_NumDeadBattDets);
  }

  return result;
}

//bool ExecDisconManeuver()
//{
//  //Purpose:  Disconnect from charging station
//  //Inputs: Call from Charging Mode case block
//  //Outputs: Robot disconnects from charging station, backs up, and turns 90 away from near wall
//  //Plan:
//  //	Step1: Turn OFF c harger status LEDs (added 04/28/17)
//  //	Step2: Determine which side wall is closer
//  //	Step3: Back straight up for long enough to clear side rails
//  //	Step4: Turn 90 away from near side wall
//  //Notes:
//  //	02/15/18 rev to use full speed to disengage, and new rolling turn routines
//  //	03/27/18 rev for TP5100 charging module
//  //  11/11/23 rev to clear 3-element IRBeam average array aIRHOMINGVALTOTALS
//
//  float batv = GetBattVoltage();
//  gl_pSerPort->printf("in ExecDisconManeuver() with BattV = %2.4f\n", batv);
//
//
//  //Step1: Turn OFF charger status LEDs (added 04/28/17)
//    //chg status LEDs are all enabled via a LOW digital output
//    //03/15/18 rev for TP5100 
//  EnableAllRearLEDs(false);
//
//  //Step2: Determine which side wall is closer.  Ping sensors on 2nd deck can see over charger side rails
//  //11/10/23 revised to use UpdateAllDistances() & gl_pSerPort->printf()
//  UpdateAllDistances();
//  gl_pSerPort->printf("In ExecDisconManeuver() with left/right distances = %2.1f/%2.1f\n", gl_LeftCenterCm, gl_RightCenterCm);
//  //int leftdist = GetAvgLeftDistCm();
//  //int rightdist = GetAvgRightDistCm();
//  //Serial.print("leftdist = "); Serial.print(leftdist); Serial.print(", ");
//  //Serial.print("rightdist = "); Serial.println(rightdist);
//
//  //Step3: Back straight up for long enough to clear side rails
//#ifndef NO_VL53L0X
//  //12/21/23 MoveToDesiredFrontDistCm() returns FALSE if robot encounters 'mirrored wall' prob
//  //MoveToDesiredFrontDistCm(70); //70cm is plenty to clear the guide rails
//  if (!MoveToDesiredFrontDistCm(70))
//  {
//    gl_pSerPort->printf("\tExecDisconManeuver(): MoveToDesiredFrontDistCm() failed!  Mirrored Wall problem? - Quitting!\n");
//    YellForHelp();
//  }
//#endif
//
//  StopBothMotors();
//  delay(1000);
//
//  //Step4: Turn around and go the other way
//  //SpinTurn(true, 180, 30); //slightly higher rate than default 20dps
//  //SpinTurn(leftdist > rightdist, 180, 30); //slightly higher rate than default 20dps
//  SpinTurn(gl_LeftCenterCm > gl_RightCenterCm, 180); //use default turn rate
//
//  InitIRHomingAverageArray(); //added 11/11/23 to prevent inadvertent IR beam re-detection
//
//  return true; //can't think of anything else at the moment
//}

long GetBatRunDurationSec()
{
  return 1; //dummy for now
}

void UpdateChgStatusLEDs(float battv, bool bStillCharging) //04/22/20 added bStillCharging to sig
{
  //Purpose: Update new 'fuel guage' LED status to show very rough approximation of battery charge level
  //Inputs:
  //	battv = battery voltage measurement from Mega A/D
  //	bChg = bool that is LOW while charging, HIGH when finished
  //	bFin = bool that is HIGH while charging, LOW when finished
  //	aBattVtoPct = table of battery voltage ranges vs pct charge
  //	
  //Outputs:
  //	Chg, 20-40-60-80%, and FIN LEDs illuminated as appropriate
  //Plan:
  //	Step1: turn all LEDs OFF
  //	Step2: Illuminate appropriate LEDs
  //Notes:
  //	04/22/20 added bStillCharging to sig to eliminate unneccessary call to IsStillCharging()
  //	04/01/21 rev to blink highest charge level LED
  //  05/21/22 c/o step 1
  //  06/16/22 revised to handle LED enables properly

  //12/16/20 added for debugging
  //gl_pSerPort->printf("UpdateChgStatusLEDs(%2.2f, %s)\n", battv, bStillCharging ? "TRUE" : "FALSE");

  //Step1: turn all LEDs OFF
  //EnableAllRearLEDs(false); //turns them all OFF //commented out 05/21/22

//Step2: illuminate appropriate LEDs
  //04/01/21 rev to enclose all update blocks inside if (bStillCharging) block
  if (bStillCharging) //04/22/20 rev to pass this in as calling parameter 
  {
    digitalWrite(CHG_CONNECT_LED_PIN, LOW);

    //04/01/21 rev to blink top level
    if (battv < _20PCT_BATT_VOLTS) //6.48V with 6V dead batt voltage
    {
      //gl_pSerPort->printf("%lu: In < 20%% section with BattV = %2.4f\n", millis(), battv);
      digitalToggle(_20PCT_LED_PIN); //blink
      digitalWrite(_40PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(_60PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(_80PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn OFF
    }
    else if (battv >= _20PCT_BATT_VOLTS && battv < _40PCT_BATT_VOLTS) //6.48V - 6.96V
    {
      //gl_pSerPort->printf("%lu: In 20%% section with BattV = %2.4f\n", millis(), battv);
      digitalWrite(_20PCT_LED_PIN, LOW); //on solid
      digitalToggle(_40PCT_LED_PIN); //blink
      digitalWrite(_60PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(_80PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn OFF
    }
    else if (battv >= _40PCT_BATT_VOLTS && battv < _60PCT_BATT_VOLTS) //6.96V - 7.44V
    {
      //gl_pSerPort->printf("In > 40%% section with BattV = %2.4f\n", battv);
      digitalWrite(_20PCT_LED_PIN, LOW); //solid
      digitalWrite(_40PCT_LED_PIN, LOW); //solid
      digitalToggle(_60PCT_LED_PIN); //blink
      digitalWrite(_80PCT_LED_PIN, HIGH); //turn OFF
      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn OFF
    }

    else if (battv >= _60PCT_BATT_VOLTS && battv < _80PCT_BATT_VOLTS) //7.44V - 7.92V
    {
      //gl_pSerPort->printf("In > 60%% section with BattV = %2.4f\n", battv);
      digitalWrite(_20PCT_LED_PIN, LOW); //solid
      digitalWrite(_40PCT_LED_PIN, LOW); //solid
      digitalWrite(_60PCT_LED_PIN, LOW); //solid
      digitalToggle(_80PCT_LED_PIN); //blink
      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn OFF
    }

    else if (battv >= _80PCT_BATT_VOLTS && battv < _90PCT_BATT_VOLTS) //7.92V with 6V dead batt voltage
    {
      //gl_pSerPort->printf("In > 80%% section with BattV = %2.4f\n", battv);
      digitalWrite(_20PCT_LED_PIN, LOW); //solid
      digitalWrite(_40PCT_LED_PIN, LOW); //solid
      digitalWrite(_60PCT_LED_PIN, LOW); //solid
      digitalWrite(_80PCT_LED_PIN, LOW); //solid
      digitalToggle(CHG_FIN_LED_PIN); //blink
    }

    else if (battv >= _90PCT_BATT_VOLTS && battv < FULL_BATT_VOLTS) //8.16V with 6V dead batt voltage
    {
      //gl_pSerPort->printf("In > 90%% section with BattV = %2.4f\n", battv);
      digitalWrite(_20PCT_LED_PIN, LOW); //solid
      digitalWrite(_40PCT_LED_PIN, LOW); //solid
      digitalWrite(_60PCT_LED_PIN, LOW); //solid
      digitalWrite(_80PCT_LED_PIN, LOW); //solid
      digitalWrite(CHG_FIN_LED_PIN, LOW); //solid
      gl_pSerPort->printf("UpdateChgStatusLEDs: digitalWrite(CHG_FIN_LED_PIN, LOW)\n");
    }
    else
    {
      gl_pSerPort->printf("In indeterminate section with BattV = %2.4f\n", battv);
    }
  }
}
#pragma endregion CHARGE_SUPPORT_FUNCTIONS
//
#pragma region MISCELLANEOUS
//11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary
bool CheckForUserInput()
{
  //Notes:
  //  11/21/21 rev to accomodate cmd input from either serial, and output to both
  //  12/25/21 another try at using pointer for Serial object
  //  09/19/22 Serial object is now global - no need to figure it out here
  //  09/25/22 remove if (incomingByte != 0), extend if (gl_pSerPort->available() > 0) to entire function
  //  09/25/22 chg return type from void to char so 'auto' (Aa) exit can be detected
  //  09/27/22 back to void type - 'Aa' now soft-reboots the processor
  //  09/27/22 now this function just grabs character & calls CheckForUserInput(char in_char)
  //  11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary
  //  08/07/26 rev to check for input on any serial port

  //DEBUG!!
  //gl_pSerPort->printf("%lu: In CheckForUserInput() with gl_pSerPort->available() = %s\n",
  //  (uint32_t)gl_ElapsedRunMillisec, gl_pSerPort->available() > 0?"TRUE":"FALSE");

  const int bufflen = 3;
  char buff[bufflen];
  memset(buff, 0, bufflen);
  byte incomingByte = 0; //moved here 11/21/21

  bool retval = true; //11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary


  //  08/07/26 rev to check for input on any serial port
  //if (gl_pSerPort->available() > 0)
  if(Serial.available() > 0)
  {
    // read the incoming byte://05/08/23 this relies on having 'Both CR & LF' 
    // enabled at the bottom of the serial port window
    //gl_pSerPort->readBytesUntil('\n', buff, sizeof(buff));
    Serial.readBytesUntil('\n', buff, sizeof(buff));
    incomingByte = buff[0];

    // say what you got:
    Serial.printf("in CheckForUserInput got %c from Serial\n", incomingByte);

    //09/27/22 now just call CheckForUserInput(incomingByte)
    retval = CheckForUserInput(incomingByte);
  }
  else if (Serial1.available() > 0)
  {
    // read the incoming byte://05/08/23 this relies on having 'Both CR & LF' 
    // enabled at the bottom of the serial port window
    //gl_pSerPort->readBytesUntil('\n', buff, sizeof(buff));
    Serial1.readBytesUntil('\n', buff, sizeof(buff));
    incomingByte = buff[0];

    // say what you got:
    //08/07/26 rev to output debug info on Serial (USB port)
    Serial.printf("in CheckForUserInput just before call to CheckForUserInput(incomingByte)\n");

    //09/27/22 now just call CheckForUserInput(incomingByte)
    retval = CheckForUserInput(incomingByte);
  }

  return retval;//11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary 
}

//09/17/22 added for use in situations where calling program manages serial input
//void CheckForUserInput(char in_char)
bool CheckForUserInput(char in_char)//11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary
{
  //Purpose: Check for user override
  //Inputs: in_char = char object representing user override input
  //Outputs: override actions taken.  Returns if 'auto' mode selected
  //Notes:
  //  09/19/22 copied from CheckUserInput() and modified to accept char argument 
  //  09/28/22 'Aa' now causes a processor reboot
  //  11/04/23 '*' now causes fcn to return FALSE

  const int bufflen = 3;
  char buff[bufflen];
  memset(buff, 0, bufflen);
  byte incomingByte = 0; //moved here 11/21/21
  bool retval = true; //11/04/23 chg to bool ret value so can use to exit calling while() loop if necessary

  buff[0] = in_char; //need to to this as %s only works with char[]
  //gl_pSerPort->printf("In CheckForUserInput(%s)\n",buff);
  if (in_char != 0)
  {
    //gl_pSerPort->printf("%lu: In CheckForUserInput() just before switch() with in_char = %c\n", (uint32_t)gl_ElapsedRunMillisec, in_char);
    switch (in_char)
    {
    case 0x55: //ASCII 'U'
    case 0x75: //ASCII 'u'
#pragma region FIRMWARE_UPDATE_MAIN
            StopBothMotors();
            gl_pSerPort->printf(F("Start Program Update - Send new HEX file!"));
      
            //09/20/21 copied from FlasherX - loop()
            if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
            {
              gl_pSerPort->printf("unable to create buffer\n"); Serial.flush();
      
              for (;;) {}
            }
      
            gl_pSerPort->printf("buffer = %1luK %s (%08lX - %08lX)\n",
              buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
              buffer_addr, buffer_addr + buffer_size);
      
            //09/20/21 clear the serial buffer
            while (gl_pSerPort->available())
            {
              gl_pSerPort->read();
            }
      
            // receive hex file via serial, write new firmware to flash, clean up, reboot
            //update_firmware(&Serial1, buffer_addr, buffer_size); // no return if success
            update_firmware(&Serial1, &Serial1, buffer_addr, buffer_size); // no return if success
      
            // return from update_firmware() means error or user abort, so clean up and
            // reboot to ensure that static vars get boot-up initialized before retry
            gl_pSerPort->printf("erase FLASH buffer / free RAM buffer...\n");
            firmware_buffer_free(buffer_addr, buffer_size);
            Serial1.flush();
            REBOOT;
#pragma endregion FIRMWARE_UPDATE_MAIN //doesn't return
      break;
    case 0x43: //ASCII 'C'
    case 0x63: //ASCII 'c'
#pragma region COMMAND_MODE
      gl_pSerPort->printf("%lu: At top of COMMAND_MODE Case\n", (uint32_t)gl_ElapsedRunMillisec);
      gl_pSerPort->printf(F("ENTERING COMMAND MODE:\n"));
      gl_pSerPort->printf(F("0 = 180 deg CCW Turn\n"));
      gl_pSerPort->printf(F("1 = 180 deg CW Turn\n"));
      //gl_pSerPort->println(F("A = Back to Auto Mode"));
      gl_pSerPort->println(F("A = Abort - Reboots Processor"));//rev 09/27/22
      //gl_pSerPort->printf(F("S = Stop\n"));//c/o 05/08/23
      gl_pSerPort->printf(F("/ = Forward\n"));
      gl_pSerPort->printf(F(".(dot) = Reverse\n"));
      gl_pSerPort->printf(F("* = Exit ChkForUserInput()\n"));
      gl_pSerPort->printf(F("\n"));
      gl_pSerPort->printf(F("       Faster\n"));
      gl_pSerPort->printf(F("\t8\n"));
      gl_pSerPort->printf(F("Left 4\t5  6 Right\n"));
      gl_pSerPort->printf(F("\t2\n"));
      gl_pSerPort->printf(F("       Slower\n"));

      //gl_pSerPort->printf("%lu: Just before StopBothMotors()\n", (uint32_t)gl_ElapsedRunMillisec);
      StopBothMotors();
      int speed = 0;
      //bool bAutoMode = false;
      //gl_pSerPort->printf("Just before while (gl_pSerPort->available())\n");

      //int res = gl_pSerPort->available();
      //int res = Serial.available();
      //gl_pSerPort->printf("gl_pSerPort->available() returned %d\n", res);


      while (gl_pSerPort->available())
      {
        incomingByte = gl_pSerPort->read();
        //gl_pSerPort->printf("%lu: I removed 0X%X from Serial1\n",millis(), incomingByte);
        //delay(200);
      }

      //gl_pSerPort->printf("Just after while (gl_pSerPort->available())\n");
      incomingByte = 0;
      bool bDone = false;//added 11/04/23 to implement user-controlled exit

      //while (1) //09/27/22 removed bAutoMode check - now 'Aa' reboots processor
      while (!bDone) //09/27/22 removed bAutoMode check - now 'Aa' reboots processor
      {
        //12/25/21 now using Stream* for serial
        if (gl_pSerPort->available() > 0)
        {
          // read the incoming bytes:
          gl_pSerPort->readBytesUntil('\n', buff, sizeof(buff));
          incomingByte = buff[0];

          // say what you got:
          gl_pSerPort->printf("I received %s\n", buff);

          //clear out any remaining chars
          while (gl_pSerPort->available())
          {
            gl_pSerPort->read();
            //gl_pSerPort->printf("I removed 0X%X from Serial1\n", incomingByte);
            //gl_pSerPort->printf("%lu: I removed 0X%X from Serial1\n", (uint32_t)gl_ElapsedRunMillisec, incomingByte);
          }
        }

        //08/07/26 added to allow Pi5 to inject characters
        else if (Serial1.available() > 0)
        {
          // read the incoming bytes:
          Serial1.readBytesUntil('\n', buff, sizeof(buff));
          incomingByte = buff[0];

          // say what you got:
          Serial.printf("I received %s on Serial1\n", buff);

          //clear out any remaining chars
          while (Serial1.available())
          {
            Serial1.read();
          }
        }


        //11/21/21 incomingByte can come from either serial input
        if (incomingByte != 0)
        {
          //gl_pSerPort->printf("%lu: Top of Switch Statement\n", (uint32_t)gl_ElapsedRunMillisec);
          switch (incomingByte)
          {
          case 0x30: //Dec '0'
            gl_pSerPort->printf(F("CCW 180 deg Turn\n"));
            SpinTurn(true, 180, 90);
            MoveAhead(speed, speed);
            break;
          case 0x31: //Dec '1'
            gl_pSerPort->printf(F("CW 180 deg Turn\n"));
            SpinTurn(false, 180, 45);
            break;
          case 0x34: //Turn left 10 deg and keep moving
            //gl_pSerPort->printf("%lu: In CCW 10 deg turn block\n", (uint32_t)gl_ElapsedRunMillisec);
            gl_pSerPort->printf(F("CCW 10 deg Turn\n"));
            SpinTurn(true, 10, 30);

            if (gl_bIsForwardDir)
            {
              MoveAhead(speed, speed);
            }
            else
            {
              MoveReverse(speed, speed);
            }
            break;
          case 0x36: //Turn right 10 deg and keep moving
            //gl_pSerPort->print("CW 10 deg Turn\n");
            gl_pSerPort->printf(F("CW 10 deg Turn\n"));
            SpinTurn(false, 10, 30);

            //added 05/03/20
            if (gl_bIsForwardDir)
            {
              MoveAhead(speed, speed);
            }
            else
            {
              MoveReverse(speed, speed);
            }
            break;
          case 0x38: //Speed up 
            speed += 50;
            speed = (speed >= MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : speed;
            //gl_pSerPort->printf("Speeding up: speed now %d\n", speed);
            if (gl_bIsForwardDir)
            {
              MoveAhead(speed, speed);
            }
            else
            {
              MoveReverse(speed, speed);
            }
            break;
          case 0x32: //Slow down 
            speed -= 50;
            speed = (speed < 0) ? 0 : speed;
            //gl_pSerPort->printf("Slowing down: speed now %d\n", speed);
            if (gl_bIsForwardDir)
            {
              MoveAhead(speed, speed);
            }
            else
            {
              MoveReverse(speed, speed);
            }
            break;
          case 0x35: //05/07/20 changed to use '5' vs 'S'
            //gl_pSerPort->println(F("Stopping Motors!"));
            StopBothMotors();
            speed = 0;
            break;
            //case 0x41: //ASCII 'A'
            //case 0x61: //ASCII 'a'
            //  StopBothMotors();

            //  //09/27/22 rev to execute soft reboot
            //  gl_pSerPort->printf(F("Received 'A' or 'a' - Rebooting in 1 second...\n"));
            //  delay(1000);
            //  REBOOT;
            //  break;
          case 0x2E: //ASCII '.' (dot)
            gl_pSerPort->printf(F("Setting both motors to reverse\n"));
            gl_bIsForwardDir = false;
            MoveReverse(speed, speed);
            break;
            //case 0x46: //ASCII 'F'
            //case 0x66: //ASCII 'f'
          case 0x2F: //ASCII '/'
            gl_pSerPort->printf(F("Setting both motors to forward\n"));
            gl_bIsForwardDir = true;
            MoveAhead(speed, speed);
#pragma endregion COMMAND_MODE //only returns for 'a' (auto) input
            break;

            //01/11/22 copied here from main switch statement to allow firmware updates
            //even when in 'command' mode
          case 0x55: //ASCII 'U'
          case 0x75: //ASCII 'u'
#pragma region FIRMWARE UPDATE
            StopBothMotors();
            gl_pSerPort->printf(F("Start Program Update - Send new HEX file!\n"));

            //09/20/21 copied from FlasherX - loop()
            if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
            {
              gl_pSerPort->printf("unable to create buffer\n"); Serial.flush();

              for (;;) {}
            }

            gl_pSerPort->printf("buffer = %1luK %s (%08lX - %08lX)\n",
              buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
              buffer_addr, buffer_addr + buffer_size);

            //09/20/21 clear the serial buffer
            while (gl_pSerPort->available())
            {
              gl_pSerPort->read();
            }

            // receive hex file via serial, write new firmware to flash, clean up, reboot
            //update_firmware(&Serial1, buffer_addr, buffer_size); // no return if success
            update_firmware(&Serial1, &Serial1, buffer_addr, buffer_size); // no return if success


            // return from update_firmware() means error or user abort, so clean up and
            // reboot to ensure that static vars get boot-up initialized before retry
            gl_pSerPort->printf("erase FLASH buffer / free RAM buffer...\n");
            firmware_buffer_free(buffer_addr, buffer_size);
            Serial1.flush();
            REBOOT;
#pragma endregion FIRMWARE UPDATE  //doesn't return
            break;
          case 0x2A: //ASCII '*' //11/04/23 added to force FALSE return
            gl_pSerPort->printf(F("Exiting ChkForUserInput()\n"));
            StopBothMotors();
            bDone = true;
            retval = false;
            break;
          default:
            gl_pSerPort->printf(F("In Default Case: Stopping Motors!\n"));
            StopBothMotors();
          }
          incomingByte = 0;
        }
      }
      gl_pSerPort->printf(F("Exited  'while (!bDone)'\n"));
    }
  }
  //gl_pSerPort->printf("Returning from 'CheckForUserInput()' with retval = %s\n", retval? "TRUE" : "FALSE");
  return retval;
}

void EnableAllRearLEDs(bool bEnable)
{
  //Purpose:  Turns all 4 LEDs ON or OFF (LOW is ON)
  //Provenance: Created 05/02/17 gfp

  if (bEnable)
  {
    //gl_pSerPort->printf("EnableAllRearLEDs(TRUE) - setting all LED lines LOW\n");
    digitalWrite(CHG_CONNECT_LED_PIN, LOW);
    digitalWrite(_20PCT_LED_PIN, LOW);
    digitalWrite(_40PCT_LED_PIN, LOW);
    digitalWrite(_60PCT_LED_PIN, LOW);
    digitalWrite(_80PCT_LED_PIN, LOW);
    digitalWrite(CHG_FIN_LED_PIN, LOW);
  }
  else
  {
    //gl_pSerPort->printf("EnableAllRearLEDs(FALSE) - setting all LED lines HIGH\n");
    digitalWrite(CHG_CONNECT_LED_PIN, HIGH);
    digitalWrite(_20PCT_LED_PIN, HIGH);
    digitalWrite(_40PCT_LED_PIN, HIGH);
    digitalWrite(_60PCT_LED_PIN, HIGH);
    digitalWrite(_80PCT_LED_PIN, HIGH);
    digitalWrite(CHG_FIN_LED_PIN, HIGH);
  }
}
//
//void YellForHelp()
//{
//  //Purpose: last-ditch effort to preserve the battery.  All non-essential loads are shut down
//  //		   and an visual/audible SOS is sounded forever
//  //Inputs: Call from OpMode case switch when GetBattVoltage() returns a below-threshold value
//  //Outputs: 
//  //	All wheel motors stopped
//  //	All rear panel LED's turned OFF
//  //	PWM 'SOS' tones on SOS_PWM_PIN
//  //Plan:
//  //	Step1: Turn off wheel motors
//  //	Step2: Turn off all rear panel LEDs
//  //	Step3: Send SOS to speaker, the purple rear panel LEDs, and the red laser
//  //Notes:
//  //	01/16/18 - SOS tone code copied from PWMTest.pde
//
////Step1: Turn off wheel motors
//  StopBothMotors();
//
//  //Step2: Turn off all rear panel LEDs
//  EnableAllRearLEDs(false);
//
//  //Step3: infinte loop to xmit SOS on speaker and the purple rear panel LEDs
//  while (!gl_bChgConnect)//12/16/20 rev to use ISR-generated value
//  {
//    uint16_t DOT_MS = 200;
//    uint16_t DASH_MS = 800;
//    uint16_t HIGHTONE = 1000;
//
//
//    //Send 'S'
//    for (size_t i = 0; i < 3; i++)
//    {
//      digitalWrite(RED_LASER_DIODE_PIN, HIGH); //turn laser on
//      digitalWrite(CHG_FIN_LED_PIN, LOW); //turn LED on
//      digitalWrite(CHG_CONNECT_LED_PIN, LOW); //turn LED on
//      tone(SOS_PWM_PIN, HIGHTONE, DOT_MS); //returns immediately
//      delay(DOT_MS);              //delay for LED viewing
//      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn LED off
//      digitalWrite(CHG_CONNECT_LED_PIN, HIGH); //turn LED off
//      digitalWrite(RED_LASER_DIODE_PIN, LOW); //turn laser off
//      delay(DOT_MS);              //inter-symbol spacing
//    }
//    delay(DOT_MS); //inter-letter spacing
//
//    //Send 'O'
//    for (size_t i = 0; i < 3; i++)
//    {
//      digitalWrite(RED_LASER_DIODE_PIN, HIGH); //turn laser on
//      digitalWrite(CHG_FIN_LED_PIN, LOW); //turn LED on
//      digitalWrite(CHG_CONNECT_LED_PIN, LOW); //turn LED on
//      tone(SOS_PWM_PIN, HIGHTONE, DASH_MS); //returns immediately
//      delay(DASH_MS);              //delay for LED viewing
//      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn LED off
//      digitalWrite(CHG_CONNECT_LED_PIN, HIGH); //turn LED off
//      digitalWrite(RED_LASER_DIODE_PIN, LOW); //turn laser off
//      delay(DOT_MS);              //inter-symbol spacing
//    }
//    delay(DOT_MS); //inter-letter spacing
//
//    //Send 'S'
//    for (size_t i = 0; i < 3; i++)
//    {
//      digitalWrite(RED_LASER_DIODE_PIN, HIGH); //turn laser on
//      digitalWrite(CHG_FIN_LED_PIN, LOW); //turn LED on
//      digitalWrite(CHG_CONNECT_LED_PIN, LOW); //turn LED on
//      tone(SOS_PWM_PIN, HIGHTONE, DOT_MS); //returns immediately
//      delay(DOT_MS);              //delay for LED viewing
//      digitalWrite(CHG_FIN_LED_PIN, HIGH); //turn LED off
//      digitalWrite(CHG_CONNECT_LED_PIN, HIGH); //turn LED off
//      digitalWrite(RED_LASER_DIODE_PIN, LOW); //turn laser off
//      delay(DOT_MS);              //inter-symbol spacing
//    }
//
//    delay(10 * DOT_MS);         // inter-message spacing
//
//    //UpdateAllEnvironmentParameters(); //06/12/22 added to update gl_bChgConnect
//    //CheckForUserInput(); //added 06/14/22
//  }
//}
//
//#pragma endregion MISCELLANEOUS
//
#pragma region HDG_BASED_TURN_SUPPORT
bool SpinTurn(bool b_ccw, float numDeg, float degPersec) //04/25/21 added turn-rate arg (default = TURN_RATE_TARGET_DEGPERSEC)
{
  //Purpose: Make a numDeg CW or CCW 'spin' turn
  //Inputs:
  //	b_ccw - True if turn is to be ccw, false otherwise
  //	numDeg - angle to be swept out in the turn
  //	ROLLING_TURN_MAX_SEC_PER_DEG = const used to generate timeout proportional to turn deg
  //	IMUHdgValDeg = IMU heading value updated by UpdateIMUHdgValDeg() //11/02/20 now updated in ISR
  //	degPerSec = float value denoting desired turn rate
  //Plan:
  //	Step1: Get current heading as starting point
  //	Step2: Disable TIMER5 interrupts
  //	Step3: Compute new target value & timeout value
  //	Step4: Run motors until target reached, using inline PID algorithm to control turn rate
  //	Step5: Re-enable TIMER5 interrupts
  //Notes:
  //	06/06/21 we-written to remove PID library - now uses custom 'PIDCalcs()' function
  //	06/06/21 added re-try for 180.00 return from IMU - could be bad value
  //	06/11/21 added code to correct dHdg errors due to 179/-179 transition & bad IMU values
  //	06/12/21 cleaned up & commented out debug code
  //	11/14/21 removed 'first time skip' block; added motor start before entering loop
  //  03/22/22 added code to ensure numDeg >= 0
  //  06/01/22 now using MOTOR_SPEED_HALF for max motor speed and MOTOR_SPEED_OFF for low end

  float tgt_deg;
  float timeout_sec;
  bool bDoneTurning = false;
  bool bTimedOut = false;
  bool bResult = true; //04/21/20 added so will be only one exit point

  numDeg = abs(numDeg);//  03/22/22 added to ensure numDeg >= 0
  //DEBUG!!
  gl_pSerPort->printf("In SpinTurn(%s, %2.2f, %2.2f) with PID = (%2.1f,%2.1f,%2.1f)\n",
    b_ccw == TURNDIR_CCW ? "CCW" : "CW", numDeg, degPersec,
    TurnRate_Kp, TurnRate_Ki, TurnRate_Kd);
  //DEBUG!!

  //no need to continue if the IMU isn't available
  if (!dmpReady)
  {
    Serial.printf("DMP Failure - returning FALSE\n");
    return false;
  }

  //Step1: Get current heading as starting point
    //06/06/21 it is possible for IMU to return 180.00 on failure
    //so try again.  If it really IS 180, then 
    //it will eventually time out and go on

  //08/26/21 re-wrote using 3-value array to make sure initial heading is a steady value
  UpdateIMUHdgValDeg();

  int retries = 0;
  if ((IMUHdgValDeg == 180.f || IMUHdgValDeg == 0.f) && retries < 5)
  {
    //DEBUG!!
    gl_pSerPort->printf("Got 180.00 or 0.00 exactly (%2.3f) from IMU - retrying %d...\n", IMUHdgValDeg, retries);
    //DEBUG!!
    UpdateIMUHdgValDeg();
    retries++;
    delay(100);
  }

  //Step2: Compute new target value & timeout value
  timeout_sec = 2 * numDeg / degPersec; //05/29/21 rev to use new turn rate parmeter

  //05/17/20 limit timeout_sec to 1 sec or more
  timeout_sec = (timeout_sec < 1) ? 1.f : timeout_sec;

  //12/05/19 added #define back in to manage which direction increases yaw values
#ifdef MPU6050_CCW_INCREASES_YAWVAL
  tgt_deg = b_ccw ? IMUHdgValDeg + numDeg : IMUHdgValDeg - numDeg;
#else
  tgt_deg = b_ccw ? IMUHdgValDeg - numDeg : IMUHdgValDeg + numDeg;

#endif // MPU6050_CCW_INCREASES_YAWVAL

  //correct for -180/180 transition
  if (tgt_deg < -180)
  {
    tgt_deg += 360;
  }

  //07/29/19 bugfix
  if (tgt_deg > 180)
  {
    tgt_deg -= 360;
  }

//DEBUG!!
  //gl_pSerPort->printf("SpinTurn: Init hdg = %4.2f deg, Turn = %4.2f  deg, tgt = %4.2f deg, timeout = %4.2f sec\n\n",
  //  IMUHdgValDeg, numDeg, tgt_deg, timeout_sec);
//DEBUG!!

  float curHdgMatchVal = 0;

  //09/08/18 added to bolster end-of-turn detection
  float prevHdgMatchVal = 0;
  float matchSlope = 0;

  //Step3: Run motors until target reached, using PID algorithm to control turn rate
  Prev_HdgDeg = IMUHdgValDeg; //06/10/21 synch Prev_HdgDeg & IMUHdgValDeg just before entering loop

  elapsedMillis mSecSinceTurnStart = 0;
  MsecSinceLastTurnRateUpdate = 0;
  float lastError = 0;
  float lastInput = 0;
  float lastIval = 0;
  float lastDerror = 0;
  //bool bFirstIMUHdg = true;

  ////DEBUG!!
  //gl_pSerPort->printf("Msec\tHdg\tPrvHdg\tdHdg\tRate\ttgtDPS\terr\tKp*err\tIval\tKd*Derr\tspeed\tMatch\tSlope\n");
  ////DEBUG!!

  float avgrate = 0;
  uint16_t numrates = 0;

  while (!bDoneTurning && !bTimedOut)
  {
    //11/06/20 now just loops between PID calcs
    CheckForUserInput();

    if (MsecSinceLastTurnRateUpdate >= TURN_RATE_UPDATE_INTERVAL_MSEC)
    {
      MsecSinceLastTurnRateUpdate -= TURN_RATE_UPDATE_INTERVAL_MSEC;

      UpdateIMUHdgValDeg(); //update IMUHdgValDeg

      float dHdg = IMUHdgValDeg - Prev_HdgDeg;
      if (dHdg > 180)
      {
        dHdg -= 360;
        //Serial.printf("dHdg > 180 - subtracting 360\n");
      }
      else if (dHdg < -180)
      {
        dHdg += 360;
        //Serial.printf("dHdg < -180 - adding 360\n");
      }

      //watch for turn rates that are wildly off
      float rate = abs(1000 * dHdg / TURN_RATE_UPDATE_INTERVAL_MSEC);
      avgrate += rate;
      numrates++;

      //if (rate > 3 * degPersec)
      //{
      //	//DEBUG!!
      //Serial.printf("hdg/prevhdg/dHdg/rate = %2.2f\t%2.2f\t%2.2f\t%2.2f, excessive rate - replacing with %2.2f\n", IMUHdgValDeg, Prev_HdgDeg, dHdg, rate, degPersec);
      //	//DEBUG!!
      //	rate = degPersec;
      //}

      //02/05/22 sampleTime removed from signature
      TurnRatePIDOutput = PIDCalcs(rate, degPersec, lastError, lastInput, lastIval, lastDerror, TurnRate_Kp, TurnRate_Ki, TurnRate_Kd);

      int speed = 0;

      //05/31/22 now using MOTOR_SPEED_HALF for max motor speed
      //06/01/22 and MOTOR_SPEED_OFF for low end
      speed = (TurnRatePIDOutput > MOTOR_SPEED_HALF) ? MOTOR_SPEED_HALF : (int)TurnRatePIDOutput;
      speed = (TurnRatePIDOutput <= MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : (int)TurnRatePIDOutput;

      //05/06/22 speed val always >= 0 here. 
      //05/06/22 using SetLeft/RightMotorDirAndSpeed() simpler than using RunBothMotorsBidirectional().
      SetLeftMotorDirAndSpeed(!b_ccw, speed);
      SetRightMotorDirAndSpeed(b_ccw, speed);

      //check for nearly there and all the way there
      curHdgMatchVal = GetHdgMatchVal(tgt_deg, IMUHdgValDeg);
      matchSlope = curHdgMatchVal - prevHdgMatchVal;

      ////DEBUG!!
      //gl_pSerPort->printf("%lu\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%d\t%2.2f\t%2.2f\n",
      //  millis(),
      //  IMUHdgValDeg,
      //  Prev_HdgDeg,
      //  dHdg,
      //  rate,
      //  degPersec,
      //  lastError,
      //  TurnRate_Kp * lastError,
      //  lastIval,
      //  TurnRate_Kd * lastDerror,
      //  speed,
      //  curHdgMatchVal,
      //  matchSlope);
      //gl_pSerPort->printf("%lu\t%2.2f\t%d\n",
      //  millis(),
      //  rate,
      //  speed);
      ////DEBUG!!

      Prev_HdgDeg = IMUHdgValDeg; //re-synch prev to curr hdg for next time

      //look for full match
      bDoneTurning = (curHdgMatchVal >= HDG_FULL_MATCH_VAL
        || (prevHdgMatchVal >= HDG_MIN_MATCH_VAL && matchSlope <= -0.01)); //have to use < vs <= as slope == 0 at start

      //Serial.printf("curHdgMatchVal = %2.2f, prevHdgMatchVal = %2.2f, matchslope = %2.2f, bDoneTurning = %d\n",
      //  curHdgMatchVal,
      //  prevHdgMatchVal,
      //  matchSlope,
      //  bDoneTurning);

      prevHdgMatchVal = curHdgMatchVal; //07/31/21 moved below bDoneTurning chk so can use prevHdgMatchVal vs curHdgMatchVal in slope check

      bTimedOut = (mSecSinceTurnStart > timeout_sec * 1000);

      if (bTimedOut)
      {
        //DEBUG!!
        gl_pSerPort->printf("timed out with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", IMUHdgValDeg, tgt_deg, curHdgMatchVal);
        //DEBUG!!

        bResult = false;
        break;
      }

      if (bDoneTurning)
      {
        //gl_pSerPort->printf("Completed turn with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", IMUHdgValDeg, tgt_deg, curHdgMatchVal);

        bResult = true;
        break;
      }
    }
  }

  avgrate = avgrate / numrates;

  //gl_pSerPort->printf("average turn rate = %2.1f\n", avgrate);

  StopBothMotors();
  //delay(1000); //added 04/27/21 for debug
  return bResult;
}

bool TurnToHdgDeg(float tgtdeg)
{
  //Purpose:  Turn robot to a specific heading
  //Inputs: 
  //  tgtdeg = float denoting heading target in degrees
  //Outputs:
  //  robot turns to specified heading
  //Plan:
  //  Step1: determine current heading
  //  Step2: determine number of degrees to turn to get to desired heading
  //  Step3: call SpinTurn() to turn the desired number of degrees

//Step1: determine current heading

  //Step1: Get current heading as starting point
    //06/06/21 it is possible for IMU to return 180.00 on failure
    //so try again.  If it really IS 180, then 
    //it will eventually time out and go on

  //08/26/21 re-wrote using 3-value array to make sure initial heading is a steady value
  UpdateIMUHdgValDeg();

  int retries = 0;
  if ((IMUHdgValDeg == 180.f || IMUHdgValDeg == 0.f) && retries < 5)
  {
    //DEBUG!!
    gl_pSerPort->printf("Got 180.00 or 0.00 exactly (%2.3f) from IMU - retrying %d...\n", IMUHdgValDeg, retries);
    //DEBUG!!
    UpdateIMUHdgValDeg();
    retries++;
    delay(100);
  }
  float currentDeg = UpdateIMUHdgValDeg();
  float turndeg = 0;

  //gl_pSerPort->printf("TurnToHdgDeg: currHdg = %2.1f, tgtHdg = %2.1f, retries = %d \n", currentDeg, tgtdeg, retries);

  //Step2: determine number of degrees to turn to get to desired heading
  if (currentDeg * tgtdeg >= 0) //same sign means tgtdeg on same side as currentgeg (0 to 180 or 0 to -179) for sure
  {
    //OK, current & tgt deg have same sign
#ifdef MPU6050_CCW_INCREASES_YAWVAL
    SpinTurn(tgtdeg > currentDeg, abs(tgtdeg - currentDeg));
#else
    SpinTurn(tgtdeg < currentDeg, abs(tgtdeg - currentDeg));
#endif // MPU6050_CCW_INCREASES_YAWVAL
  }
  else
  {
    //current deg has opposite sign from tgtdeg
    turndeg = currentDeg - tgtdeg;

    //correct for -180/180 transition
    if (turndeg < -180)
    {
      turndeg += 360;
    }

    //07/29/19 bugfix
    if (turndeg > 180)
    {
      turndeg -= 360;
    }


    //12/05/19 added #define back in to manage which direction increases yaw values
#ifdef MPU6050_CCW_INCREASES_YAWVAL
    SpinTurn(turndeg > 0, turndeg);
#else
    //gl_pSerPort->printf("TTHD: curr/tgt/turndeg = %2.1f/%2.1f/%2.1f\n", currentDeg, tgtdeg, turndeg);
    SpinTurn(turndeg > 0, turndeg);
#endif // MPU6050_CCW_INCREASES_YAWVAL
  }
  return true;
}

//02/05/22 rem sampleTime from sig - now calling fcn is resp for maintaining consistent timing interval
float PIDCalcs(float input, float setpoint, float& lastError, float& lastInput, float& lastIval, float& lastDerror, float Kp, float Ki, float Kd)
{
  //Purpose:  Encapsulate PID algorithm so can get rid of PID library. Library too cumbersome and won't synch with TIMER5 ISR
  //Inputs:
  //	input = float denoting current input value (turn rate, speed, whatever)
  //	setpoint = float denoting desired setpoint in same units as input
  //	lastError = ref to float denoting error saved from prev calc
  //	lastInput = ref to float denoting input saved from prev calc
  //	Kp/Ki/Kd = floats denoting PID values to be used for calcs
  //	Output = ref to float denoting output from calc
  //  Notes:
  //    01/13/22 sampleTime input parameter is never used.  This is OK as long as PIDCalcs
  //      is called at regular intervals.
  //    02/05/22 sampleTime input parameter removed from signature
  //  07/22/23 added globals to allow PIDCalcs results printout when PID_TUNING_TELEMETRY_ONLY #define'd

  float error = setpoint - input;
  float dErr = (error - lastError);

  lastIval += (Ki * error);

  //07/22/23 added for PID tuning printouts
  gl_PIDLastIval = lastIval;
  gl_PID_Kp_error = Kp * error;
  gl_PID_Kd_dErr = Kd * dErr;


  //gl_pSerPort->printf("PIDCalcs: error/lastIval/dErr/kp/ki/kd = %2.2f/%2.2f/%2.2f/%2.2f/%2.2f/%2.2f\n", error, lastIval, dErr,
  //	Kp,Ki,Kd);

  /*Compute PID Output*/
  //07/22/23 added to allow PID tuning outpur controlled by #define PID_TUNING_TELEMETRY_ONLY

  //11/16/21 rev to subtract differential term rather than add.
  float output = Kp * error + lastIval - Kd * dErr;
  gl_PID_Outval = output;//07/22/23 added for PID tuning printouts

  //gl_pSerPort->printf("Kp * error = %2.2f, lastIval = %2.2f, Kd * dErr = %2.2f, output = %2.2f\n", 
  //  Kp * error, lastIval, Kd * dErr, output);

  /*Remember some variables for next time*/
  lastError = error;
  lastInput = input;
  lastDerror = dErr;

  return output; //added 09/03/21
}

float GetHdgMatchVal(float tgt_deg, float cur_deg)
{
  //Purpose:  Compute the match ratio between two compass headings
  //Inputs:
  //	tgt_deg = float representing target heading in +/-180 range
  //	IMUHdgValDeg = float representing sensor yaw value in +/-180 deg range
  //Outputs:
  //	returns result of 1 - abs(Tgt_deg - Hdg_deg)/180, all angles in 0-360 deg range
  //Plan:
  //	Step1: convert both inputs to 0-360 deg range
  //	Step2: compute match ratio
  //Notes:
  //	formula from https://gis.stackexchange.com/questions/129954/comparing-compass-bearings

//Step1: convert both inputs to 0-360 deg range
  float tgthdg = (tgt_deg < 0) ? tgt_deg + 360 : tgt_deg;
  float curhdg = (cur_deg < 0) ? cur_deg + 360 : cur_deg;

  //Step2: compute match ratio
  float match_ratio = 1 - abs(tgthdg - curhdg) / 180;

  //DEBUG!!
    //gl_pSerPort->printf("tgt\tcur\tmatch = %4.2f\t%4.2f\t%1.3f\n", tgthdg, curhdg, match_ratio);
  //DEBUG!!
  return abs(match_ratio);
}

float UpdateIMUHdgValDeg()
{
  //Purpose: Get latest yaw (heading) value from IMU
  //Inputs: None.  This function should only be called after mpu.dmpPacketAvailable() returns TRUE
  //Outputs: 
  //	returns true if successful, otherwise false
  //	IMUHdgValDeg updated on success
  //Plan:
  //Step1: check for overflow and reset the FIFO if it occurs. In this case, wait for new packet
  //Step2: read all available packets to get to latest data
  //Step3: update IMUHdgValDeg with latest value
  //Notes:
  //	10/08/19 changed return type to boolean
  //	10/08/19 no longer need mpuIntStatus
  //	10/21/19 completely rewritten to use Homer's algorithm
  //	05/05/20 changed return type to float vs bool.
  //  06/13/23 added code to update gl_HdgHistoryArray for 'spinning' condx detection

  int flag = GetCurrentFIFOPacket(fifoBuffer, packetSize, MAX_GETPACKET_LOOPS); //get the latest mpu packet

  if (flag != 0) //0 = error exit, 1 = normal exit, 2 = recovered from an overflow
  {
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //compute the yaw value
    IMUHdgValDeg = ypr[0] * 180 / M_PI;
  }

  //06/13/23 added to update gl_HdgHistoryArray for 'spinning' condx detection
  //all array entries bumped down one, with most recent value at i = HEADING_HISTORY_ARRAY_SIZE-1
  for (uint16_t i = 1; i < HEADING_HISTORY_ARRAY_SIZE; i++)
  {
    gl_HdgHistoryArray[i - 1] = gl_HdgHistoryArray[i];
    //gl_pSerPort->printf("gl_HdgHistoryArray[%d] = %2.2f\n", i, gl_HdgHistoryArray[i]);
  }
  gl_HdgHistoryArray[HEADING_HISTORY_ARRAY_SIZE - 1] = IMUHdgValDeg;


  return IMUHdgValDeg;//05/05/20 now returns updated value for use convenience
}

uint8_t GetCurrentFIFOPacket(uint8_t* data, uint8_t length, uint16_t max_loops)
{
  mpu.resetFIFO();
  delay(1);
  //int countloop = 0;

  fifoCount = mpu.getFIFOCount();
  GetPacketLoopCount = 0;

  //gl_pSerPort->printf("In GetCurrentFIFOPacket: before loop fifoC = %d\t", fifoCount);
  while (fifoCount < packetSize && GetPacketLoopCount < max_loops)
  {
    GetPacketLoopCount++;
    fifoCount = mpu.getFIFOCount();
    delay(2);
  }

  //gl_pSerPort->printf("In GetCurrentFIFOPacket: after loop fifoC = %d, loop count = %d\n", fifoCount, GetPacketLoopCount);

  if (GetPacketLoopCount >= max_loops)
  {
    return 0;
  }

  //if we get to here, there should be exactly one packet in the FIFO
  mpu.getFIFOBytes(data, packetSize);
  return 1;
}

bool RollingTurn(bool b_ccw, bool b_fwd, float numDeg, float Kp, float Ki, float Kd, float degPersec) //04/25/21 added turn-rate arg (default = TURN_RATE_TARGET_DEGPERSEC)
{
  //Purpose: Make a numDeg CW or CCW 'rolling' turn
  //Inputs:
  //	b_ccw - True if turn is to be ccw, false otherwise
  //	numDeg - angle to be swept out in the turn
  //	ROLLING_TURN_MAX_SEC_PER_DEG = const used to generate timeout proportional to turn deg
  //	IMUHdgValDeg = IMU heading value updated by UpdateIMUHdgValDeg() //11/02/20 now updated in ISR
  //	degPerSec = float value denoting desired turn rate
  //  Kp, Ki, Kd = PID parameters
  //Plan:
  //	Step1: Get current heading as starting point
  //	Step2: Disable TIMER5 interrupts
  //	Step3: Compute new target value & timeout value
  //	Step4: Run motors until target reached, using inline PID algorithm to control turn rate
  //	Step5: Re-enable TIMER5 interrupts
  //Notes:
  //	06/06/21 we-written to remove PID library - now uses custom 'PIDCalcs()' function
  //	06/06/21 added re-try for 180.00 return from IMU - could be bad value
  //	06/11/21 added code to correct dHdg errors due to 179/-179 transition & bad IMU values
  //	06/12/21 cleaned up & commented out debug code
  //	11/14/21 removed 'first time skip' block; added motor start before entering loop
  //  03/22/22 added code to ensure numDeg >= 0
  //  06/01/22 now using MOTOR_SPEED_HALF for max motor speed and MOTOR_SPEED_OFF for low end
  //  12/03/22 copied from 'SpinTurn()' and adapted for 'rolling turn' algorithm
  //  12/05/22 added 'isFwd' to signature, and added code to support backwards rolling turns
  //  02/03/23 copied here from WallE3_RollingTurn_V1

  float tgt_deg;
  float timeout_sec;
  bool bDoneTurning = false;
  bool bTimedOut = false;
  bool bResult = true; //04/21/20 added so will be only one exit point

  numDeg = abs(numDeg);//  03/22/22 added to ensure numDeg >= 0
  //DEBUG!!
  //gl_pSerPort->printf("In RollingTurn(%s, %s, %2.2f, %2.2f) with PID = (%2.1f,%2.1f,%2.1f)\n",
  //  b_ccw == TURNDIR_CCW ? "CCW" : "CW", b_fwd == FWD_DIR ? "FWD" : "REV", numDeg, degPersec,
  //  Kp, Ki, Kd);
  //DEBUG!!

  //no need to continue if the IMU isn't available
  if (!dmpReady)
  {
    Serial.printf("DMP Failure - returning FALSE\n");
    return false;
  }

  //Step1: Get current heading as starting point
    //06/06/21 it is possible for IMU to return 180.00 on failure
    //so try again.  If it really IS 180, then 
    //it will eventually time out and go on

  //08/26/21 re-wrote using 3-value array to make sure initial heading is a steady value
  UpdateIMUHdgValDeg();

  int retries = 0;
  if ((IMUHdgValDeg == 180.f || IMUHdgValDeg == 0.f) && retries < 5)
  {
    //DEBUG!!
    //gl_pSerPort->printf("Got 180.00 or 0.00 exactly (%2.3f) from IMU - retrying %d...\n", IMUHdgValDeg, retries);
    //DEBUG!!
    UpdateIMUHdgValDeg();
    retries++;
    delay(100);
  }

  //Step2: Compute new target value & timeout value
  timeout_sec = 2 * numDeg / degPersec; //05/29/21 rev to use new turn rate parmeter

  //05/17/20 limit timeout_sec to 1 sec or more
  timeout_sec = (timeout_sec < 1) ? 1.f : timeout_sec;

  //12/05/19 added #define back in to manage which direction increases yaw values
#ifdef MPU6050_CCW_INCREASES_YAWVAL
  tgt_deg = b_ccw ? IMUHdgValDeg + numDeg : IMUHdgValDeg - numDeg;
#else
  tgt_deg = b_ccw ? IMUHdgValDeg - numDeg : IMUHdgValDeg + numDeg;

#endif // MPU6050_CCW_INCREASES_YAWVAL

  //correct for -180/180 transition
  if (tgt_deg < -180)
  {
    tgt_deg += 360;
  }

  //07/29/19 bugfix
  if (tgt_deg > 180)
  {
    tgt_deg -= 360;
  }

  //DEBUG!!
  //gl_pSerPort->printf("Init hdg = %4.2f deg, Turn = %4.2f  deg, tgt = %4.2f deg, timeout = %4.2f sec\n\n",
  //  IMUHdgValDeg, numDeg, tgt_deg, timeout_sec);
  //DEBUG!!

  float curHdgMatchVal = 0;

  //09/08/18 added to bolster end-of-turn detection
  float prevHdgMatchVal = 0;
  float matchSlope = 0;

  //Step3: Run motors until target reached, using PID algorithm to control turn rate
  Prev_HdgDeg = IMUHdgValDeg; //06/10/21 synch Prev_HdgDeg & IMUHdgValDeg just before entering loop

  elapsedMillis mSecSinceTurnStart = 0;
  MsecSinceLastTurnRateUpdate = 0;
  float lastError = 0;
  float lastInput = 0;
  float lastIval = 0;
  float lastDerror = 0;

  //DEBUG!!
  //gl_pSerPort->printf("Msec\tHdg\tPrvHdg\tdHdg\tRate\ttgtDPS\terr\tKp*err\tIval\tKd*Derr\tOut\tspeed\tMatch\tSlope\n");
  //DEBUG!!

  float avgrate = 0;
  uint16_t numrates = 0;

  while (!bDoneTurning && !bTimedOut)
  {
    //11/06/20 now just loops between PID calcs
    CheckForUserInput();

    if (MsecSinceLastTurnRateUpdate >= TURN_RATE_UPDATE_INTERVAL_MSEC)
    {
      MsecSinceLastTurnRateUpdate -= TURN_RATE_UPDATE_INTERVAL_MSEC;

      UpdateIMUHdgValDeg(); //update IMUHdgValDeg

      float dHdg = IMUHdgValDeg - Prev_HdgDeg;
      if (dHdg > 180)
      {
        dHdg -= 360;
        //Serial.printf("dHdg > 180 - subtracting 360\n");
      }
      else if (dHdg < -180)
      {
        dHdg += 360;
        //Serial.printf("dHdg < -180 - adding 360\n");
      }

      //watch for turn rates that are wildly off
      float rate = abs(1000 * dHdg / TURN_RATE_UPDATE_INTERVAL_MSEC);
      avgrate += rate;
      numrates++;

      //if (rate > 3 * degPersec)
      //{
      //	//DEBUG!!
      //	Serial.printf("hdg/prevhdg/dHdg/rate = %2.2f\t%2.2f\t%2.2f\t%2.2f, excessive rate - replacing with %2.2f\n", IMUHdgValDeg, Prev_HdgDeg, dHdg, rate, degPersec);
      //	//DEBUG!!
      //	rate = degPersec;
      //}

      //02/05/22 sampleTime removed from signature
      TurnRatePIDOutput = PIDCalcs(rate, degPersec, lastError, lastInput, lastIval, lastDerror, Kp, Ki, Kd);

      int speed = 0;

      //05/31/22 now using MOTOR_SPEED_HALF for max motor speed
      //06/01/22 and MOTOR_SPEED_OFF for low end
      speed = (TurnRatePIDOutput > MOTOR_SPEED_HALF) ? MOTOR_SPEED_HALF : (int)TurnRatePIDOutput;
      speed = (TurnRatePIDOutput <= MOTOR_SPEED_OFF) ? MOTOR_SPEED_OFF : (int)TurnRatePIDOutput;


      //gl_pSerPort->printf("In RollingTurn in %s section\n", b_fwd == FWD_DIR ? "FWD" : "REV");

      if (b_ccw)
      {
        if (b_fwd)
        {
          //this is the "1 1" case CCW & Fwd; rt motor fwd fast, left motor fwd slow
          //gl_pSerPort->printf("In RollingTurn in 1 1 Case\n");
          SetLeftMotorDirAndSpeed(true, MOTOR_SPEED_LOW - speed);
          SetRightMotorDirAndSpeed(true, MOTOR_SPEED_LOW + speed);
        }
        else
        {
          //gl_pSerPort->printf("In RollingTurn in 1 0 Case\n");
          //this is the "1 0" case CCW & Rev; rt motor rev slow, left motor rev fast
          SetLeftMotorDirAndSpeed(false, MOTOR_SPEED_LOW + speed);
          SetRightMotorDirAndSpeed(false, MOTOR_SPEED_LOW - speed);
        }
      }
      else //must be CW
      {
        if (b_fwd)
        {
          //gl_pSerPort->printf("In RollingTurn in 0 1 Case\n");
          //this is the "0 1" case CW & Fwd; rt motor fwd slow, left motor fwd fast
          SetLeftMotorDirAndSpeed(true, MOTOR_SPEED_LOW + speed);
          SetRightMotorDirAndSpeed(true, MOTOR_SPEED_LOW - speed);
        }
        else
        {
          //gl_pSerPort->printf("In RollingTurn in 0 0 Case\n");
          //this is the "0 0" case CW & Rev; rt motor rev fast, left motor rev slow
          SetLeftMotorDirAndSpeed(false, MOTOR_SPEED_LOW - speed);
          SetRightMotorDirAndSpeed(false, MOTOR_SPEED_LOW + speed);
        }
      }

      //check for nearly there and all the way there
      curHdgMatchVal = GetHdgMatchVal(tgt_deg, IMUHdgValDeg);
      matchSlope = curHdgMatchVal - prevHdgMatchVal;

      //DEBUG!!
      //gl_pSerPort->printf("%lu\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%2.1f\t%d\t%2.1f\t%2.1f\n",
      //  millis(),
      //  IMUHdgValDeg,
      //  Prev_HdgDeg,
      //  dHdg,
      //  rate,
      //  degPersec,
      //  lastError,
      //  Kp * lastError,
      //  lastIval,
      //  Kd * lastDerror,
      //  TurnRatePIDOutput,
      //  speed,
      //  curHdgMatchVal,
      //  matchSlope);
      //DEBUG!!

      Prev_HdgDeg = IMUHdgValDeg; //re-synch prev to curr hdg for next time

      //look for full match
      bDoneTurning = (curHdgMatchVal >= HDG_FULL_MATCH_VAL
        || (prevHdgMatchVal >= HDG_MIN_MATCH_VAL && matchSlope <= -0.01)); //have to use < vs <= as slope == 0 at start

      //Serial.printf("curHdgMatchVal = %2.2f, prevHdgMatchVal = %2.2f, matchslope = %2.2f, bDoneTurning = %d\n",
      //	curHdgMatchVal,
      //	prevHdgMatchVal,
      //	matchSlope,
      //	bDoneTurning);

      prevHdgMatchVal = curHdgMatchVal; //07/31/21 moved below bDoneTurning chk so can use prevHdgMatchVal vs curHdgMatchVal in slope check

      bTimedOut = (mSecSinceTurnStart > timeout_sec * 1000);

      if (bTimedOut)
      {
        //DEBUG!!
        //gl_pSerPort->printf("timed out with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", IMUHdgValDeg, tgt_deg, curHdgMatchVal);
        //DEBUG!!

        bResult = false;
        break;
      }

      if (bDoneTurning)
      {
        //gl_pSerPort->printf("Completed turn with yaw = %3.2f, tgt = %3.2f, and match = %1.3f\n", IMUHdgValDeg, tgt_deg, curHdgMatchVal);

        bResult = true;
        break;
      }
    }
  }

  avgrate = avgrate / numrates;

  //gl_pSerPort->printf("average turn rate = %2.1f\n", avgrate);

  StopBothMotors();
  //delay(1000); //added 04/27/21 for debug
  return bResult;
}

//06/13/23 added to detect 'spinning out of control' conditon
bool IsSpinning()
{
  //Purpose: detect 'spinning' condx
  //Inputs: 
  // gl_HdgHistoryArray = HEADING_HISTORY_ARRAY_SIZE element array of heading values
  //Outputs: returns TRUE if the cumulative heading change throughout the history array > 360
  //         else returns FALSE

  bool result = false;
  float deltaHdg = 0;
  float cumHdg = 0;
  //gl_pSerPort->printf("\nHdg\tDHdg\tCumHdg\n");
  for (uint16_t i = 1; i < HEADING_HISTORY_ARRAY_SIZE - 1; i++)
  {
    //deltaHdg = abs(gl_HdgHistoryArray[i - 1] - gl_HdgHistoryArray[i]);
    deltaHdg = gl_HdgHistoryArray[i - 1] - gl_HdgHistoryArray[i];

    //adjust for +180 to -179 transition
    if (deltaHdg > 180)
    {
      deltaHdg = deltaHdg - 360;
    }
    else if (deltaHdg < -180)
    {
      deltaHdg = deltaHdg + 360;
    }

    cumHdg += deltaHdg;

    //gl_pSerPort->printf("%2.2f\t%2.2f\t%2.2f\n", gl_HdgHistoryArray[i], deltaHdg, cumHdg); //prints out at end of line in "HDG_ONLY" mode

    //if (cumHdg > 360)
    if (abs(cumHdg) > 360)
    {
      //DEBUG!!
      gl_pSerPort->printf("IsSpinning(): Spin condx detected with Hdg = %2.2f, cumHdg = %2.2f, idx = %d\n", gl_HdgHistoryArray[i - 1], cumHdg, i);

      for (uint16_t i = 0; i < HEADING_HISTORY_ARRAY_SIZE; i++)
      {
        gl_pSerPort->printf("gl_HdgHistoryArray[%d] = %2.2f\n", i, gl_HdgHistoryArray[i]);

      }
      //DEBUG!!

      InitHeadingHistoryArray();//added 06/14/23

      result = true;
      cumHdg = 0;
    }
  }

  return result;
}

void InitHeadingHistoryArray()
{
  for (uint16_t i = 0; i < HEADING_HISTORY_ARRAY_SIZE; i++)
  {
    gl_HdgHistoryArray[i] = 0;
  }

}
#pragma endregion HDG_BASED_TURN_SUPPORT
