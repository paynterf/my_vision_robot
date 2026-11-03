#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

enum AnomalyCode
{
  ANOMALY_NONE = 0,
  ANOMALY_STUCK_AHEAD,
  ANOMALY_STUCK_BEHIND,
  ANOMALY_OBSTACLE_AHEAD,
  ANOMALY_WALL_OFFSET_DIST_AHEAD,
  ANOMALY_OBSTACLE_BEHIND,
  ANOMALY_DEAD_BATTERY,
  ANOMALY_CHARGER_CONNECTED, //added 05/22/22
  ANOMALY_EXCESS_STEER_VAL, //added 05/19/23
  ANOMALY_IR_BEAM_AVAILABLE, //added 08/20/23
  ANOMALY_MIRRORED_SFC, //added 01/21/24
  ANOMALY_MANUAL_CONTROL //added 11/02/26 
};

//08/23/23 unused names removed
//11/02/26 "MANUAL_CONTROL" replaced "MIRRORED_SURFACE"
const char* AnomalyStrArray[] = { "NONE", "STUCK_AHEAD", "STUCK_BEHIND", "OBSTACLE_AHEAD",
                                  "WALL_OFFSET_DIST_AHEAD","OBSTACLE_BEHIND",
                                  "DEAD_BATTERY", "CHARGER_CONNECTED",
                                  "EXCESS_STEER_VAL", "IR_BEAM_AVAILABLE", "MANUAL_CONTROL"
                                };//02/08/23 added "OPEN_DOORWAY" 03/13/23 added "TRACKING_WRONG_WALL", 01/22/24 added "MIRRORED_SURFACE"

enum WallTrackingCases
{
  TRACKING_NONE = 0,
  TRACKING_LEFT,
  //TRACKING_LEFT_CAPTURE, //added 02/26/22 c/o 03/14/23
  TRACKING_RIGHT,
  //TRACKING_RIGHT_CAPTURE, //added 02/26/22 c/o 03/14/23
  TRACKING_NEITHER
};
const char* WallTrackStrArray[] = { "NONE", "LEFT", "RIGHT", "NEITHER"};

enum VL53L0X_REQUEST
{
  VL53L0X_READYCHECK, //added 11/10/20 to prevent bad reads during Teensy setup()
  VL53L0X_CENTERS_ONLY,
  VL53L0X_RIGHT,
  VL53L0X_LEFT,
  VL53L0X_ALL, //added 08/06/20, replaced VL53L0X_BOTH 10/31/20
  VL53L0X_REAR_ONLY //added 10/24/20
};
