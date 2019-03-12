//
//  Globals.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/12/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include <stdio.h>
#include "Globals.h"



const float g_ReadRateHz = 25;    // read rate
const float t = 1/g_ReadRateHz;   //read rate in sec

//measurement covariance (how much each measurement is “trusted”)(R)
const float g_RCV = .1;

//process covariance (uncertainty in the modeling)(Q)
const float g_QCV = .01;

//Controller scalers
extern const float g_Trim;
extern float g_Gain;
//pitch
extern const float g_pitchKi = 1;
extern const float g_pitchKp = 1;
//q
const float g_qKi = 1;
const float g_qKp = 1;
//roll
const float g_headingKi = 1;
const float g_headingKp = 1;
//bank
const float g_rollKi = 1;
const float g_rollKp = 1;
//p
const float g_pKi = 1;
const float g_pKp = 1;






