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


//FIXME: do covariances change over time???
//measurement covariance (how much each measurement is “trusted”)(R)
const float g_RCV = .1;

//process covariance (uncertainty in the modeling)(Q)
const float g_QCV = .01;

//Controller scalers
 const float g_Trim = 90;
 float g_Gain = 0;
//pitch
const float g_pitchKi = 1;
const float g_pitchKp = 1;
//q
const float g_qKi = 1;
const float g_qKp = 1;
//heading
const float g_headingKi = 1;
const float g_headingKp = 1;
//roll
const float g_rollKi = 1;
const float g_rollKp = 1;
//p
const float g_pKi = 1;
const float g_pKp = 1;

const float g_Ignore = 1000;
const float g_Glide = -3;
const float g_Zero = 0;

const float g_GainGlide = 1;
const float g_GainBoost = 1;




