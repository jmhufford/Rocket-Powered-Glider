//
//  Globals.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/12/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include <stdio.h>
#include "Globals.h"



const float g_ReadRateHz = 100;    // read rate
const float t = 1/g_ReadRateHz;   //read rate in sec


//FIXME: do covariances change over time???
//measurement covariance (how much each measurement is “trusted”)(R)
const float g_RCV = .001;

//process covariance (uncertainty in the modeling)(Q)
const float g_QCV = .1;

//Controller scalers
 const float g_Trim = 90;
//pitch
const float g_pitchKi = .1; //0.0135;
const float g_pitchKp = 4; //0.0263;
//q
const float g_qKi = .3;//0.676;
const float g_qKp = 0.7;
//roll
const float g_rollKi = .01;
const float g_rollKp = 3;
//p
const float g_pKi = .03;
const float g_pKp = 0.7;
//heading
const float g_headingKi = 0;
const float g_headingKp = 0;
//r
const float g_rKi = 0;
const float g_rKp = 0;

const float g_Ignore = 1000;
const float g_Glide = -3.5;
const float g_Zero = 0;
const float g_Launch = -1000;

const float g_GainGlide = 1;
const float g_GainBoost = 1;




