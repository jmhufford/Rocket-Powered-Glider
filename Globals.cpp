//
//  Globals.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/12/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include <stdio.h>
#include "Globals.h"



const float g_ReadRateHz = 50;    // read rate
const float t = 1/g_ReadRateHz;   //read rate in sec


//FIXME: do covariances change over time???
//measurement covariance (how much each measurement is “trusted”)(R)
const float g_RCV = .001;

//process covariance (uncertainty in the modeling)(Q)
const float g_QCV = .01;

//Controller scalers
 const float g_Trim = 90;
//pitch
const float g_pitchKi = 0.0135;
const float g_pitchKp = 0.0263;
//q
const float g_qKi = -0.676;
const float g_qKp = -5.5;
//heading
const float g_headingKi = 0;
const float g_headingKp = 0;
//roll
const float g_rollKi = .022222;
const float g_rollKp = .007777;
//p
const float g_pKi = -0.276;
const float g_pKp = -5.5;

const float g_Ignore = 1000;
const float g_Glide = -1.5;
const float g_Zero = 0;

const float g_GainGlide = 1;
const float g_GainBoost = 1;




