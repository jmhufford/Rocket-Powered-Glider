//
//  Globals.h
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#ifndef Globals_h
#define Globals_h

extern const float g_ReadRateHz;    // read rate
extern const float t;   //read rate in sec

//measurement covariance (how much each measurement is “trusted”)(R)
extern const float g_RCV;

//process covariance (uncertainty in the modeling)(Q)
extern const float g_QCV;

//Controller scalers
extern const float g_Trim;
//pitch
extern const float g_pitchKi;
extern const float g_pitchKp;
//q
extern const float g_qKi;
extern const float g_qKp;
//roll
extern const float g_headingKi;
extern const float g_headingKp;
//p
extern const float g_pKi;
extern const float g_pKp;
//bank
extern const float g_rollKi;
extern const float g_rollKp;
//r
extern const float g_rKi;
extern const float g_rKp;

extern const float g_Ignore;
extern const float g_Glide;
extern const float g_Zero;
extern const float g_GainGlide;
extern const float g_GainBoost;
extern const float g_Launch;
#endif /* Globals_h */
