//
//  JHController.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHController.h"
#include "Globals.h"

JHController::JHController(){
    headingPI = JHPI(g_headingKi, g_headingKp);
    rollPI = JHPI(g_rollKi, g_rollKi);
    pPI = JHPI(g_pKi, g_pKi);
    qPI = JHPI(g_qKi, g_qKi);
    pitchPI = JHPI(g_pitchKi, g_pitchKi);
}

void JHController::PitchController(){
    
}

void JHController::HeadingController(){
    float bill = headingPI.updatePI(432, 3422);


}

void JHController::RollController(){
    
}





float JHController::RightElevator(){
    return deltaRightElevator;
}

float JHController::LeftElevator(){
    return deltaLeftElevator;
}

float JHController::Rudder(){
    return deltaRudder;
}


