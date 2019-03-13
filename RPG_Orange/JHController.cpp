//
//  JHController.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHController.h"
#include "Globals.h"

JHController::JHController() {
    headingPI = JHPI(g_headingKi, g_headingKp);
    rollPI = JHPI(g_rollKi, g_rollKi);
    pPI = JHPI(g_pKi, g_pKi);
    qPI = JHPI(g_qKi, g_qKi);
    pitchPI = JHPI(g_pitchKi, g_pitchKi);
}

void JHController::pitchController(float pitchCommand, float currentPitch, float currentQ) {
    float qCommanded = pitchPI.updatePI(pitchCommand, currentPitch);
    pitchBilly = qPI.updatePI(qCommanded, currentQ);
}

void JHController::headingController(float headingCommand, float currentHeading, float currentRoll, float currentP) {
    float rollCommand = headingPI.updatePI(headingCommand, currentHeading);
    rollController(rollCommand, currentRoll, currentP);

}

void JHController::rollController(float rollCommand, float currentRoll, float currentP) {
    float pCommanded = rollPI.updatePI(rollCommand, currentRoll);
    rollBilly = pPI.updatePI(pCommanded, currentP);
}

void JHController::calculateDelta(float g_Gain, float g_Trim) {
    float d1 = rollBilly * g_Gain;
    float d2 = pitchBilly * g_Gain;
    deltaRudder = d1 + g_Trim;
    deltaLeftElevator = (d1 - d2) + g_Trim;
    deltaRightElevator = (d1 + d2) + g_Trim;
    
    
}

float JHController::rightElevator() {
    return deltaRightElevator;
}

float JHController::leftElevator() {
    return deltaLeftElevator;
}

float JHController::rudder()  {
    return deltaRudder;
}


