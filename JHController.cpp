//
//  JHController.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHController.h"
#include "Globals.h"

JHController::JHController() { //initialize needed PI objects during controller construction
    headingPI = JHPI(g_headingKi, g_headingKp);
    rollPI = JHPI(g_rollKi, g_rollKi);
    pPI = JHPI(g_pKi, g_pKi);
    qPI = JHPI(g_qKi, g_qKi);
    pitchPI = JHPI(g_pitchKi, g_pitchKi);
}

void JHController::pitchController(float pitchCommand, float currentPitch, float currentQ) { // two loop conrtoller for pitch
    if (pitchCommand == g_Ignore) return;
    
    float qCommanded = pitchPI.update(pitchCommand, currentPitch);
    pitchBilly = qPI.update(qCommanded, currentQ);
}

void JHController::headingController(float headingCommand, float currentHeading, float currentRoll, float currentP) { // heading controller outputs a roll command
    if (headingCommand == g_Ignore) return;
    
    float rollCommand = headingPI.update(headingCommand, currentHeading);
    rollController(rollCommand, currentRoll, currentP);

}

void JHController::rollController(float rollCommand, float currentRoll, float currentP) { // two loop conrtoller for roll
    if (rollCommand == g_Ignore) return;
    
    float pCommanded = rollPI.update(rollCommand, currentRoll);
    rollBilly = pPI.update(pCommanded, currentP);
}

void JHController::calculateDelta(float gain) { // applying gains to output and
    float d1 = rollBilly * gain;
    float d2 = pitchBilly * gain;
    deltaRudder = d1 + g_Trim;
    deltaLeftElevator = (d1 - d2) + g_Trim;
    deltaRightElevator = (d1 + d2) + g_Trim;
    
    
}
//functions that return control surface deflections
float JHController::rightElevator() {
    return deltaRightElevator;
}

float JHController::leftElevator() {
    return deltaLeftElevator;
}

float JHController::rudder()  {
    return deltaRudder;
}


