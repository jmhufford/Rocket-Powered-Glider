//
//  JHController.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHController.h"
#include "Globals.h"
#include "JHGuidance.h"
JHController::JHController() { //initialize needed PI objects during controller construction
    pitchPI = JHPI(g_pitchKi, g_pitchKp);
    qPI = JHPI(g_qKi, g_qKp);
    
    rollPI = JHPI(g_rollKi, g_rollKp);
    pPI = JHPI(g_pKi, g_pKp);
    
    headingPI = JHPI(g_headingKi, g_headingKp);
    rPI = JHPI(g_rKi, g_rKp);
}

void JHController::pitchController(float pitchCommand, float currentPitch, float currentQ) { // two loop conrtoller for pitch
    if (pitchCommand == g_Ignore) return;
    if (pitchCommand == g_Launch) {
        pitchInt = -20;
        return;
    }
    float qCommanded = pitchPI.update(pitchCommand, currentPitch);
    pitchInt = qPI.update(qCommanded, currentQ);
}

void JHController::headingController(float headingCommand, float currentR) { // heading controller
    if (headingCommand == g_Ignore) return;
    
    float rCommand = headingCommand; // headingPI.update(headingCommand, currentHeading);
    rollInt = rPI.update(rCommand, currentR);
}

void JHController::rollController(float rollCommand, float currentRoll, float currentP) { // two loop conrtoller for roll
    if (rollCommand == g_Ignore) return;
    
    float pCommanded = rollPI.update(rollCommand, currentRoll);
    rollInt = pPI.update(pCommanded, currentP);
}

void JHController::calculateDelta(float gain) { // applying gains to output and
    float d1 = rollInt * gain;
    float d2 = pitchInt * gain;
    float d3 = headingInt * gain;
    
    rollPI.d = d1;
    pPI.d = d1;
    pitchPI.d = d2;
    qPI.d = d2;
    headingPI.d = d3;
    rPI.d = d3;
    
    deltaRudder = d3 + g_Trim;
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


