//
//  JHGuidance.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHGuidance.h"
#include <stdio.h>
#include "Globals.h"
JHGuidance::JHGuidance() {
    
}

void JHGuidance::logSelf() {
    printf("Guidance: %d", currentState);
}

void JHGuidance::inputData(JHFilter& filter) {
    // called each time there is fresh Filter data
    
    detectState(filter);
    calulateGuidance(filter);
}

void JHGuidance::detectState(JHFilter& filter) {
    
    // based on the Filter data determine state change
    // for instance, if currentState is calibrate, and AccelX goes up then you advance to State:boost...
    
    GuidanceState newState = currentState;
    
    switch (currentState) {
        
        case standby:
            //have craft sit idle until it is no longer being re adjusted
            if (filter.getP() < 1 && filter.getR() < 1) {
                newState = calibrate;
                count = 0;
            }
            break;
            
        case calibrate: //capture starting angles and heading and watch for boost or rerurn to setup
            // if Accelerating we change states...
            if (filter.getAx() < -15)
                newState = boost;
            else {
                // always capture the current heading and pitch from the Filter
                initialHeading = filter.getYaw();
                initialPitch = filter.getPitch();
            }
            break;

        case boost:
            // if AccelX goes down + other indicators?
            if (filter.getAx() > -5) {
                newState = transition;
                count = 0;}
            break;
            
        case transition:
            //if pitch is about level transition too glide
            if (filter.getPitch() < 5) {
                count += 1;
                if (count >= 100) {
                    newState = glide;
                    count = 0;
                }
            }
            break;
        
        case glide:
            if (filter.getP() < 0.05 && filter.getQ() < 0.05) {
                count += 1;
                if (count >= 100000) {
                    newState = landed;
                    count = 0;
                }
            }
            break;
            
        case landed:
            break;
            
        default:
            break;
    }
    
    if (newState != currentState) {
        // State Changed, is there anything to do
        currentState = newState;
    }
}

void JHGuidance::calulateGuidance(JHFilter& filter) {
    //IMPORTANT: If not controlling value set command to g_Ignore
    switch (currentState) {
            
        case standby:
            headingCommand = g_Ignore;
            rollCommand = g_Ignore;
            pitchCommand = g_Ignore;
            gain = g_Zero;
            break;
            
        case calibrate:
            headingCommand = g_Ignore;
            rollCommand = g_Ignore;
            pitchCommand = g_Ignore;
            gain = g_Zero;
            break;

        case boost: // control roll to zero and maintain current pitch
            headingCommand = g_Ignore;
            rollCommand = g_Zero;
            pitchCommand = g_Launch;
            gain = g_GainBoost; // TODO: find this
            break;

        case transition:
            // pitch will step from starting to level
            headingCommand = g_Ignore;
            rollCommand = g_Zero;
            pitchCommand = g_Launch;  //TODO: find a way to step this
            gain = g_Zero; // TODO: find this
            break;

        case glide: //maintain optimal glide angle
            headingCommand = g_Ignore;
            rollCommand = g_Zero;
            pitchCommand = g_Glide; //TODO: find this value
            gain = g_GainGlide; // TODO: find this
            break;
            
        case landed:
            headingCommand = g_Ignore;
            rollCommand = g_Ignore;
            pitchCommand = g_Ignore;
            gain = g_Zero;
            break;
            
        default:
            headingCommand = g_Ignore;
            rollCommand = g_Ignore;
            pitchCommand = g_Ignore;
            gain = g_Zero;
            break;
    }
}

float JHGuidance::getHeadingCommand() {
    return headingCommand;
}

float JHGuidance::getRollCommand() {
    return rollCommand;
}

float JHGuidance::getPitchCommand() {
    return pitchCommand;
}

float JHGuidance::getGainCommand() {
    return gain;
}
float JHGuidance::getState() {
    return currentState;
}
