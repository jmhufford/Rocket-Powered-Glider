//
//  JHGuidance.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHGuidance.h"
#include <stdio.h>

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
            
// TODO: handle all states
            
        case calibrate:
            // if Accelerating we change states...
            if (filter.getAx() > 0.05)
                newState = boost;
            
            else {
                // always capture the current heading and pitch from the Filter
                
                initialHeading = filter.getYaw();
                initialPitch = filter.getPitch();
            }
            break;

        case boost:
            // if AccelX goes down + other indicators?
            if (filter.getAx() < 0.4)
                newState = transition;
            break;
            
        default: break;
    }
    
    if (newState != currentState) {
        // State Changed, is there anything to do
        currentState = newState;
    }
}

void JHGuidance::calulateGuidance(JHFilter& filter) {

    // TODO: based on the current state, calculate the desired pitch, and autopilot gain...
    
    switch (currentState) {
            
            // TODO: handle all states

        case calibrate:
            commandPitch = initialPitch;
            break;

        case boost:
            commandPitch = initialPitch;
            break;

        case transition:
            // pitch will step from 60 ... -3
            commandPitch = 20;
            break;

        case glide:
            commandPitch = -3.0;
            break;
            
        default: break;
    }
}

float JHGuidance::commandHeading() {
    return initialHeading;
}
