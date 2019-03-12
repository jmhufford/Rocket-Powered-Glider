//
//  JHGuidance.hpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#ifndef JHGuidance_h
#define JHGuidance_h

//#include <stdio.h>
#include "JHFilter.h"

enum GuidanceState
{
    rc = 0,
    setup,
    calibrate,
    boost,
    transition,
    glide,
//    flare,
    landed
};

class JHGuidance {

public:
    JHGuidance();
    
    void inputData(JHFilter&);

    void logSelf();
    
    // Calculated Outputs
    GuidanceState currentState = calibrate;
    float commandHeading();
    float commandPitch = 0.0f;
    float autopilotGain = 0.0f;

    // Autopilot sensitivity
//    float gain();  // possibly 2 values for elevator and rudder
    
private:
    float initialHeading = 0.0f;
    float initialPitch = 0.0f;
    
    void detectState(JHFilter& filter);
    void calulateGuidance(JHFilter& filter);
    
};

#endif /* JHGuidance_h */
