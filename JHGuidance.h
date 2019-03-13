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
    standby,
    calibrate,
    boost,
    transition,
    glide,
//    flare,
    landed
};

class JHGuidance {

private:
    float headingCommand;
    float pitchCommand;
    float rollCommand;
    void detectState(JHFilter& filter);
    void calulateGuidance(JHFilter& filter);
    float initialHeading;
    float initialPitch;

public:
    JHGuidance();
    
    void inputData(JHFilter&);

    void logSelf();
    
    // Calculated Outputs
    GuidanceState currentState = calibrate;
    
    float getHeadingCommand();
    float getRollCommand();
    float getPitchCommand();

    // Autopilot sensitivity
//    float gain();  // possibly 2 values for elevator and rudder
    

    
};

#endif /* JHGuidance_h */
