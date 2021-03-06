//
//  JHGuidance.hpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#ifndef JHGuidance_h
#define JHGuidance_h
#include "JHFilter.h"

enum GuidanceState
{
    rc = 0,
    standby,
    calibrate,
    boost,
    transition,
    glide,
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
    float gain;
    float count;
public:
    JHGuidance();
    
    void inputData(JHFilter&);

    void logSelf();
    
    // Calculated Outputs
    GuidanceState currentState = standby;
    
    float getHeadingCommand();
    float getRollCommand();
    float getPitchCommand();
    float getGainCommand();
    float getState();
};

#endif /* JHGuidance_h */
