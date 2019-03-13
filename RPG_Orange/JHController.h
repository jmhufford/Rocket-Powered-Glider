//
//  JHController.h
//  RPG Orange
//
//  Created by Jerome - UCF on 3/11/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#ifndef JHController_h
#define JHController_h

#include <stdio.h>
#include "JHPI.h"

class JHController {
private:
    float deltaRightElevator;
    float deltaLeftElevator;
    float deltaRudder;
    float pitchBilly;
    float rollBilly;
    JHPI headingPI;
    JHPI rollPI;
    JHPI pPI;
    JHPI qPI;
    JHPI pitchPI;
   
    
public:
    JHController();
    void pitchController(float pitchCommand, float currentPitch, float currentQ); //update controller
    void headingController(float headingCommand, float currentHeading, float currentRoll, float currentP);
    void rollController(float rollCommand, float currentRoll, float currentP);
    void calculateDelta(float g_Gain, float g_Trim);

    float rightElevator(); //get elevator commands
    float leftElevator();
    float rudder();


};
#endif /* JHController_h */
