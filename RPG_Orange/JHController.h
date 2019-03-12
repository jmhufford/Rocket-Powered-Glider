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

class JHController{
private:
    float deltaRightElevator;
    float deltaLeftElevator;
    float deltaRudder;
    JHPI headingPI;
    JHPI rollPI;
    JHPI pPI;
    JHPI qPI;
    JHPI pitchPI;
   
    
public:
    JHController();
    void PitchController(); //update controller
    void HeadingController();
    void RollController();
    

    float RightElevator(); //get elevator commands
    float LeftElevator();
    float Rudder();


};
#endif /* JHController_h */
