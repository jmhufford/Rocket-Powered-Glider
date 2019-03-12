//
//  main.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/10/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include <stdio.h>
#include "Globals.h"
#include "JHFilter.h"
#include "JHGuidance.h"
#include "JHController.h"

JHFilter filter;
JHGuidance guidance;

void setup2() {
    
//    filter.begin(20, 1, 1);
    filter.begin(g_ReadRateHz, g_RCV, g_QCV);
}

void loop() {
    
    // check if it's time to read data
    //   CurieIMU.readMotionSensor(aix, aiy, aiz, gip, giq, gir);
    
    // convert from raw data to gravity and degrees/second units
    // feed into the filter
    
//    filter.updateIMU(0, 0, 1, 0, 0, 0);
    filter.updateIMU(10, 0, 1, 0, 100, 0);
    filter.logRPY();
    
    guidance.inputData(filter);
    guidance.logSelf();
    
    // feed guidance commands into Autopilot
}

int main(int argc, const char * argv[]) {
    
    setup2();
    loop();
    
    printf("\n\n");
    return 0;
}

