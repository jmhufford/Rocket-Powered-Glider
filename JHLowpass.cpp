//
//  JHLowpass.cpp
//  RPG Orange
//
//  Created by Jerome - UCF on 3/31/19.
//  Copyright © 2019 Jerome.gg. All rights reserved.
//

#include "JHLowpass.h"

JHLowpass::JHLowpass() {
    // default
}

float JHLowpass::noiseReduction(float x) {
    float stor[5] { x, stor[0], stor[1], stor[2], stor[3]};
    
    float y =  (stor[0] + stor[1] + stor[2] + stor[3] + stor[4]) / 5;
    
    return  y;
}






