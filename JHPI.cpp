//
//  JHPI.cpp
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/28/19.
//

#include "JHPI.h"
#include "Globals.h"
JHPI::JHPI() { //default
}

JHPI::JHPI(float i, float p) { //customized constructor
    intError = 0;
    Ki = i;
    Kp = p;
}

void JHPI::begin() { //currently unused reset/start/adjust


}

float JHPI::update(float commanded, float actual) {
    
    float error = commanded - actual; // finding error
    intError = (error * t) + intError; //tracking/calulating int component
    output = (Kp * error) + (Ki*intError); //adding on proportional component
    
    return output; //returning PI output
}


