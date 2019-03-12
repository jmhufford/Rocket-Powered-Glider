//
//  JHPI.cpp
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/28/19.
//

#include "JHPI.h"

JHPI::JHPI(){
}

JHPI::JHPI(float i, float p){
    intError = 0;
    Ki = i;
    Kp = p;
}

void JHPI::begin(){


}

float JHPI::updatePI(float commanded, float actual){
    
    float error = commanded - actual;
    intError = (error * Ki) + intError;
    output = (Kp * error) + intError;
    
    return output;
}






