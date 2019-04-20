//
//  JHPI.h
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/28/19.
//

#ifndef JHPI_h
#define JHPI_h

class JHPI{
private:
    float intError;
    float Ki;
    float Kp;
    float output;
    
public:
    JHPI();
    JHPI(float i, float p);
    void cap(float d);
    float update(float commanded, float actual);
    float d;
    
};

#endif /* JHPI_h */
