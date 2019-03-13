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
    void begin();
    float update(float commanded, float actual);
    
    
};

#endif /* JHPI_h */
