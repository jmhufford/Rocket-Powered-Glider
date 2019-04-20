//
//  JHFilter.h
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#ifndef JHFilter_h
#define JHFilter_h
#define M_PI 3.1415926535
#include "JHLowpass.h"


class JHFilter{
private:
    float t;
    const float g = 9.81;
    float q4;
    float q1;
    float q2;
    float q3;
    float clip(float n);
    void refine();
    float convertRawAcceleration(int aRaw);
    float convertRawGyro(int gRaw);
    float rad2deg(float rad);
    float deg2rad(float deg);
    
    JHLowpass axLP;
    JHLowpass ayLP;
    JHLowpass azLP;
    JHLowpass pLP;
    JHLowpass qLP;
    JHLowpass rLP;

    float ax, ay, az, p, q, r;
    
public:
    
    JHFilter(void);
    void begin(float hz, float RCV, float QCV);
    void zeroAngles(float ax, float ay, float az);
    void updateIMU(float ax, float ay, float az, float p, float q, float r);
    
    float getRoll();
    float getPitch();
    float getYaw();
    float getAx();
    float getAy();
    float getAz();
    float getP();
    float getQ();
    float getR();
};




#endif /* JHFilter_h */
