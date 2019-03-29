//
//  JHFilter.h
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#ifndef JHFilter_h
#define JHFilter_h
#define M_PI 3.1415926535

class JHFilter{
private:
    float t;
    float RCV;
    float QCV;
    const float g = 9.81;
    float q4;
    float q1;
    float q2;
    float q3;
    float clip(float n);
    float convertRawAcceleration(int aRaw);
    float convertRawGyro(int gRaw);
    float convertRawMag(int mRaw);
    float HX = 0;
    float HY = 0;
    float HZ = 0;
    
    float mx=0;
    float my=0;
    float mz=0;
    float rad2deg(float rad);
    float deg2rad(float deg);
public:
    
    JHFilter(void);
    void begin(float hz, float RCV, float QCV);
    void zeroAngles(float ax, float ay, float az);
    void updateIMU(float ax, float ay, float az, float p, float q, float r);
    
    void logRPY();

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
