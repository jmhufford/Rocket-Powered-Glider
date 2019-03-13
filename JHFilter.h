//
//  JHFilter.h
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#ifndef JHFilter_h
#define JHFilter_h

class JHFilter{
private:
    float t;
    float RCV;
    float QCV;
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
