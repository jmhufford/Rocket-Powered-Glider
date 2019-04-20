//
//  JHFilter.cpp
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#include "JHFilter.h"
#include "BasicLinearAlgebra.h"
#include <stdio.h>

using namespace BLA;

template<int dim, class ElemT> struct Diagonal
{
    mutable ElemT m[dim];
    typedef ElemT elem_t;
    ElemT &operator()(int row, int col) const
    {
        static ElemT dummy;
        if(row == col && row < dim)
            return m[row];
        else
            return (dummy = 0);
    }
};
BLA::Matrix<3, 3, Diagonal<3, float> > z6;
BLA::Matrix<4, 4, Diagonal<4, float> > z4;
BLA::Matrix<3,3> I3;
BLA::Matrix<4,4> I4;
BLA::Matrix<3,3> R;
BLA::Matrix<4,4> Q;
BLA::Matrix<4,4> F;
BLA::Matrix<3,1> h;
BLA::Matrix<3,4> H;

BLA::Matrix<4,4> Hn;
BLA::Matrix<1,1> Xpriorin;
BLA::Matrix<1,1> Xn;


//declare function vars
BLA::Matrix<4> X;
BLA::Matrix<4> Xpriori;
BLA::Matrix<4,4> Ppriori;
BLA::Matrix<3,3> f;
BLA::Matrix<4,3> K;
BLA::Matrix<4,4> P;
BLA::Matrix<3> z;

BLA::Matrix<6> V;


JHFilter::JHFilter(){
    axLP = JHLowpass();
    ayLP = JHLowpass();
    azLP = JHLowpass();
    pLP = JHLowpass();
    qLP = JHLowpass();
    rLP = JHLowpass();
    
};

void JHFilter::refine() {
    ax = axLP.noiseReduction(ax);
    ay = ayLP.noiseReduction(ay);
    az = azLP.noiseReduction(az);
    p = pLP.noiseReduction(p);
    q = qLP.noiseReduction(q);
    r = rLP.noiseReduction(r);
    
}

float JHFilter::convertRawAcceleration(int aRaw) {
    
    float a = (aRaw * 4.0 * g) / 32768.0;
    return a;
}

float JHFilter::convertRawGyro(int gRaw) {
    
    float b = (gRaw * 500.0) / 32768.0;
    return b;
}

void JHFilter::begin(float hz, float RCV, float QCV) {
    t = 1.0f / hz;
    
    I3 = z6.Fill(1);
    I4 = z4.Fill(1);
    P = I4;
    R = z6.Fill(RCV);
    Q = z4.Fill(QCV);
}

void JHFilter::zeroAngles(float aix, float aiy, float aiz) {
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    
    float G = sqrt(ax*ax+ay*ay+az*az);
    float theta = asin(-ax/G);
    float phi = acos(az/(G*cos(theta)));
    float psi = 0;
    
    q4 = (cos(phi/2) * cos(theta/2) * cos(psi/2)) + (sin(phi/2) * sin(theta/2) * sin(psi/2));
    q1 =(sin(phi/2) * cos(theta/2) * cos(psi/2)) - (cos(phi/2) * sin(theta/2) * sin(psi/2));
    q2 =(cos(phi/2) * sin(theta/2) * cos(psi/2)) + (sin(phi/2) * cos(theta/2) * sin(psi/2));
    q3 =(cos(phi/2) * cos(theta/2) * sin(psi/2)) - (sin(phi/2) * sin(theta/2) * cos(psi/2));

    X = {
        q1,
        q2,
        q3,
        q4
    };
}

void JHFilter::updateIMU(float aix, float aiy, float aiz, float gip, float giq, float gir) {
    
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    p = convertRawGyro(gip);
    q = convertRawGyro(giq);
    r = convertRawGyro(gir);
    
    V = {ax,ay,az,p,q,r};

    p = deg2rad(p);
    q = deg2rad(q);
    r = deg2rad(r);
    
    z = {ax,ay,az};
    
    //entries go left to right, top to bottom
    H = {
        2*X(2)*g,
        2*-X(3)*g,
        2*X(0)*g,
        2*-X(1)*g,
        
        2*X(3)*g,
        2*X(2)*g,
        2*X(1)*g,
        2*X(0)*g,
        
        2*-X(0)*g,
        2*-X(1)*g,
        2*X(2)*g,
        2*X(3)*g,
    };
    
    F = {
        1,          r * t/2,   -q * t/2,   p * t/2,
        -r * t/2,    1,          p * t/2,    q * t/2,
        q * t/2,    -p * t/2,   1,          r * t/2,
        -p * t/2,    -q * t/2,    -r * t/2,    1
    };
    
    Xpriori = F * X;
    
    Xpriorin = {1/sqrt((Xpriori(0))*(Xpriori(0))+(Xpriori(1))*(Xpriori(1))+(Xpriori(2))*(Xpriori(2))+(Xpriori(3))*(Xpriori(3)))};
    
    Xpriori = Xpriori * Xpriorin;

    Ppriori = F * P * ~F + Q;
    
    f = H * Ppriori * ~H + R;
    
    K = Ppriori * ~H * Invert(f);

    h = {
        2*g*(Xpriori(0)*Xpriori(2)-Xpriori(1)*Xpriori(3)),
        2*g*(Xpriori(1)*Xpriori(2)+Xpriori(0)*Xpriori(3)),
        g*(-(Xpriori(0)*Xpriori(0))-(Xpriori(1)*Xpriori(1))+(Xpriori(2)*Xpriori(2))+(Xpriori(3)*Xpriori(3))),
    };
 
    X = Xpriori + K * (z - h);
    
    //normalizing X
    Xn = {1/sqrt((X(0))*(X(0))+(X(1))*(X(1))+(X(2))*(X(2))+(X(3))*(X(3)))};
    
    X = X * Xn;

    P = (I4 - K * H) * Ppriori;
}

float JHFilter::rad2deg(float rad) {
    return (rad / M_PI) * 180;
}

float JHFilter::deg2rad(float deg) {
    return (deg * M_PI) / 180;
}

float JHFilter::clip(float n) {
    return fmax(-1, fmin(n, 1));
}

float JHFilter::getRoll(){
    float roll = atan2(2 * (X(3) * X(0) + X(1) * X(2)), 1 - 2 * (X(0) * X(0) + X(1) * X(1)));
    
    return rad2deg(roll);
}

float JHFilter::getPitch(){
    float pitch = asin(2 * (X(3) * X(1) - X(0) * X(2)));
    
    if (2 * (X(3) * X(1) - X(0) * X(2))<-1) {
        pitch = -3.141592/2;
    }
    if (2 * (X(3) * X(1) - X(0) * X(2))>1) {
        pitch = 3.141592/2;
    }
    return rad2deg(pitch);
}

float JHFilter::getYaw(){
    float yaw = atan2(2 * (X(3) * X(2) + X(0) * X(1)), 1 - 2 * (X(1) * X(1) + X(2) * X(2)));
    
    return rad2deg(yaw);
}

float JHFilter::getAx(){
    return V(0);
}
float JHFilter::getAy(){
    return V(1);
}
float JHFilter::getAz(){
    return V(2);
}
float JHFilter::getP(){
    return V(3);
}
float JHFilter::getQ(){
    return V(4);
}
float JHFilter::getR(){
    return V(5);
}


