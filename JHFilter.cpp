//
//  JHFilter.cpp
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#include "JHFilter.h"
#include "BasicLinearAlgebra.h"
//#include <iostream>

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
//FIXME: need to fix matricies and math
BLA::Matrix<3, 3, Diagonal<3, float> > z3;
BLA::Matrix<4, 4, Diagonal<4, float> > z4;
BLA::Matrix<3,3> I3;
BLA::Matrix<4,4> I4;
BLA::Matrix<3,3> R;
BLA::Matrix<4,4> Q;
BLA::Matrix<4,4> F;
BLA::Matrix<3,1> h;
BLA::Matrix<3,4> H;

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
    
};



void JHFilter::begin(float hz, float RCV, float QCV) {
    t = 1.0f / hz;
    
    I3 = z3.Fill(1);
    I4 = z4.Fill(1);
    P = I4;
    R = z3.Fill(RCV);
    Q = z4.Fill(QCV);
    
}

void JHFilter::zeroAngles(float ax, float ay, float az) {
    float G = sqrt(ax*ax+ay*ay+az*az);
    float theta = asin(-ax/G);
    float phi = acos(az/(G*cos(theta)));
    float psi = 0;
    
    
    
    q0 = (cos(phi/2) * cos(theta/2) * cos(psi/2)) + (sin(phi/2) * sin(theta/2) * sin(psi/2));
    
    q1 =(sin(phi/2) * cos(theta/2) * cos(psi/2)) - (cos(phi/2) * sin(theta/2) * sin(psi/2));

    q2 =(cos(phi/2) * sin(theta/2) * cos(psi/2)) + (sin(phi/2) * cos(theta/2) * sin(psi/2));

    q3 =(cos(phi/2) * cos(theta/2) * sin(psi/2)) - (sin(phi/2) * sin(theta/2) * cos(psi/2));

    
    X = {
        q0,
        q1,
        q2,
        q3
    };
    
    //initial conditions
 /*   X = {
        0, //u
        0, //v
        0, //w
        180*phi/3.141592, //roll
        180*theta/3.141592, //pitch
        0, //yaw
        ax, //ax
        ay, //ay
        az, //az
        0, //p
        0, //q
        0, //r
    };
*/
    

}

void JHFilter::logRPY() {
   // std::cout << "Roll: " << getRoll() << ", Pitch: " << getPitch() << ", Yaw: " << getYaw() << "\n";
}

void JHFilter::updateIMU(float ax, float ay, float az, float p, float q, float r) {
    
    V = {ax,ay,az,p,q,r};
    
    p = p * 3.141592 / 180;
    q = q * 3.141592 / 180;
    r = r * 3.141592 / 180;
    
    z = {ax,ay,az};
    
    F = {
        1,          -p * t/2,   -q * t/2,   -r * t/2,
        p * t/2,    1,          r * t/2,    -q * t/2,
        q * t/2,    -r * t/2,   1,          p * t/2,
        r * t/2,    q * t/2,    -p * t/2,    1
    };
    

    H = {
      2 * g * q2,       2 * g * q3,     2 * g * q0,     2 * g * q1,
      2 * g * (-q1),    2 * g * (-q0),  2 * g * q3,     2 * g * q2,
      2 * g * q0,       2 * g * (-q1),  2 * g * (-q2),  2 * g * q3,
    };
    
     Xpriori = F * X;
    
     Ppriori = F * P * ~F + Q;
    
     f = H * Ppriori * ~H + R;
    
     K = Ppriori * ~H * Invert(f);

    h = {
        2 * g * (Xpriori(0) * Xpriori(2) + Xpriori(1) * Xpriori(3)),
        2 * g * (Xpriori(2) * Xpriori(3) - Xpriori(0) * Xpriori(1)),
        g * (Xpriori(0) * Xpriori(0) - Xpriori(1) * Xpriori(1) - Xpriori(2) * Xpriori(2) + Xpriori(3) * Xpriori(3))
    };
    
    X = Xpriori + K * (z - h);
    P = (I4 - K * H) * Ppriori;
    
}


float JHFilter::getRoll(){
    float roll = atan2(2 * (X(0) * X(1) + X(2) * X(3)), 1 - 2 * (X(1) * X(1) + X(2) * X(2)));
    
    return roll * 180 / 3.141592;
}
float JHFilter::getPitch(){
    float pitch = asin(2 * (X(0) * X(2) - X(1) * X(2)));
    
    return pitch * 180 / 3.141592;
}
float JHFilter::getYaw(){
    float yaw = atan2(2 * (X(0) * X(3) + X(1) * X(2)), 1 - 2 * (X(2) * X(2) + X(3) * X(3)));
    
    return yaw * 180 / 3.141592;
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
