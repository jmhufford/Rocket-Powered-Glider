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
//FIXME: need to fix matricies and math
BLA::Matrix<6, 6, Diagonal<6, float> > z6;
BLA::Matrix<4, 4, Diagonal<4, float> > z4;
BLA::Matrix<6,6> I6;
BLA::Matrix<4,4> I4;
BLA::Matrix<6,6> R;
BLA::Matrix<4,4> Q;
BLA::Matrix<4,4> F;
BLA::Matrix<6,1> h;
BLA::Matrix<6,4> H;

BLA::Matrix<4,4> Hn;
BLA::Matrix<1,1> hn;

//declare function vars
BLA::Matrix<4> X;
BLA::Matrix<4> Xpriori;
BLA::Matrix<4,4> Ppriori;
BLA::Matrix<6,6> f;
BLA::Matrix<4,6> K;
BLA::Matrix<4,4> P;
BLA::Matrix<6> z;

BLA::Matrix<6> V;

JHFilter::JHFilter(){
    
};

float JHFilter::convertRawAcceleration(int aRaw) {
    
    float a = (aRaw * 4.0 * 9.81) / 32768.0;
    return a;
}

float JHFilter::convertRawGyro(int gRaw) {
    
    float g = (gRaw * 500.0) / 32768.0;
    return g;
}

float JHFilter::convertRawMag(int mRaw) {
    
    float m = (mRaw * 250.0) / 32768.0;
    return m;
}


void JHFilter::begin(float hz, float RCV, float QCV) {
    t = 1.0f / hz;
    
    I6 = z6.Fill(1);
    I4 = z4.Fill(1);
    P = I4;
    R = z6.Fill(RCV);
    Q = z4.Fill(QCV);
    
}

void JHFilter::zeroAngles(float aix, float aiy, float aiz) {
    float ax = convertRawAcceleration(aix);
    float ay = convertRawAcceleration(aiy);
    float az = convertRawAcceleration(aiz);
    
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

void JHFilter::updateIMU(float aix, float aiy, float aiz, float gip, float giq, float gir) {
    
    float ax = convertRawAcceleration(aix);
    float ay = convertRawAcceleration(aiy);
    float az = convertRawAcceleration(aiz);
    float p = convertRawGyro(gip);
    float q = convertRawGyro(giq);
    float r = convertRawGyro(gir);

    
    V = {ax,ay,az,p,q,r};
    
    p = p * 3.141592 / 180;
    q = q * 3.141592 / 180;
    r = r * 3.141592 / 180;
    
    z = {ax,ay,az,mx,my,mz};
    
    Hn = z4.Fill(1/sqrt((X(0))*(X(0))+(X(1))*(X(1))+(X(2))*(X(2))+(X(3))*(X(3))));
    
    H = {
        ((2*X(2))-(2*(X(0)*X(2)-X(1)*X(3))*X(0)))*g,
        ((2*-X(3))-(2*(X(0)*X(2)-X(1)*X(3))*X(1)))*g,
        ((2*X(0))-(2*(X(0)*X(2)-X(1)*X(3))*X(2)))*g,
        ((2*-X(1))-(2*(X(0)*X(2)-X(1)*X(3))*X(3)))*g,
        
        ((2*X(3))-(2*(X(1)*X(2)-X(0)*X(3))*X(0)))*g,
        ((2*X(2))-(2*(X(1)*X(2)-X(0)*X(3))*X(1)))*g,
        ((2*X(1))-(2*(X(1)*X(2)-X(0)*X(3))*X(2)))*g,
        ((2*X(0))-(2*(X(1)*X(2)-X(0)*X(3))*X(3)))*g,
        
        ((2*-X(0))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(0)))*g,
        ((2*-X(1))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(1)))*g,
        ((2*X(2))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(2)))*g,
        ((2*X(3))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(3)))*g,
        
        ((2*X(0))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(0)))*HX + ((2*X(1))-((2*(X(0)*X(1)+X(2)*X(3)))*X(0)))*HY + ((2*X(2))-((2*(X(0)*X(2)-X(1)*X(3)))*X(0)))*HZ,
        ((2*-X(1))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(1)))*HX + ((2*X(0))-((2*(X(0)*X(1)+X(2)*X(3)))*X(1)))*HY + ((2*-X(3))-((2*(X(0)*X(2)-X(1)*X(3)))*X(1)))*HZ,
        ((2*-X(2))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(2)))*HX + ((2*X(3))-((2*(X(0)*X(1)+X(2)*X(3)))*X(2)))*HY + ((2*X(0))-((2*(X(0)*X(2)-X(1)*X(3)))*X(2)))*HZ,
        ((2*X(3))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(3)))*HX + ((2*X(2))-((2*(X(0)*X(1)+X(2)*X(3)))*X(3)))*HY + ((2*-X(1))-((2*(X(0)*X(2)-X(1)*X(3)))*X(3)))*HZ,
        
        ((2*X(1))-(2*(X(0)*X(1)-X(2)*X(3))*X(0)))*HX + ((2*-X(0))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(0)))*HY + ((2*X(3))-(2*(X(1)*X(2)+X(0)*X(3))*X(0)))*HZ,
        ((2*X(0))-(2*(X(0)*X(1)-X(2)*X(3))*X(1)))*HX + ((2*X(1))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(1)))*HY + ((2*X(2))-(2*(X(1)*X(2)+X(0)*X(3))*X(1)))*HZ,
        ((2*-X(3))-(2*(X(0)*X(1)-X(2)*X(3))*X(2)))*HX + ((2*-X(2))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(2)))*HY + ((2*X(1))-(2*(X(1)*X(2)+X(0)*X(3))*X(2)))*HZ,
        ((2*-X(2))-(2*(X(0)*X(1)-X(2)*X(3))*X(3)))*HX + ((2*X(3))-(((X(0)*X(0))-(X(1)*X(1))-(X(2)*X(2))+(X(3)*X(3)))*X(3)))*HY + ((2*X(0))-(2*(X(1)*X(2)+X(0)*X(3))*X(3)))*HZ,
        
        ((2*X(2))-(2*(X(0)*X(2)+X(1)*X(3))*X(0)))*HX + ((2*-X(3))-(2*(X(1)*X(2)-X(0)*X(3))*X(0)))*HY + ((2*-X(0))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(0)))*HZ,
        ((2*X(3))-(2*(X(0)*X(2)+X(1)*X(3))*X(1)))*HX + ((2*X(2))-(2*(X(1)*X(2)-X(0)*X(3))*X(1)))*HY + ((2*-X(1))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(1)))*HZ,
        ((2*X(0))-(2*(X(0)*X(2)+X(1)*X(3))*X(2)))*HX + ((2*X(1))-(2*(X(1)*X(2)-X(0)*X(3))*X(2)))*HY + ((2*X(2))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(2)))*HZ,
        ((2*X(1))-(2*(X(0)*X(2)+X(1)*X(3))*X(3)))*HX + ((2*-X(0))-(2*(X(1)*X(2)-X(0)*X(3))*X(3)))*HY + ((2*X(3))-((-(X(0)*X(0))-(X(1)*X(1))+(X(2)*X(2))+(X(3)*X(3)))*X(3)))*HZ
        

    };

    H = H * Hn;
    
    
    F = {
        1,          -r * t/2,   q * t/2,   p * t/2,
        r * t/2,    1,          -p * t/2,    q * t/2,
        -q * t/2,    p * t/2,   1,          r * t/2,
        -p * t/2,    -q * t/2,    -r * t/2,    1
    };
    

    
     Xpriori = F * X;
    /*
     Xpriori = {clip(Xpriori(0)),
                clip(Xpriori(1)),
                clip(Xpriori(2)),
                clip(Xpriori(3))
     };
    */
     Ppriori = F * P * ~F + Q;
    
     f = H * Ppriori * ~H + R;
    
     K = Ppriori * ~H * Invert(f);

    h = {
        2*g*(Xpriori(0)*Xpriori(2)-Xpriori(1)*Xpriori(3)),
        2*g*(Xpriori(1)*Xpriori(2)+Xpriori(0)*Xpriori(3)),
        g*(-(Xpriori(0)*Xpriori(0))-(Xpriori(1)*Xpriori(1))+(Xpriori(2)*Xpriori(2))+(Xpriori(3)*Xpriori(3))),
        (HX*((Xpriori(0)*Xpriori(0))-(Xpriori(1)*Xpriori(1))-(Xpriori(2)*Xpriori(2))+(Xpriori(3)*Xpriori(3))))+(2*HY*(Xpriori(0)*Xpriori(1)+Xpriori(2)*Xpriori(3)))+(2*HZ*(Xpriori(0)*Xpriori(2)-Xpriori(1)*Xpriori(3))),
        (2*HX*(Xpriori(0)*Xpriori(1)-Xpriori(2)*Xpriori(3)))+(HY*(-(Xpriori(0)*Xpriori(0))+(Xpriori(1)*Xpriori(1))-(Xpriori(2)*Xpriori(2))+(Xpriori(3)*Xpriori(3)))+(2*HZ*(Xpriori(1)*Xpriori(2)+Xpriori(0)*Xpriori(3)))),
        (2*HX*(Xpriori(0)*Xpriori(2)+Xpriori(1)*Xpriori(3)))+(2*HY*(Xpriori(1)*Xpriori(2)-Xpriori(0)*Xpriori(3)))+(HZ*(-(Xpriori(0)*Xpriori(0))-(Xpriori(1)*Xpriori(1))+(Xpriori(2)*Xpriori(2))+(Xpriori(3)*Xpriori(3))))
    };

    hn = {1/sqrt((Xpriori(0))*(Xpriori(0))+(Xpriori(1))*(Xpriori(1))+(Xpriori(2))*(Xpriori(2))+(X(3))*(Xpriori(3)))};
    
    h = h * hn;
    
    X = Xpriori + K * (z - h);
   
    X = {clip(X(0)),
        clip(X(1)),
        clip(X(2)),
        clip(X(3))
    };
   
    P = (I4 - K * H) * Ppriori;
    
    
}



float JHFilter::clip(float n) {
    return fmax(-1, fmin(n, 1));
}



float JHFilter::getRoll(){
    float roll = atan2(2 * (X(3) * X(0) + X(1) * X(2)), 1 - 2 * (X(0) * X(0) + X(1) * X(1)));
    
    return roll * 180 / 3.141592;
}
float JHFilter::getPitch(){
    float pitch = asin(2 * (X(3) * X(1) - X(0) * X(2)));
    
    if (2 * (X(3) * X(1) - X(0) * X(2))<-1) {
        pitch = -3.141592/2;
    }
    
    return pitch * 180 / 3.141592;
}
float JHFilter::getYaw(){
    float yaw = atan2(2 * (X(3) * X(2) + X(0) * X(1)), 1 - 2 * (X(1) * X(1) + X(2) * X(2)));
    
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


