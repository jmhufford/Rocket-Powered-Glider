//
//  JHFilter.cpp
//  Rocket Powered Glider
//
//  Created by Jerome - UCF on 2/25/19.
//

#include "JHFilter.h"
#include "BasicLinearAlgebra.h"
#include <iostream>

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

BLA::Matrix<6, 6, Diagonal<6, float> > z6;
BLA::Matrix<12, 12, Diagonal<12, float> > z12;
BLA::Matrix<6,6> I6;
BLA::Matrix<12,12> I12;
BLA::Matrix<6,6> R;
BLA::Matrix<12,12> Q;
BLA::Matrix<12,12> F;
BLA::Matrix<6,12> H;

//declare function vars
BLA::Matrix<12> X;
BLA::Matrix<12> Xpriori;
BLA::Matrix<12,12> Ppriori;
BLA::Matrix<6,6> f;
BLA::Matrix<12,6> K;
BLA::Matrix<12,12> P;
BLA::Matrix<6> z;


JHFilter::JHFilter(){
    
};



void JHFilter::begin(float hz, float RCV, float QCV){
    float t = 1.0f / hz;
    
    I6 = z6.Fill(1);
    I12 = z12.Fill(1);
    P = I12;
    R = z6.Fill(RCV);
    Q = z12.Fill(QCV);
    F = {
        1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    };
    H = {
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
    };
    
}

void JHFilter::zeroAngles(float ax,float ay,float az){
    float g = sqrt(ax*ax+ay*ay+az*az);
    float theta = asin(-ax/g);
    float phi = acos(az/(g*cos(theta)));
    
    //initial conditions
    X = {
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



}

void JHFilter::logRPY() {
    std::cout << "Roll: " << getRoll() << ", Pitch: " << getPitch() << ", Yaw: " << getYaw() << "\n";
}

void JHFilter::updateIMU(float ax, float ay, float az, float p, float q, float r){
    
    z = {ax,ay,az,p,q,r};
    
    Xpriori = F * X;
    Ppriori = F * P * ~F + Q;
    f = H * Ppriori * ~H + R;
    K = Ppriori * ~H * Invert(f);
    X = Xpriori + K * (z - (H * Xpriori));
    P = (I12 - K * H) * Ppriori;
    
}
float JHFilter::getRoll(){
    return X(3);
}
float JHFilter::getPitch(){
    return X(4);
}
float JHFilter::getYaw(){
    return X(5);
}
float JHFilter::getAx(){
    return X(6);
}
float JHFilter::getAy(){
    return X(7);
}
float JHFilter::getAz(){
    return X(8);
}
float JHFilter::getP(){
    return X(9);
}
float JHFilter::getQ(){
    return X(10);
}
float JHFilter::getR(){
    return X(11);
}
