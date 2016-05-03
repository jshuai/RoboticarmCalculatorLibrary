/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DenavitHartenbergMatrix.cpp
 * Author: fulop
 * 
 * Created on May 3, 2016, 9:20 AM
 */

#include "DenavitHartenbergMatrix.h"

using namespace arma;

DenavitHartenbergMatrix::DenavitHartenbergMatrix() {
}

DenavitHartenbergMatrix::DenavitHartenbergMatrix(const DenavitHartenbergMatrix& orig) {
    
    this->a = orig.a;
    this->alpha = orig.alpha;
    this->d = orig.d;
    this->theta = orig.theta;
    
    this->joint = orig.joint;
    
    this->matrix = orig.matrix;
    
}

DenavitHartenbergMatrix::~DenavitHartenbergMatrix() {
}

/**
 * Parameterized constructor.
 * @param alpha
 * @param a
 * @param theta
 * @param d
 */
DenavitHartenbergMatrix::DenavitHartenbergMatrix(float alpha, float a, float theta, float d) {

    this->alpha = alpha;
    this->a = a;
    this->d = d;
    this->theta = theta;

    // create matrix, fill with zeros
    this->matrix = zeros<fmat>(4,4);
    
}

/**
 * Calculates and sets the current value of the DHT matrix.
 * 
 * DHT Matrix:
 * 
 *  [   cos(theta)  -sin(theta)cos(alpha)     sin(theta)sin(alpha)      a*cos(theta)    ]
 *  [   sin(theta)   cos(theta)cos(alpha)    -cos(theta)sin(alpha)      a*sin(theta)    ]
 *  [     0             sin(alpha)                  cos(alpha)              d           ]
 *  [     0                 0                           0                   1           ]
 * 
 */
void DenavitHartenbergMatrix::calcValue(){
     
    try{
        // update current values from joint
        if(this->joint != nullptr){
            this->theta = this->joint->getAngle();
            this->d = this->joint->getLength();
        }
       
        
        // convert to radian
        float alpa_rad = alpha * M_PI / 180;
        float theta_rad = (theta + theta_offset) * M_PI / 180;
        
        // pre-calc values to avoid redundant calculation
        float cos_alpha = cos(alpa_rad);
        float sin_alpha = sin(alpa_rad);
        float cos_theta = cos(theta_rad);
        float sin_theta = sin(theta_rad);
        
        
        // calculate values
        matrix.at(0,0) = cos_theta;
        matrix.at(0,1) = -sin_theta * cos_alpha;
        matrix.at(0,2) = sin_theta * sin_alpha;
        matrix.at(0,3) = a * cos_theta;
        
        matrix.at(1,0) = sin_theta;
        matrix.at(1,1) = cos_theta * cos_alpha;
        matrix.at(1,2) = -cos_theta * sin_alpha;
        matrix.at(1,3) = a * sin_theta;
    
        matrix.at(2,1) = sin_alpha;
        matrix.at(2,2) = cos_alpha;
        matrix.at(2,3) = d + d_offset;
        
        matrix.at(3,3) = 1;
        
    }
    catch(std::logic_error e){
        std::cout << e.what() << std::endl;
    }
    
    
}

/* returns the DHT matrix */
arma::fmat* DenavitHartenbergMatrix::getMatrix(){
    return &matrix;
}

void DenavitHartenbergMatrix::test(){

    using namespace arma;
    
    arma::mat A = randu<mat>(4,5);
    mat B = randu<mat>(4,5);
  
    std::cout << A*B.t() << std::endl;

}

/**
 * Sets the offset to the given value for the given variable.
 * @param v
 * @param offset_value
 */
void DenavitHartenbergMatrix::setOffset(OFFSET_VARIABLE v, float offset_value){
    
    switch(v){
        
        case OFFSET_VARIABLE::THETA : this->theta_offset = offset_value;
        break;
        
        case OFFSET_VARIABLE::D :   this->d_offset = offset_value;
        break;
    
    }

}


void DenavitHartenbergMatrix::assignToJoint(std::shared_ptr<RoboticArm::Joint> joint){
    this->joint = joint;
}