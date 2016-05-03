/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DenavitHartenbergMatrix.h
 * Author: fulop
 *
 * Created on May 3, 2016, 9:20 AM
 */

#ifndef DENAVITHARTENBERGMATRIX_H
#define DENAVITHARTENBERGMATRIX_H

#include <armadillo>
#include <memory>
#include <cmath> 
#include "../RoboticArmPartLibrary/Joint.h"


class DenavitHartenbergMatrix {
    
public:
    
    /* The invocable parameterized constructor */
    DenavitHartenbergMatrix(float alpha, float a, float theta, float d);
    
    DenavitHartenbergMatrix(const DenavitHartenbergMatrix& orig);
    
    virtual ~DenavitHartenbergMatrix();
    
    /* Calculates the Matrix value based on the current angles/distances*/
    void calcValue();
    
    /* Assign the DHT matrix to the given joint */
    void assignToJoint(std::shared_ptr<RoboticArm::Joint> joint);
    
    /* Returns the pointer to the current matrix with the current values */
    arma::fmat* getMatrix();
    
    
private:
    
    /* Attached joint*/
    std::shared_ptr<RoboticArm::Joint> joint;
    
    /* Denavit-Hartenberg parameters */
    float alpha, a, theta, d;
    
    /* The DHT result matrix */
    arma::fmat matrix;
    
    /* Default constructor is hidden */
    DenavitHartenbergMatrix();
    
    void test();
    

};

#endif /* DENAVITHARTENBERGMATRIX_H */

