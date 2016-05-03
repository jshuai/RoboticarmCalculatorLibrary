/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ForwardKinematics.h
 * Author: fulop
 *
 * Created on May 3, 2016, 9:07 AM
 */

#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include <vector>
#include "DenavitHartenbergMatrix.h"
#include <armadillo>
#include <thread>
#include <chrono> 

using namespace std;
using namespace arma;

class ForwardKinematics {
    
public:
    
    /* Singleton get method */
    static ForwardKinematics& getInstance();
    
    /* Copy constructor */
    ForwardKinematics(const ForwardKinematics& orig);
    
    /* Destructor */
    virtual ~ForwardKinematics();
    
    // To get the current position of the effector
    fmat getEffectorPosition();
    
    // to store new matrices
    void storeDHTMatrix(DenavitHartenbergMatrix* matrix);
    
    /* Changes the auto-run behavior. */
    void setAutorun(bool enabled);
    
    /* Calculates the current position of the effector */
    static void calculateEffectorPosition();
    
    void refreshEffectorPosition();

    void setRefreshRate(int hertz);
    
private:
    
    /* Default constructor is hidden */
    ForwardKinematics();
    
    // Stores the transformation matrices
    vector<DenavitHartenbergMatrix*> matrices;

    // Stores the position of the effector
    fmat effectorPosition;
    
    // Specifies if the calculation should be done automatically
    bool autorun = false;
    
    // The frequency of the calculation in Hz - maximum value depends on the system
    int calcFrequency = 60;
    
    // Calculator thread
    thread calcThread;
    
    

};

#endif /* FORWARDKINEMATICS_H */

