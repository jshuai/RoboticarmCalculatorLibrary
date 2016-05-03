/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ForwardKinematics.cpp
 * Author: fulop
 * 
 * Created on May 3, 2016, 9:07 AM
 */

#include "ForwardKinematics.h"

ForwardKinematics::ForwardKinematics() {
}

ForwardKinematics::ForwardKinematics(const ForwardKinematics& orig) {
}

ForwardKinematics::~ForwardKinematics() {
}

/**
 * Singleton getter.
 * @return The one and only instance of the class.
 */
ForwardKinematics& ForwardKinematics::getInstance(){
    static ForwardKinematics instance;
    return instance;
}

void ForwardKinematics::storeDHTMatrix(DenavitHartenbergMatrix* matrix){
    this->matrices.emplace(matrices.end(), matrix);
}

fmat ForwardKinematics::getEffectorPosition(){
    return this->effectorPosition;
}

void ForwardKinematics::setRefreshRate(int hertz){
    this->calcFrequency = hertz;
}

void ForwardKinematics::setAutorun(bool enabled){
    
    // handle only state change
    if(this->autorun == enabled)return;
    
    // disabled -> enabled
    if(enabled == true){
        this->autorun = true;
        this->calcThread = thread(calculateEffectorPosition);
    }else{
        // enabled -> disabled
        this->autorun = false;
        this->calcThread.~thread();
    }
    
}

void ForwardKinematics::calculateEffectorPosition(){
    
    // get the current instance
    ForwardKinematics& inst = ForwardKinematics::getInstance();
    unsigned int sleep_time = ((1.0/inst.calcFrequency)*1000);
    
    std::cout << "Sleep time is " << sleep_time << " ms" << std::endl;
    
    // refresh data
    while(true){
        
        
        for(DenavitHartenbergMatrix* m : inst.matrices){
            m->calcValue();
        }
        
        std::cout << "DHT matrixes have been refreshed." << std::endl;
        
        // refresh effector pos
        inst.refreshEffectorPosition();
        
        std::cout << "Effector position refreshed." << std::endl;
        
        // break if autorun disabled
        if(inst.autorun == false){
            std::cout << "Not in auto-run mode." << std::endl;
            break;
        }
        
        std::cout << "Auto-run enabled.. now we are goind to sleep." << std::endl;
        
        // autorun enabled, wait after iteration
        if(inst.autorun)this_thread::sleep_for(chrono::milliseconds(sleep_time));
    }
    
    std::cout << "End of while." << std::endl;
    
}

/**
 * Multiplies the DHT matrices and sets the position of the effector.
 */
void ForwardKinematics::refreshEffectorPosition(){
    
    
    
    std::cout << "number of matrices: " << matrices.size() << std::endl;
    
    fmat* origin_matrix = matrices.front()->getMatrix();
    
    
    std::cout << *origin_matrix;
    
    std::cout << "Starting iterative multiplication..." << std::endl;
    
    for(std::vector<DenavitHartenbergMatrix*>::iterator it = matrices.begin() + 1; it != matrices.end(); ++it){
       arma::fmat* temp = (*it)->getMatrix();
       *origin_matrix = *origin_matrix * *temp;
       //std::cout << *origin_matrix << std::endl;
       //std::cout << "------------------------------------------------------" << std::endl;
    }
    
    std::cout << "Matrix multiplication done." << std::endl;
    
    // update effector position
    // create vector
    arma::fmat v = fmat(4,1);
    v.at(0,0) = 0; 
    v.at(1,0) = 0;
    v.at(2,0) = 0;
    v.at(3,0) = 1;
   
    
    effectorPosition =  *origin_matrix * v;

}