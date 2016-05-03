/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   DenavitHartenbergTest.cpp
 * Author: fulop
 *
 * Created on May 3, 2016, 12:18:48 PM
 */

#include <armadillo_bits/typedef_mat_fixed.hpp>
#include <armadillo_bits/Mat_bones.hpp>

#include "DenavitHartenbergTest.h"


CPPUNIT_TEST_SUITE_REGISTRATION(DenavitHartenbergTest);

DenavitHartenbergTest::DenavitHartenbergTest() {
}

DenavitHartenbergTest::~DenavitHartenbergTest() {
}

void DenavitHartenbergTest::setUp() {
}

void DenavitHartenbergTest::tearDown() {
}

void DenavitHartenbergTest::J1_test() {
    
   using namespace arma;
    
    RoboticArm::Joint* j1 = new RoboticArm::Joint("Joint 1", 1, 1, -170, +170);
    
    DenavitHartenbergMatrix* m1 = new DenavitHartenbergMatrix(90, 5, 0, 10);
    m1->calcValue();
    
    arma::fmat v = fmat(4,1);
    v.at(0,0) = 1; 
    v.at(1,0) = 2;
    v.at(2,0) = 3;
    v.at(3,0) = 1;
    
    auto dht_m = *m1->getMatrix();
    


    auto result =  dht_m * v;
    
    
    std::cout << "DHT is: " << std::endl << dht_m << std::endl;
    std::cout << "Vector is: " << std::endl << v << std::endl;
    std::cout << "Result: " << std::endl << result << std::endl;
    
    
 
    
    
    CPPUNIT_ASSERT(true);
}



