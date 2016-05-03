/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   DenavitHartenbergTest.h
 * Author: fulop
 *
 * Created on May 3, 2016, 12:18:48 PM
 */

#ifndef DENAVITHARTENBERGTEST_H
#define DENAVITHARTENBERGTEST_H


#include <cppunit/extensions/HelperMacros.h>
#define _USE_MATH_DEFINES


class DenavitHartenbergTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(DenavitHartenbergTest);

    CPPUNIT_TEST(J1_test);

    CPPUNIT_TEST_SUITE_END();

public:
    DenavitHartenbergTest();
    virtual ~DenavitHartenbergTest();
    void setUp();
    void tearDown();

private:
    void J1_test();
};

#endif /* DENAVITHARTENBERGTEST_H */

