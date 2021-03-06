// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _POSITIONDIRECT_H_
#define _POSITIONDIRECT_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
* This tests checks the positionDirect control, sending a sinusoidal reference signal, with parametric frequency and amplitude.
* The sample time, typical in the range of 10 ms, can be also be adjusted by the user.
* This test currently does not return any error report. It simply moves a joint, and the user visually evaluates the smoothness of the performed trajectory. In the future automatic checks/plots may be added to the test.
* Be aware theat may exists set of parameters (e.g. high values of sample time / ampiltude /frequency) that may lead to PID instability and damage the joint.
* The test is able to check all the three types of yarp methods (single joint, multi joint, all joints), depending on the value of cmdMode Parameter.

* example: testRunner -v -t PositionDirect.dll - p "--robot icub --part head --joints ""(0 1 2)"" --zero 0 --frequency 0.8 --amplitude 10.0 --cycles 10 --tolerance 1.0 --sampleTime 0.010 --cmdMode 0"
* example: testRunner -v -t PositionDirect.dll - p "--robot icub --part head --joints ""(2)"" --zero 0 --frequency 0.4 --amplitude 10.0 --cycles 10 --tolerance 1.0 --sampleTime 0.010 --cmdMode 0"

* Check the following functions:
* \li IPositionDirect::setPositions()
* \li IControlMode2::setControlMode()
*
*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -     | Yes | The name of the robot.     | e.g. icub |
* | part               | string | -     | -     | Yes | The name of trhe robot part. | e.g. left_arm |
* | joints             | vector of ints | - | - | Yes | List of joints to be tested | |
* | zero               | double | deg   | -     | Yes | The home position for each joint | |
* | cycles             | int    | -     | -     | Yes | The number of test cycles (going from max to min position and viceversa) |   |
* | frequency          | double | deg   | -     | Yes | The frequency of the sine reference signal | |
* | amplitude          | double | deg   | -     | Yes | The ampiltude of the sine reference signal | |
* | tolerance          | double | deg   | -     | Yes | The tolerance used when moving from min to max reference position and viceversa | |
* | sampleTime         | double | s     | -     | Yes | The sample time of the control thread | |
* | cmdMode            | int    | deg   | -     | Yes | = 0 to test single joint method, = 1 to test all joints, = 2 to test multi joint method | |
*
*/

class PositionDirect : public YarpTestCase {
public:
    PositionDirect();
    virtual ~PositionDirect();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void executeCmd();
    void setMode(int desired_mode);

private:
    std::string robotName;
    std::string partName;
    int* jointsList;
    double frequency;
    double amplitude;
    double cycles;
    double tolerance;
    double sampleTime;
    double zero;
    int    n_part_joints;
    int    n_cmd_joints;
    enum cmd_mode_t
    {
      single_joint = 0,
      all_joints = 1,
      some_joints =2
    } cmd_mode;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPositionDirect   *idir;

    double  cmd_single;
    double* cmd_tot;
    double* cmd_some;

    double* pos_tot;

    double prev_cmd;
};

#endif //_PositionDirect_H
