// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _JOINTSPOSMOVE_H_
#define _JOINTSPOSMOVE_H_

#include <string>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
namespace iCubTestsSupport{

class jointsPosMotion{
public:
    jointsPosMotion();
    jointsPosMotion(yarp::dev::PolyDriver *polydriver);
    jointsPosMotion(yarp::dev::PolyDriver *polydriver, yarp::sig::Vector &jlist);
    virtual ~jointsPosMotion();

    void setTolerance(double tolerance);
    void setJointsList(yarp::sig::Vector &jlist);
    bool setPositionMode();
    void setTimeout(double timeout);
    void setSpeed(yarp::sig::Vector &speedlist);
    bool goToSingle(int j, double pos, double *reached_pos=NULL);
    bool goTo(yarp::sig::Vector position, yarp::sig::Vector *reached_pos=NULL);


private:
    int getIndexOfJoint(int j);
    yarp::sig::Vector jointsList;
    yarp::sig::Vector encoders;
    yarp::sig::Vector speed;
    double tolerance;
    double timeout;
    int    n_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;

};
}
#endif //_JOINTSPOSMOVE_H_
