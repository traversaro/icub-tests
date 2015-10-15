// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/TestAssert.h>
#include <yarp/os/Time.h>
#include <icub_tests_support/jointsPosMotion.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace iCubTestsSupport;


jointsPosMotion::jointsPosMotion(){}

jointsPosMotion::jointsPosMotion(yarp::dev::PolyDriver *polydriver)
{
    jointsList = 0;
    n_joints = 0;
    encoders = 0;
    speed = 0;

    tolerance = 1.0;
    timeout = 100;

    dd = polydriver;
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");

}

jointsPosMotion::jointsPosMotion(yarp::dev::PolyDriver *polydriver, yarp::sig::Vector &jlist)
{

    n_joints = jlist.size();
    jointsList.resize(n_joints);
    jointsList = jlist;
    encoders.resize(n_joints); encoders.zero();
    speed = yarp::sig::Vector(n_joints, 10.0);


    tolerance = 1.0;
    timeout = 100;

    dd = polydriver;
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");

    //send default speed
    for (unsigned int i=0; i<n_joints; i++)
    {
        ipos->setRefSpeed((int)jointsList[i],speed[i]);
    }
}

jointsPosMotion::~jointsPosMotion()
{
   // if (dd) {delete dd; dd =0;}
}


void jointsPosMotion::setTolerance(double tolerance) {tolerance = tolerance;}

void jointsPosMotion::setJointsList(yarp::sig::Vector &jlist)
{
    jointsList = jlist;
    n_joints = jlist.size();
    encoders.resize(n_joints); encoders.zero();
    speed.resize(n_joints); speed.zero();
}
bool jointsPosMotion::setPositionMode()
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode((int)jointsList[i],VOCAB_CM_POSITION);
        iimd->setInteractionMode((int)jointsList[i],VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.010);
    }

    int cmode;
    yarp::dev::InteractionModeEnum imode;
    int timeout_count = 0;

    while (1)
    {
        int ok=0;
        for (unsigned int i=0; i<n_joints; i++)
        {
            icmd->getControlMode ((int)jointsList[i],&cmode);
            iimd->getInteractionMode((int)jointsList[i],&imode);
            if (cmode==VOCAB_CM_POSITION && imode==VOCAB_IM_STIFF) ok++;
        }
        if (ok==n_joints) break;
        if (timeout_count>timeout)
        {
            RTF_ASSERT_ERROR("Unable to set control mode/interaction mode");
        }
        yarp::os::Time::delay(0.2);
        timeout_count++;
    }

}

void jointsPosMotion::setTimeout(double timeout){timeout = timeout;}

int jointsPosMotion::getIndexOfJoint(int j)
{
    for(int i=0; i<n_joints; i++)
    {
        if(jointsList[i] == j)
            return i;
    }
    return jointsList.size()+1;
}

void jointsPosMotion::setSpeed(yarp::sig::Vector &speedlist)
{
    RTF_ASSERT_ERROR_IF((speedlist.size() != jointsList.size()), "speed list has a different size of joint list");
    speed = speedlist;
    for (unsigned int i=0; i<n_joints; i++)
    {
        ipos->setRefSpeed((int)jointsList[i],speed[i]);
    }
}



bool jointsPosMotion::goToSingle(int j, double pos, double *reached_pos)
{
    int i = getIndexOfJoint(j);
    RTF_ASSERT_ERROR_IF(i<n_joints, "cannot move a joint not in list.");

    ipos->positionMove((int)jointsList[i],pos);
    double tmp=0;

    int timeout_count = 0;
    bool ret = true;
    while (1)
    {
        ienc->getEncoder((int)jointsList[i],&tmp);
        if (fabs(tmp-pos)<tolerance)
            break;

        if (timeout_count>timeout)
        {
            ret  = false;
            break;
        }

        yarp::os::Time::delay(0.2);
        timeout_count++;
    }

    if(reached_pos != NULL)
    {
        *reached_pos = tmp;
    }
    return(ret);
}


bool jointsPosMotion::goTo(yarp::sig::Vector position, yarp::sig::Vector *reached_pos)
{
    for (unsigned int i=0; i<n_joints; i++)
    {
        ipos->positionMove((int)jointsList[i],position[i]);
    }

    int timeout_count = 0;
    yarp::sig::Vector tmp(n_joints);tmp.zero();
    bool ret = true;

    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<n_joints; i++)
        {
            ienc->getEncoder((int)jointsList[i],&tmp[i]);
            if (fabs(tmp[i]-position[i])<tolerance)
                in_position++;
        }
        if (in_position==n_joints)
            break;

        if (timeout_count>timeout)
        {
            ret = false;
            break;
        }
        yarp::os::Time::delay(0.2);
        timeout_count++;
    }

    if(reached_pos != NULL)
    {
        reached_pos->resize(n_joints);
        *reached_pos = tmp;
    }
    return(ret);
}
