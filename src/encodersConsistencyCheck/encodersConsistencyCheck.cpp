// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <encodersConsistencyCheck.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCubTestsSupport;

// prepare the plugin
PREPARE_PLUGIN(encodersConsistencyCheck)

encodersConsistencyCheck::encodersConsistencyCheck() : YarpTestCase("encodersConsistencyCheck") {
    jointsList=0;

    dd=0;

    ienc=0;
    imot=0;
    imotenc=0;
   
    enc_jnt=0;
    enc_jnt2mot=0;
    enc_mot=0;

    cycles =10;
    position_move_tolerance = 1.0;

}

encodersConsistencyCheck::~encodersConsistencyCheck() { }

bool encodersConsistencyCheck::setup(yarp::os::Property& property) {

    char b[5000];
    strcpy (b,property.toString().c_str());
    RTF_TEST_REPORT("on setup()");
    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),      "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("max"),       "The max position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("min"),       "The min position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"),     "The positionMove reference speed must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "The tolerance of differenc between motor position and calulated motor position pay joint position. Must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("pos_move_tolerance"), "The tolerance of position move. Must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse home parameter");

    Bottle* maxBottle = property.find("max").asList();
    RTF_ASSERT_ERROR_IF(maxBottle!=0,"unable to parse max parameter");

    Bottle* minBottle = property.find("min").asList();
    RTF_ASSERT_ERROR_IF(minBottle!=0,"unable to parse min parameter");

    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF(speedBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>=0,"invalid tolerance");

    position_move_tolerance = property.find("pos_move_tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>=0,"invalid tolerance");


    //optional parameters
    if (property.check("cycles"))
    {cycles = property.find("cycles").asInt();}

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/icub/"+partName);
    options.put("local", "/positionDirectTest/icub/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(imot),"Unable to open motor interface");
    RTF_ASSERT_ERROR_IF(dd->view(imotenc),"Unable to open motor interface");
    RTF_ASSERT_ERROR_IF(dd->view(iremvar),"Unable to open remote variables interface");

    //prova: leggo encoders
//    RTF_TEST_REPORT("sto per leggere encoders");
//    yarp::os::Bottle valenc;
//    if(!iremvar-> getRemoteVariable("encoders", valenc))
//    {
//         RTF_ASSERT_ERROR("unable to get enc!");
//    }
//    else
//    {
//        RTF_TEST_REPORT(valenc.toString().c_str());
//        RTF_ASSERT_ERROR("esco");
//    }

    //leggi matrice;
    RTF_TEST_REPORT("sto per leggere la matrice");
    yarp::os::Bottle val;
    if(!iremvar-> getRemoteVariable("kinematic_mj", val))
    {
         RTF_ASSERT_ERROR("unable to get the coupling matrix!");
    }
    else
    {
        RTF_TEST_REPORT(val.toString().c_str());
        matrix.resize(4,4);
        int k=0;
        for(int r=0; r<4; r++)
        {
            for(int c=0; c<4 && k<16; c++)
            {
                matrix[r][c] = val.get(0).asList()->get(k).asDouble();
                k++;
            }
        }
        RTF_TEST_REPORT("ecco la matrice letta!");
        RTF_ASSERT_ERROR(matrix.toString());
    }



    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF((n_cmd_joints>0 && n_cmd_joints<=n_part_joints),"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_cmd_joints); enc_jnt.zero();
    enc_mot.resize(n_cmd_joints); enc_mot.zero();
//    vel_jnt.resize(n_cmd_joints); vel_jnt.zero();
//    vel_mot.resize(n_cmd_joints); vel_mot.zero();
//    acc_jnt.resize(n_cmd_joints); acc_jnt.zero();
//    acc_mot.resize(n_cmd_joints); acc_mot.zero();


    max.resize(n_cmd_joints);     for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min.resize(n_cmd_joints);     for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    home.resize(n_cmd_joints);    for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints);   for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();
    gearbox.resize(n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) {double t; int b=imot->getGearboxRatio(i,&t); gearbox[i]=t;}

    jMotion = jointsPosMotion(dd, jointsList);
    jMotion.setTolerance(position_move_tolerance);
    jMotion.setTimeout(100);

    RTF_ASSERT_ERROR_IF(jMotion.setPositionMode(), "Unable to set position mode");

    jMotion.setSpeed(speed);
    return true;
}

void encodersConsistencyCheck::tearDown()
{
    RTF_TEST_REPORT("Closing test module");

    RTF_ASSERT_ERROR_IF(jMotion.goTo(home), "Unable to go in home");

    if (dd) {delete dd; dd =0;}
}



void encodersConsistencyCheck::run()
{/*
    char buff [500];
    int  cycle=0;


    RTF_ASSERT_ERROR_IF(jMotion.goTo(home), "Unable to go in home");

    bool go_to_max=false;
    RTF_ASSERT_ERROR_IF(jMotion.goTo(min),"Unable to go to min");

    yarp::sig::Vector tmp_vector (n_part_joints);
    while (cycle<=cycles)
    {
        bool ret = true;
        ret = ienc->getEncoders(tmp_vector.data());                  for (unsigned int i = 0; i < jointsList.size(); i++) enc_jnt[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoders returned false");
        ret = imotenc->getMotorEncoders(tmp_vector.data());             for (unsigned int i = 0; i < jointsList.size(); i++) enc_mot[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoder returned false");

//        ret = ienc->getEncoderSpeeds(tmp_vector.data());             for (unsigned int i = 0; i < jointsList.size(); i++) vel_jnt[i] = tmp_vector[jointsList(i)];
//        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoderSpeeds returned false");
//        ret = imotenc->getMotorEncoderSpeeds(tmp_vector.data());        for (unsigned int i = 0; i < jointsList.size(); i++) vel_mot[i] = tmp_vector[jointsList(i)];
//        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoderSpeeds returned false");
//        ret = ienc->getEncoderAccelerations(tmp_vector.data());      for (unsigned int i = 0; i < jointsList.size(); i++) acc_jnt[i] = tmp_vector[jointsList(i)];
//        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoderAccelerations returned false");
//        ret = imotenc->getMotorEncoderAccelerations(tmp_vector.data()); for (unsigned int i = 0; i < jointsList.size(); i++) acc_mot[i] = tmp_vector[jointsList(i)];
//        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoderAccelerations returned false");


        enc_jnt2mot = matrix * enc_jnt;
//        vel_jnt2mot = matrix * vel_jnt;
//        acc_jnt2mot = matrix * acc_jnt;
        for (unsigned int i = 0; i < jointsList.size(); i++) enc_jnt2mot[i] = enc_jnt2mot[i] * gearbox[jointsList(i)];
//        for (unsigned int i = 0; i < jointsList.size(); i++) vel_jnt2mot[i] = vel_jnt2mot[i] * gearbox[jointsList(i)];
//        for (unsigned int i = 0; i < jointsList.size(); i++) acc_jnt2mot[i] = acc_jnt2mot[i] * gearbox[jointsList(i)];


        for (unsigned int i = 0; i < jointsList.size(); i++)
        {
            if(((enc_jnt2mot[i] < enc_mot[i]-tolerance) || (enc_jnt2mot[i] > enc_mot[i]+tolerance)))
            {
                sprintf(buff, "j %d: encoder on motor(%.2f) isn't consistency on encoder on joint(%2.f)",i, enc_mot[i], enc_jnt2mot[i] );
                RTF_ASSERT_ERROR(buff);
            }
        }



        sprintf(buff, "Test cycle %d/%d", cycle, cycles); RTF_TEST_REPORT(buff);
        if (go_to_max == false)
        {

            RTF_ASSERT_ERROR_IF(jMotion.goTo(max), "Unable to go to MAX");
            go_to_max = true;
        }
        else
        {

            RTF_ASSERT_ERROR_IF(jMotion.goTo(min), "Unable to go to min");
            go_to_max = false;
        }

        cycle++;
    }

    RTF_TEST_REPORT("End cicles");
*/
}
