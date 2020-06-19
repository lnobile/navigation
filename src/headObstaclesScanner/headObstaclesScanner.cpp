﻿/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define _USE_MATH_DEFINES

#include "extendedRangefinder2DWrapper.h"
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/LogStream.h>

#include <cmath>
#include <sstream>
#include <iostream>
#include <string>
#include <algorithm>

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;

/**
  * It reads the data from a rangefinder sensor and sends them on one port.
  * It also creates one rpc port.
  */

extendedRangefinder2DWrapper::extendedRangefinder2DWrapper() : PeriodicThread(DEFAULT_THREAD_PERIOD_2D),
    partName("extendedRangefinder2DWrapper"),
    sens_p(nullptr),
    iTimed(nullptr),
    _period(DEFAULT_THREAD_PERIOD_2D),
    minAngle(0),
    maxAngle(0),
    minDistance(0),
    maxDistance(0),
    resolution(0),
    isDeviceOwned(false),
    // init ROS data
    useROS(ROS_disabled),
    frame_id(""),
    rosNodeName(""),
    rosTopicName(""),
    rosTopicNameMod(""),
    rosNode(nullptr),
    rosMsgCounter(0),
    rosMsgCounterMod(0),
    transformClientInt(nullptr),
    verbose(false)
{
    yarp::sig::Matrix a(1,3);
    a.zero();
    a(0,2) = yarp::os::Time::now();
    for (int i=0;i<20;i++)
    {
        transformMatStorage.push_back(a);
    }


}

extendedRangefinder2DWrapper::~extendedRangefinder2DWrapper()
{
    sens_p = nullptr;
}

bool extendedRangefinder2DWrapper::checkROSParams(yarp::os::Searchable &config)
{
    // check for ROS parameter group
    if (!config.check("ROS"))
    {
        useROS = ROS_disabled;
        yInfo() << "No ROS group found in config file ... skipping ROS initialization.";
        return true;
    }

    yInfo() << "ROS group was FOUND in config file.";

    Bottle &rosGroup = config.findGroup("ROS");
    if (rosGroup.isNull())
    {
        yError() << partName << "ROS group params is not a valid group or empty";
        useROS = ROS_config_error;
        return false;
    }

    // check for useROS parameter
    if (!rosGroup.check("useROS"))
    {
        yError() << partName << " cannot find useROS parameter, mandatory when using ROS message. \n \
                                                        Allowed values are true, false, ROS_only";
        useROS = ROS_config_error;
        return false;
    }
    std::string ros_use_type = rosGroup.find("useROS").asString();
    if (ros_use_type == "false")
    {
        yInfo() << partName << "useROS topic if set to 'false'";
        useROS = ROS_disabled;
        return true;
    }
    else if (ros_use_type == "true")
    {
        yInfo() << partName << "useROS topic if set to 'true'";
        useROS = ROS_enabled;
    }
    else if (ros_use_type == "only")
    {
        yInfo() << partName << "useROS topic if set to 'only";
        useROS = ROS_only;
    }
    else
    {
        yInfo() << partName << "useROS parameter is set to unvalid value ('" << ros_use_type << "'), supported values are 'true', 'false', 'only'";
        useROS = ROS_config_error;
        return false;
    }

    // check for ROS_nodeName parameter
    if (!rosGroup.check("ROS_nodeName"))
    {
        yError() << partName << " cannot find ROS_nodeName parameter, mandatory when using ROS message";
        useROS = ROS_config_error;
        return false;
    }
    rosNodeName = rosGroup.find("ROS_nodeName").asString();  // TODO: check name is correct
    yInfo() << partName << "rosNodeName is " << rosNodeName;

    // check for ROS_topicName parameter
    if (!rosGroup.check("ROS_topicName"))
    {
        yError() << partName << " cannot find ROS_topicName parameter, mandatory when using ROS message";
        useROS = ROS_config_error;
        return false;
    }
    rosTopicName = rosGroup.find("ROS_topicName").asString();
    rosTopicNameMod = rosTopicName + "/nolegs";
    yInfo() << partName << "rosTopicName is " << rosTopicName;
    yInfo() << partName << "rosTopicNameMod is " << rosTopicNameMod;

    // check for frame_id parameter
    if (!rosGroup.check("frame_id"))
    {
        yError() << partName << " cannot find frame_id parameter, mandatory when using ROS message";
        useROS = ROS_config_error;
        return false;
    }
    frame_id = rosGroup.find("frame_id").asString();
    yInfo() << partName << "frame_id is " << frame_id;

    return true;
}

bool extendedRangefinder2DWrapper::initialize_ROS()
{
    bool success = false;
    switch (useROS)
    {
        case ROS_enabled:
        case ROS_only:
        {
            rosNode = new yarp::os::Node(rosNodeName);   // add a ROS node
            if (rosNode == nullptr)
            {
                yError() << " opening " << rosNodeName << " Node, check your yarp-ROS network configuration\n";
                success = false;
                break;
            }
            if (!rosPublisherPort.topic(rosTopicName))
            {
                yError() << " opening " << rosTopicName << " Topic, check your yarp-ROS network configuration\n";
                success = false;
                break;
            }
            if (!rosPublisherPortMod.topic(rosTopicNameMod))
            {
                yError() << " opening " << rosTopicNameMod << " Topic, check your yarp-ROS network configuration\n";
                success = false;
                break;
            }


            success = true;
        } break;

        case ROS_disabled:
        {
            yInfo() << partName << ": no ROS initialization required";
            success = true;
        } break;

        case ROS_config_error:
        {
            yError() << partName << " ROS parameter are not correct, check your configuration file";
            success = false;
        } break;

        default:
        {
            yError() << partName << " something went wrong with ROS configuration, we should never be here!!!";
            success = false;
        } break;
    }
    return success;
}

/**
  * Specify which sensor this thread has to read from.
  */

bool extendedRangefinder2DWrapper::attachAll(const PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        yError("extendedRangefinder2DWrapper: cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(sens_p);
        Idevice2attach->view(iTimed);
    }

    if (nullptr == sens_p)
    {
        yError("extendedRangefinder2DWrapper: subdevice passed to attach method is invalid");
        return false;
    }
    attach(sens_p);

    if(!sens_p->getDistanceRange(minDistance, maxDistance))
    {
        yError() << "Laser device does not provide min & max distance range.";
        return false;
    }

    if(!sens_p->getScanLimits(minAngle, maxAngle))
    {
        yError() << "Laser device does not provide min & max angle scan range.";
        return false;
    }

    if (!sens_p->getHorizontalResolution(resolution))
    {
        yError() << "Laser device does not provide horizontal resolution ";
        return false;
    }

    PeriodicThread::setPeriod(_period);
    return PeriodicThread::start();
}

bool extendedRangefinder2DWrapper::detachAll()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    sens_p = nullptr;
    return true;
}

void extendedRangefinder2DWrapper::attach(yarp::dev::IRangefinder2D *s)
{
    sens_p = s;
}

void extendedRangefinder2DWrapper::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    sens_p = nullptr;
}

bool extendedRangefinder2DWrapper::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    // parse in, prepare out
    int action = in.get(0).asVocab();
    int inter  = in.get(1).asVocab();
    bool ret = false;

    if (inter == VOCAB_ILASER2D)
    {
        if (action == VOCAB_GET)
        {
            int cmd = in.get(2).asVocab();
            if (cmd == VOCAB_DEVICE_INFO)
            {
                if (sens_p)
                {
                    std::string info;
                    if (sens_p->getDeviceInfo(info))
                    {
                        out.addVocab(VOCAB_IS);
                        out.addVocab(cmd);
                        out.addString(info);
                        ret = true;
                    }
                    else
                    {
                        ret = false;
                    }
                }
            }
            else if (cmd == VOCAB_LASER_DISTANCE_RANGE)
            {
                if (sens_p)
                {
                    double max = 0;
                    double min = 0;
                    if (sens_p->getDistanceRange(min, max))
                    {
                        out.addVocab(VOCAB_IS);
                        out.addVocab(cmd);
                        out.addFloat64(min);
                        out.addFloat64(max);
                        ret = true;
                    }
                    else
                    {
                        ret = false;
                    }
                }
            }
            else if (cmd == VOCAB_LASER_ANGULAR_RANGE)
            {
                if (sens_p)
                {
                    double max = 0;
                    double min = 0;
                    if (sens_p->getScanLimits(min, max))
                    {
                        out.addVocab(VOCAB_IS);
                        out.addVocab(cmd);
                        out.addFloat64(min);
                        out.addFloat64(max);
                        ret = true;
                    }
                    else
                    {
                        ret = false;
                    }
                }
            }
            else if (cmd == VOCAB_LASER_ANGULAR_STEP)
            {
                if (sens_p)
                {
                    double step = 0;
                    if (sens_p->getHorizontalResolution(step))
                    {
                        out.addVocab(VOCAB_IS);
                        out.addVocab(cmd);
                        out.addFloat64(step);
                        ret = true;
                    }
                    else
                    {
                        ret = false;
                    }
                }
            }
            else if (cmd == VOCAB_LASER_SCAN_RATE)
            {
                if (sens_p)
                {
                    double rate = 0;
                    if (sens_p->getScanRate(rate))
                    {
                        out.addVocab(VOCAB_IS);
                        out.addVocab(cmd);
                        out.addFloat64(rate);
                        ret = true;
                    }
                    else
                    {
                        ret = false;
                    }
                }
            }
            else
            {
                yError("Invalid command received in extendedRangefinder2DWrapper");
            }
        }
        else if (action == VOCAB_SET)
        {
            int cmd = in.get(2).asVocab();
            if (cmd == VOCAB_LASER_DISTANCE_RANGE)
            {
                if (sens_p)
                {
                    double min = in.get(3).asInt32();
                    double max = in.get(4).asInt32();
                    sens_p->setDistanceRange(min, max);
                    ret = true;
                }
            }
            else if (cmd == VOCAB_LASER_ANGULAR_RANGE)
            {
                if (sens_p)
                {
                    double min = in.get(3).asInt32();
                    double max = in.get(4).asInt32();
                    sens_p->setScanLimits(min, max);
                    ret = true;
                }
            }
            else if (cmd == VOCAB_LASER_SCAN_RATE)
            {
                if (sens_p)
                {
                    double rate = in.get(3).asInt32();
                    sens_p->setScanRate(rate);
                    ret = true;
                }
            }
            else if (cmd == VOCAB_LASER_ANGULAR_STEP)
            {
                if (sens_p)
                {
                    double step = in.get(3).asFloat64();
                    sens_p->setHorizontalResolution(step);
                    ret = true;
                }
            }
            else
            {
                yError("Invalid command received in extendedRangefinder2DWrapper");
            }
        }
        else
        {
            yError("Invalid action received in extendedRangefinder2DWrapper");
        }
    }
    else
    {
        yError("Invalid interface vocab received in extendedRangefinder2DWrapper");
    }

    if (!ret)
    {
        out.clear();
        out.addVocab(VOCAB_FAILED);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr) {
        out.write(*returnToSender);
    }
    return true;
}

bool extendedRangefinder2DWrapper::threadInit()
{
    return true;
}

void extendedRangefinder2DWrapper::setId(const std::string &id)
{
    sensorId=id;
}

std::string extendedRangefinder2DWrapper::getId()
{
    return sensorId;
}


bool extendedRangefinder2DWrapper::open(yarp::os::Searchable &config)
{
    // NECESSARY config PARAMETERS:
    // name             -> full name of the port by extendedRangefinder2DWrapper
    // remoteTC         -> remote port required by FrameTransformClient::open(), if not specified extended functionality is not enabled
    // period           -> period parameter shared by extendedRangefinder2DWrapper and FrameTransformClient

    // NOT NECESSARY config PARAMETERS:
    // localTC          -> local port required by FrameTransformClient::open()
    // refFrame         -> set the target frame to which bodies coordinates detected are referred
    // remRadius   -> set the radius around the torso to be removed from laser scan [m]


    Property params;
    params.fromString(config.toString());


    if (config.check("refFrame"))
        targetFrame = config.find("refFrame").asString();
    else
        targetFrame = "mobile_base_body_link";

    if (config.check("remRadius"))
        remRadius = config.find("remRadius").asDouble();
    else
        remRadius = 0.25;   // removing radius in m

    if (!config.check("remoteTC"))
    {
        yWarning() << "extendedRangefinder2DWrapper: missing 'remoteTC' parameter. Estended functionality not enabled\n";
        extendedFuncEnabled = 0;
    }
    else
    {
        extendedFuncEnabled = 1;
    }

    if (!config.check("period"))
    {
        yError() << "extendedRangefinder2DWrapper: missing 'period' parameter. Check you configuration file\n";
        return false;
    }
    else
        _period = config.find("period").asInt32() / 1000.0;

    if (!config.check("name"))
    {
        yError() << "extendedRangefinder2DWrapper: missing 'name' parameter. Check you configuration file; it must be like:";
        yError() << "   name:         full name of the port, like /robotName/deviceId/sensorType:o";
        return false;
    }
    else
    {
        streamingPortName  = config.find("name").asString();
        streamingPortNameMod = streamingPortName + "/nolegs";
        rpcPortName = streamingPortName + "/rpc:i";
        rpcPortNameMod = streamingPortNameMod + "/rpc:i";
        setId("extendedRangefinder2DWrapper");
    }

    if (config.check("verbose"))
        verbose = true;

    checkROSParams(config);

    if(!initialize_YARP(config) )
    {
        yError() << sensorId << "Error initializing YARP ports";
        return false;
    }


    // call ROS node/topic initialization, if needed
    if (!initialize_ROS())
    {
        return false;
    }DEFAULT_THREAD_PERIOD_2D;

    // connect to transform client and load interface
    if (extendedFuncEnabled)
    {
        Property    pTC;
        pTC.put("device","transformClient");
        if (config.check("localTC"))
            pTC.put("local",config.find("localTC").asString());
        else
            pTC.put("local","/extendedRangefinder2D/transformClient");

        pTC.put("remote",config.find("remoteTC").asString());
        pTC.put("period",config.find("period").asString());

        transformClientDriver.open(pTC);

//        while (transformClientDriver.open(pTC) == false)
//        {
//             yInfo() << "tranformClient waiting to open";
//             yarp::os::Time::delay(10);
//        }

//          yError() << "Unable to connect to transform client";
//          return false;

        transformClientDriver.view(transformClientInt);
        if (transformClientInt == nullptr)
        {
             yError() << "Unable to open Transform Client interface";
             return false;
        }
        yInfo() << "tranformClient successfully open";
    }
    else
    {
        yDebug() << "extendedFuncEnabled=false";
    }
        
    if(config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if(!laserDriver.open(p) || !laserDriver.isValid())
        {
            yError() << "extendedRangefinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&laserDriver, "1");
        if(!attachAll(driverlist))
        {
            yError() << "extendedRangefinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }
        isDeviceOwned = true;
    }

    return true;
}

bool extendedRangefinder2DWrapper::initialize_YARP(yarp::os::Searchable &params)
{
    if(useROS != ROS_only)
    {
        if (!streamingPort.open(streamingPortName))
            {
                yError("extendedRangefinder2DWrapper: failed to open port %s", streamingPortName.c_str());
                return false;
            }
        if (!streamingPortMod.open(streamingPortNameMod))
            {
                yError("extendedRangefinder2DWrapper: failed to open port %s", streamingPortNameMod.c_str());
                return false;
            }
        if (!rpcPort.open(rpcPortName))
            {
                yError("extendedRangefinder2DWrapper: failed to open port %s", rpcPortName.c_str());
                return false;
            }
        //rpcPort.setReader(*this);

        if (!rpcPortMod.open(rpcPortNameMod))
            {
                yError("extendedRangefinder2DWrapper: failed to open port %s", rpcPortNameMod.c_str());
                return false;
            }
        rpcPortMod.setReader(*this);

        if (!streamingPortDebug.open(streamingPortName + "/debug"))
            {
                yError("extendedRangefinder2DWrapper: failed to open port %s", (streamingPortName + "/debug").c_str());
                return false;
            }


    }
    return true;
}

void extendedRangefinder2DWrapper::threadRelease()
{
    streamingPort.interrupt();
    streamingPort.close();
    streamingPortMod.interrupt();
    streamingPortMod.close();
    rpcPort.interrupt();
    rpcPort.close();
    rpcPortMod.interrupt();
    rpcPortMod.close();
    streamingPortDebug.interrupt();
    streamingPortDebug.close();
}

void extendedRangefinder2DWrapper::run()
{
    if (sens_p!=nullptr)
    {
        bool ret = true;

        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        yarp::sig::Vector rangesMod;
        std::vector< std::string > allFrameIds;
        std::vector< std::string > filteredFrameIds;
        //std::vector< std::string > debVect;
        yarp::sig::Matrix transformMat;

        std::string test;
        std::vector< double > x_coord(5);
        std::vector< double > y_coord(5);
        std::vector< double > theta_coord(5,0.0);
        std::vector< double > debVect(18,0.0);
        std::vector< bool > refreshed_strings(20,false);

        //std::vector< double > rho_coord(5);
        double xTorso;
        double yTorso;
        double rhoTorso;
        double thetaTorso;
        double theta_min;
        double theta_max;
        double circ_sect;
        double refr_cont = 0;

        std::size_t index_max;
        std::size_t index_min;

        ret &= sens_p->getRawData(ranges);
        ret &= sens_p->getDeviceStatus(status);
        rangesMod = ranges;

        if (ret)
        {
            if(iTimed)
                lastStateStamp = iTimed->getLastInputStamp();
            else
                lastStateStamp.update(yarp::os::Time::now());

            int ranges_size = ranges.size();

            if (extendedFuncEnabled)
            {
                debVect[17] = -2;
                // READ HUMAN PRESENCE AD ERASE LEGS
                //scan only for /human#/shoulderCenter reference systems
                transformClientInt->getAllFrameIds(allFrameIds);

                for (int i=0; i<allFrameIds.size(); i++)
                {
                    //if (it2->compare(0, 6, "/human") == 0)
                    if ((allFrameIds[i].compare(0, 6, "/human") == 0) && (allFrameIds[i].find("/shoulderCenter")!=string::npos))
                    {
                        filteredFrameIds.push_back(allFrameIds[i]);
                    }
                }


                //creating laser signal with removed legs (corresponding to torso position)
                refreshed_strings.assign(20,false);
                if (filteredFrameIds.size()>0)
                {
                    int num_frame;
                    for (auto it=filteredFrameIds.begin(); it!=filteredFrameIds.end(); it++)
                    {
                        if (verbose)
                            yDebug() << "FRAME: " << *it << ": " ;

                        if (transformClientInt->getTransform(*it, targetFrame, transformMat) == false)
                        {
                            if (verbose)
                                yDebug() << "no transform between: " << *it << " and " << targetFrame;
                            continue;
                        }
                        //yarp::os::Time::delay(0.1);
                        num_frame = stoi(it->substr(6,2));
                        refreshed_strings[num_frame]= true;

                        //std::cout << "TransformMatrix:" << std::endl;
                        if (verbose)
                        {
                            yDebug() << transformMat.toString() ;
                            yDebug() << "num frame" << num_frame;
                        }

                        (transformMatStorage.at(num_frame))(0,0) = transformMat(0,3);
                        (transformMatStorage.at(num_frame))(0,1) = transformMat(1,3);
                        (transformMatStorage.at(num_frame))(0,2) = yarp::os::Time::now();
                    }
                }

                int contframe = 0;
                double time_now = yarp::os::Time::now();
                for(auto it=transformMatStorage.begin(); it!=transformMatStorage.end(); it++)
                {

                    if (((time_now - (*it)(0,2) > 0.02)) && refreshed_strings[contframe] == false)
                    {
                        //yDebug() << "FRAME: " << contframe << " skipped, delta time : " << (time_now - (*it)(0,2)) << " is refreshed: " << refreshed_strings[contframe] ;
                        contframe = contframe + 1;
                        continue;
                    }
                    if (verbose)
                    {
                        yDebug() << "FRAME: " << contframe << " Stored matrix: " << it->toString();
                        yDebug() << "not skipped, time : " << (time_now - (*it)(0,2)) << " is refreshed: " << refreshed_strings[contframe];
                    }

                    xTorso = (*it)(0,0);
                    yTorso = (*it)(0,1);

                    rhoTorso = sqrt(std::pow(xTorso, 2) + std::pow(yTorso, 2));
                    thetaTorso = atan2 (yTorso, xTorso)*180/3.14159;        //degrees  -180 to +180
                    //thetaTorso = thetaTorso - 90;                           // -90 degree to be consistent with the fake laser
                    if (thetaTorso<0)
                        thetaTorso = thetaTorso + 360;                       //degrees  0 to +360
                    //thetaTorso = 360 - thetaTorso;                           //to shift from anticlockwise to clockwise movement

                    if (remRadius > rhoTorso)
                        remRadius = rhoTorso;
                    circ_sect = asin(remRadius/rhoTorso)*180/3.14159;

                    theta_min = thetaTorso - circ_sect ;
                    theta_max = thetaTorso + circ_sect ;

                    if (verbose)
                        yDebug() << "X:" << xTorso <<"Y:" << yTorso<< "Theta:" << thetaTorso <<"Rho:" << rhoTorso <<"remRadius:" << remRadius <<"circ_sect:" << circ_sect << "theta_min:" << theta_min << "theta_max:" << theta_max;

                    index_min = (int) (theta_min / resolution);
                    index_max = (int) (theta_max / resolution);
                    debVect[0] = xTorso;
                    debVect[1] = yTorso;
                    debVect[2] = thetaTorso;
                    debVect[3] = rhoTorso;
                    debVect[4] = circ_sect;
                    debVect[5] = theta_min;
                    debVect[6] = theta_max;
                    debVect[7] = -1;

                    if (theta_min <0 )
                    {
                        theta_min = theta_min + 360;
                        index_min = (int) (theta_min / resolution);
                        index_max = (int) (theta_max / resolution);
                        if (index_max>ranges_size)
                            index_max = ranges_size;
                        for (int i=index_min; i<ranges_size; i++ )
                            if (rangesMod[i] > (rhoTorso - remRadius))
                                rangesMod[i] = std::numeric_limits<double>::infinity();
                        for (int i=0; i<index_max; i++ )
                            if (rangesMod[i] > (rhoTorso - remRadius))
                                rangesMod[i] = std::numeric_limits<double>::infinity();

                    }
                    else if (theta_max > 360 )
                    {
                        theta_max = theta_max - 360;
                        index_min = (int) (theta_min / resolution);
                        index_max = (int) (theta_max / resolution);
                        if (index_min>ranges_size)
                            index_min = ranges_size;
                        for (int i=0; i<index_max; i++ )
                            if (rangesMod[i] > (rhoTorso - remRadius))
                                rangesMod[i] = std::numeric_limits<double>::infinity();
                        for (int i=index_min; i<ranges_size; i++ )
                            if (rangesMod[i] > (rhoTorso - remRadius))
                                rangesMod[i] = std::numeric_limits<double>::infinity();
                    }
                    else
                    {
                        index_min = (int) (theta_min / resolution);
                        index_max = (int) (theta_max / resolution);
                        if (index_max>ranges_size)
                            index_max = ranges_size;
                        for (int i=index_min; i<index_max; i++ )
                            if (rangesMod[i] > (rhoTorso - remRadius))
                                rangesMod[i] = std::numeric_limits<double>::infinity();
                    }

                    debVect[8] = ranges_size;
                    debVect[9] = index_min;
                    debVect[10] = index_max;
                    debVect[11] = theta_min;
                    debVect[12] = theta_max;
                    debVect[13] = -2;

                    if (verbose)
                        yDebug() << "ranges_size:" << ranges_size << " index_min:" << index_min << " index_max:" << index_max << " theta_min:" << theta_min << " theta_max:" << theta_max << " resolution:" << resolution ;

                    contframe = contframe + 1;
                }
            }


            debVect[14] = extendedFuncEnabled;
            debVect[15] = allFrameIds.size();
            debVect[16] = filteredFrameIds.size();


            // publish standard laser port
            yarp::os::Bottle& b = streamingPort.prepare();
            b.clear();
            Bottle& bl = b.addList();

            bl.read(ranges);
            b.addInt32(status);
            streamingPort.setEnvelope(lastStateStamp);
            streamingPort.write();

            // publish modified laser port
            yarp::os::Bottle& bMod = streamingPortMod.prepare();
            bMod.clear();
            Bottle& blMod = bMod.addList();

            blMod.read(rangesMod);
            bMod.addInt32(status);
            streamingPortMod.setEnvelope(lastStateStamp);
            streamingPortMod.write();

            //publish debug port
            yarp::os::Bottle& bd = streamingPortDebug.prepare();
            bd.clear();
            for(int i=0;i<debVect.size();i++)
            {
                bd.addDouble(debVect[i]);
            }

            //bd.addInt32(status);
            streamingPortDebug.setEnvelope(lastStateStamp);
            streamingPortDebug.write();


            // publish ROS topic if required
            if (useROS != ROS_disabled)
            {
                yarp::rosmsg::sensor_msgs::LaserScan &rosData = rosPublisherPort.prepare();
                rosData.header.seq = rosMsgCounter++;
                rosData.header.stamp = lastStateStamp.getTime();
                rosData.header.frame_id = frame_id;

                rosData.angle_min = minAngle * M_PI / 180.0;
                rosData.angle_max = maxAngle * M_PI / 180.0;
                rosData.angle_increment = resolution * M_PI / 180.0;
                rosData.time_increment = 0;             // all points in a single scan are considered took at the very same time
                rosData.scan_time = getPeriod();        // time elapsed between two successive readings
                rosData.range_min = minDistance;
                rosData.range_max = maxDistance;
                rosData.ranges.resize(ranges_size);
                rosData.intensities.resize(ranges_size);

                for (int i = 0; i < ranges_size; i++)
                {
                    rosData.ranges[i] = ranges[i];
                    rosData.intensities[i] = 0.0;
                }
                rosPublisherPort.write();

                yarp::rosmsg::sensor_msgs::LaserScan &rosDataMod = rosPublisherPortMod.prepare();
                rosDataMod.header.seq = rosMsgCounterMod++;
                rosDataMod.header.stamp = lastStateStamp.getTime();
                rosDataMod.header.frame_id = frame_id;

                rosDataMod.angle_min = minAngle * M_PI / 180.0;
                rosDataMod.angle_max = maxAngle * M_PI / 180.0;
                rosDataMod.angle_increment = resolution * M_PI / 180.0;
                rosDataMod.time_increment = 0;             // all points in a single scan are considered took at the very same time
                rosDataMod.scan_time = getPeriod();        // time elapsed between two successive readings
                rosDataMod.range_min = minDistance;
                rosDataMod.range_max = maxDistance;
                rosDataMod.ranges.resize(ranges_size);
                rosDataMod.intensities.resize(ranges_size);

                for (int i = 0; i < ranges_size; i++)
                {
                    rosDataMod.ranges[i] = rangesMod[i];
                    rosDataMod.intensities[i] = 0.0;
                }
                rosPublisherPortMod.write();
            }
        }
        else
        {
            yError("extendedRangefinder2DWrapper: %s: Sensor returned error", sensorId.c_str());
        }

    }
}

bool extendedRangefinder2DWrapper::close()
{
    yTrace("extendedRangefinder2DWrapper::Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    if(rosNode!=nullptr) {
        rosNode->interrupt();
        delete rosNode;
        rosNode = nullptr;
    }

    detachAll();
    return true;
}
