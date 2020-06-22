/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <iostream>

#include <string>
#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

class MyModule : public yarp::os::RFModule
{
    yarp::os::Port handlerPort; // a port to handle messages
    int count;
public:
    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 1.0;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {






        count++;
        std::cout << "[" << count << "]" << " updateModule..." << '\n';
        return true;
    }
    // Message handler. Just echo all received messages.
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        std::cout << "Got something, echo is on" << '\n';
        if (command.get(0).asString() == "quit")
            return false;
        else
            reply = command;
        return true;
    }
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {
        count=0;
        if (!handlerPort.open("/myModule"))
            return false;

        // optional, attach a port to the module
        // so that messages received from the port are redirected
        // to the respond method
        attach(handlerPort);

        Network yarp;


            if (!rf.check("robot"))
            {
                fprintf(stderr, "Please specify the name of the robot\n");
                fprintf(stderr, "--robot name (e.g. icub)\n");
                return 1;
            }
            std::string robotName=rf.find("R1").asString();
            std::string remotePorts="/";
            remotePorts+=robotName;
            remotePorts+="/head";

            std::string localPorts="/test/client";

            Property options;
            options.put("device", "remote_controlboard");
            options.put("local", localPorts);   //local port names
            options.put("remote", remotePorts); //where we connect to

            // create a device
            PolyDriver robotDevice(options);
            if (!robotDevice.isValid()) {
                printf("Device not available.  Here are the known devices:\n");
                printf("%s", Drivers::factory().toString().c_str());
                return 0;
            }

            IPositionControl *pos;
            IEncoders *encs;

            bool ok;
            ok = robotDevice.view(pos);
            ok = ok && robotDevice.view(encs);

            if (!ok) {
                printf("Problems acquiring interfaces\n");
                return 0;
            }

            int nj=0;
            pos->getAxes(&nj);
            Vector encoders;
            Vector command;
            Vector tmp;
            encoders.resize(nj);
            tmp.resize(nj);
            command.resize(nj);

            int i;
            for (i = 0; i < nj; i++) {
                 tmp[i] = 30.0;
            }
            pos->setRefAccelerations(tmp.data());

            for (i = 0; i < nj; i++) {
                tmp[i] = 10.0;
                pos->setRefSpeed(i, tmp[i]);
            }

            // read encoders
            printf("waiting for encoders");
            while(!encs->getEncoders(encoders.data()))
            {
                Time::delay(0.1);
                printf(".");
            }
            printf("\n;");

            command=encoders;

            //now set head initial position
            command[0]=-10;
            command[1]=0;

            pos->positionMove(command.data());

            bool done=false;

            while(!done)
            {
                pos->checkMotionDone(&done);
                Time::delay(0.1);
            }


        return true;
    }
    // Interrupt function.
    bool interruptModule()
    {
        std::cout << "Interrupting your module, for port cleanup" << '\n';
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {

        // optional, close port explicitly
        std::cout << "Calling close function\n";
        handlerPort.close();
        return true;
    }
};

int main(int argc, char * argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;

    // create your module
    MyModule module;

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::cout << "Configuring and starting module.\n";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!module.runModule(rf)) {
        std::cerr << "Error module did not start\n";
    }

    std::cout << "Main returning..." << '\n';
    return 0;
}
