/*
 * Copyright (C)2017  ICub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <visualization_msgs_MarkerArray.h>
#include <geometry_msgs_PoseStamped.h>
#include <geometry_msgs_PoseWithCovarianceStamped.h>
#include <nav_msgs_OccupancyGrid.h>

#include <math.h>

using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

/**
 * \section localizationModule
 * A module which acts as the server side for a Localization2DClient.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------------------------------------------------------------:|:-----:|
 * | GENERAL        |  module_name   | string  | -              | localizationServer | Yes          | The name of the module use to open ports                          |       |
 * | GENERAL        |  enable_ros    | int     | 0/1            | -                  | Yes          | If set to 1, the module will open the ROS topic specified by ROS::initialpose_topic parameter     |       |
 * | INITIAL_POS    |  initial_x     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_y     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_theta | double  | deg            | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_map   | string  | -              |   -                | Yes          | Name of the map on which localization is performed                | -     |
 * | MAP            |  connect_to_yarp_mapserver   | int  | 0/1     |   -              | Yes          | If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated | -     |
 * | ROS            |  initialpose_topic   | string     | -         |   -              | No           | Name of the topic which will be used to publish the initial pose  | -     |
 * | ROS            |  occupancygrid_topic | string     | -         |   -              | No           | Name of the topic which will be used to publish map data when initial pose is updated         | -     |
 * | TF             |  map_frame_id      | string     | -         |   -                | Yes          | Name of the map reference frame                                   | e.g. /map    |
 * | TF             |  robot_frame_id    | string     | -         |   -                | Yes          | Name of the robot reference frame                                 | e.g. /mobile_base    |
 * | LOCALIZATION   |  use_localization_from_odometry_port    | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a port to receive localization data                         | Incompatible with 'use_localization_from_tf=1'  |
 * | LOCALIZATION   |  use_localization_from_tf      | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a tfClient to receive localization data                     | Incompatible with 'use_localization_from_odometry_port=1 |
 * | ODOMETRY       |  odometry_broadcast_port       | string   |  -   |   -         | Yes          | Full name of port broadcasting the localization data. The server will connect to this port.  | -    |
 */

class localizationModule : public yarp::os::RFModule
{
protected:
    std::string                  m_module_name;
    bool                         m_ros_enabled;
    yarp::dev::MapGrid2D         m_current_map;
    yarp::dev::Map2DLocation     m_initial_loc;
    yarp::dev::Map2DLocation     m_localization_data;
    yarp::dev::PolyDriver        m_ptf;
    yarp::dev::PolyDriver        m_pmap;
    yarp::os::ResourceFinder     m_rf;
    yarp::os::Port               m_rpcPort;
    yarp::os::Mutex              m_mutex;
    bool                         m_use_localization_from_odometry_port;
    bool                         m_use_localization_from_tf;
    bool                         m_use_map_server;
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::sig::Vector>  m_port_odometry_input;
    double                       m_last_odometry_data_received;
    yarp::dev::IFrameTransform*  m_iTf;
    yarp::dev::IMap2D*           m_iMap;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_map_id;
    double                       m_last_statistics_printed;
    
    //ros
    yarp::os::Node*                   m_rosNode;
    std::string                       m_topic_initial_pose;
    std::string                       m_topic_occupancyGrid;
    yarp::os::Publisher<geometry_msgs_PoseWithCovarianceStamped> m_rosPublisher_initial_pose;
    yarp::os::Publisher<nav_msgs_OccupancyGrid> m_rosPublisher_occupancyGrid;

public:
    localizationModule()
    {
        m_iMap = 0;
        m_iTf = 0;
        m_rosNode = 0;
        m_ros_enabled = false;
        m_module_name = "localizationServer";
        m_tf_data_received = -1;
        m_last_odometry_data_received = -1;
        m_last_statistics_printed = -1;

        m_localization_data.map_id = "unknown";
        m_localization_data.x = nan("");
        m_localization_data.y = nan("");
        m_localization_data.theta = nan("");
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        m_rpcPort.open("/"+m_module_name+"/rpc");
        attach(m_rpcPort);
        //attachTerminal();
        m_rf = rf;
        
        //configuration file cheking
        Bottle general_group = m_rf.findGroup("GENERAL");
        if (general_group.isNull())
        {
            yError() << "Missing GENERAL group!";
            return false;
        }

        Bottle initial_group = m_rf.findGroup("INITIAL_POS");
        if (initial_group.isNull())
        {
            yError() << "Missing INITIAL_POS group!";
            return false;
        }

        Bottle localization_group = m_rf.findGroup("LOCALIZATION");
        if (localization_group.isNull())
        {
            yError() << "Missing LOCALIZATION group!";
            return false;
        }

        Bottle ros_group = m_rf.findGroup("ROS");

        Bottle tf_group = m_rf.findGroup("TF");
        if (tf_group.isNull())
        {
            yError() << "Missing TF group!";
            return false;
        }

        Bottle odometry_group = m_rf.findGroup("ODOMETRY");
        if (odometry_group.isNull())
        {
            yError() << "Missing ODOMETRY group!";
            return false;
        }

        Bottle map_group = m_rf.findGroup("MAP");
        if (map_group.isNull())
        {
            yError() << "Missing MAP group!";
            return false;
        }
        yDebug() << map_group.toString();

        //general group
        if (general_group.check("module_name") == false)
        {
            yError() << "Missing `module_name` in [GENERAL] group";
            return false;
        }
        m_module_name = general_group.find("module_name").asString();

        if (general_group.check("enable_ros") == false)
        {
            yError() << "Missing `ros_enable` in [GENERAL] group";
            return false;
        }
        m_ros_enabled = (general_group.find("enable_ros").asInt()==1);

        //ros group
        if (m_ros_enabled)
        {
            m_rosNode = new yarp::os::Node("/"+m_module_name);
                    
            if (ros_group.check ("occupancygrid_topic"))
            {
                m_topic_occupancyGrid = ros_group.find ("occupancygrid_topic").asString();
                if (!m_rosPublisher_occupancyGrid.topic(m_topic_occupancyGrid))
                {
                     if (m_rosNode)
                     {
                         delete m_rosNode;
                         m_rosNode=0;
                     }
                     yError() << "localizationModule: unable to publish data on " << m_topic_occupancyGrid << " topic, check your yarp-ROS network configuration";
                     return false;
                }
            }

            if (ros_group.check ("initialpose_topic")) 
            {
                m_topic_initial_pose = ros_group.find ("initialpose_topic").asString();
                {
                    if (!m_rosPublisher_initial_pose.topic(m_topic_initial_pose))
                    {
                        if (m_rosNode)
                        {
                            delete m_rosNode;
                            m_rosNode=0;
                        }
                        yError() << "localizationModule: unable to publish data on " << m_topic_initial_pose << " topic, check your yarp-ROS network configuration";
                        return false;
                    }
                }
            }
        }

        //localization group
        if (localization_group.check("use_localization_from_odometry_port")) { m_use_localization_from_odometry_port = (localization_group.find("use_localization_from_odometry_port").asInt() == 1); }
        if (localization_group.check("use_localization_from_tf"))   { m_use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }

        if (m_use_localization_from_odometry_port == true && m_use_localization_from_tf == true)
        {
            yError() << "`use_localization_from_tf` and `use_localization_from_odometry_port` cannot be true simulteneously!";
            return false;
        }

        //map server group
        yDebug() << map_group.toString();

        if (map_group.check("connect_to_yarp_mapserver") == false)
        {
            yError() << "Missing `connect_to_yarp_mapserver` in [MAP] group";
            return false;
        }
        m_use_map_server= (map_group.find("connect_to_yarp_mapserver").asInt()==1);

        //tf group
        if (tf_group.check("map_frame_id") == false)
        {
            yError() << "Missing `map_frame_id` in [TF] group";
            return false;
        }
        if (tf_group.check("robot_frame_id") == false)
        {
            yError() << "Missing `robot_frame_id` in [TF] group";
            return false;
        }
        m_frame_map_id = tf_group.find("map_frame_id").asString();
        m_frame_robot_id = tf_group.find("robot_frame_id").asString();

        //odometry group
        if (odometry_group.check("odometry_broadcast_port") == false)
        {
            yError() << "Missing `odometry_port` in [ODOMETRY] group";
            return false;
        }
        m_port_broadcast_odometry_name = odometry_group.find("odometry_broadcast_port").asString();

        //device driver opening and/or connections
        if (m_use_localization_from_odometry_port)
        {
            std::string odom_portname = "/" + m_module_name + "/odometry:i";
            bool b1 = m_port_odometry_input.open(odom_portname.c_str());
            bool b2 = yarp::os::Network::sync(odom_portname.c_str(),false);
            bool b3 = yarp::os::Network::connect(m_port_broadcast_odometry_name.c_str(), odom_portname.c_str());
            if (b1 == false || b2 ==false || b3==false)
            {
                yError() << "Unable to initialize odometry port connection from " << m_port_broadcast_odometry_name.c_str()<< "to:" << odom_portname.c_str();
                return false;
            }
        }

        if (m_use_localization_from_tf)
        {
            Property options;
            options.put("device", "transformClient");
            options.put("local", "/"+m_module_name + "/TfClient");
            options.put("remote", "/transformServer");
            if (m_ptf.open(options) == false)
            {
                yError() << "Unable to open transform client";
                return false;
            }
            m_ptf.view(m_iTf);
            if (m_ptf.isValid() == false || m_iTf == 0)
            {
                yError() << "Unable to view iTransform interface";
                return false;
            }
        }

        if (m_use_map_server)
        {
            Property map_options;
            map_options.put("device", "map2DClient");
            map_options.put("local", "/" +m_module_name + "/mapClient");
            map_options.put("remote", "/mapServer");
            if (m_pmap.open(map_options) == false)
            {
                yWarning() << "Unable to open mapClient";
            }
            else
            {
                yInfo() << "Opened mapClient";
                m_pmap.view(m_iMap);
                if (m_pmap.isValid() == false || m_iMap == 0)
                {
                    yError() << "Unable to view map interface";
                    return false;
                }
            }
        }

        //initial location intialization
        if (initial_group.check("initial_x"))     { m_initial_loc.x = initial_group.find("initial_x").asDouble(); }
        else { yError() << "missing initial_x param"; return false; }
        if (initial_group.check("initial_y"))     { m_initial_loc.y = initial_group.find("initial_y").asDouble(); }
        else { yError() << "missing initial_y param"; return false; }
        if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asDouble(); }
        else { yError() << "missing initial_theta param"; return false; }
        if (initial_group.check("initial_map"))   { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
        else { yError() << "missing initial_map param"; return false; }
        this->initializeLocalization(m_initial_loc);

        return true;
    }

    virtual bool interruptModule()
    {
        m_rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        if (m_ptf.isValid())
        {
            m_ptf.close();
        }
        if (m_pmap.isValid())
        {
            m_pmap.close();
        }

        if (m_rosPublisher_occupancyGrid.asPort().isOpen())
        {
            m_rosPublisher_occupancyGrid.interrupt();
            m_rosPublisher_occupancyGrid.close();
        }
        if (m_rosPublisher_initial_pose.asPort().isOpen())
        {
            m_rosPublisher_initial_pose.interrupt();
            m_rosPublisher_initial_pose.close();
        }
        if (m_rosNode)
        {
            delete m_rosNode;
            m_rosNode = 0;
        }
        m_rpcPort.interrupt();
        m_rpcPort.close();

        return true;
    }

    virtual double getPeriod()
    { 
        return 0.1; 
    }
    
    void printStats()
    {
        static int counter = 0;
        yInfo() << "Module running since " << counter << "s";
        counter+=10;
    }

    virtual bool updateModule()
    {
        double current_time = yarp::os::Time::now();

        if (current_time - m_last_statistics_printed > 10.0)
        {
            printStats();
            m_last_statistics_printed = yarp::os::Time::now();
        }

        LockGuard lock(m_mutex);
        if (m_use_localization_from_odometry_port)
        {
            yarp::sig::Vector *loc = m_port_odometry_input.read(false);
            if (loc)
            {
                m_last_odometry_data_received = yarp::os::Time::now();
                m_localization_data.x = loc->data()[0];
                m_localization_data.y = loc->data()[1];
                m_localization_data.theta = loc->data()[2];
            }
            if (current_time - m_last_odometry_data_received > 0.1)
            {
                yWarning() << "No localization data recevied for more than 0.1s!";
            }
        }
        else if (m_use_localization_from_tf)
        {
            yarp::sig::Vector iv;
            yarp::sig::Vector pose;
            iv.resize(6, 0.0);
            pose.resize(6, 0.0);
            bool r = m_iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);
            if (r)
            {
                //data is formatted as follows: x, y, angle (in degrees)
                m_tf_data_received = yarp::os::Time::now();
                m_localization_data.x = pose[0];
                m_localization_data.y = pose[1];
                m_localization_data.theta = pose[5] * RAD2DEG;
            }
            if (current_time - m_tf_data_received > 0.1)
            {
                yWarning() << "No localization data received for more than 0.1s!";
            }
        }
        else
        {
            yWarning() << "Localization disabled";
            return false;
        }

        return true; 
    }

    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).isString() && command.get(0).asString() == "getLoc")
        {
            std::string s = std::string("Current Location is: ") + m_localization_data.toString();
            reply.addString(s);
        }

        else if (command.get(0).isString() && command.get(0).asString() == "initLoc")
        {
            yarp::dev::Map2DLocation loc;
            loc.map_id = command.get(1).asString();
            loc.x = command.get(2).asDouble();
            loc.y = command.get(3).asDouble();
            loc.theta = command.get(4).asDouble();
            initializeLocalization(loc);
            std::string s = std::string("Localization initialized to: ") + loc.toString();
            reply.addString(s);
        }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

    bool initializeLocalization(yarp::dev::Map2DLocation& loc)
    {
        m_localization_data.map_id = loc.map_id;
        
        if (m_iMap)
        {
            bool b = m_iMap->get_map(m_localization_data.map_id, m_current_map);
            if (b==false)
            {
                yError() << "Map "<<m_localization_data.map_id << " not found.";
            }
            else
            {
                if (m_rosNode)
                {
                    double tmp=0;
                    nav_msgs_OccupancyGrid& ogrid = m_rosPublisher_occupancyGrid.prepare();
                    ogrid.clear();
                    ogrid.info.height=m_current_map.height();
                    ogrid.info.width=m_current_map.width();
                    m_current_map.getResolution(tmp);
                    ogrid.info.resolution=tmp;
                  //  m_current_map.
                    ogrid.info.map_load_time.sec=0;
                    ogrid.info.map_load_time.nsec=0;
                    double x, y, t;
                    m_current_map.getOrigin(x,y,t);
                    ogrid.info.origin.position.x=x;
                    ogrid.info.origin.position.y=y;
                    yarp::math::Quaternion q;
                    yarp::sig::Vector v(4);
                    v[0]=0; v[1]=0; v[2]=1; v[3]=t*DEG2RAD;
                    q.fromAxisAngle(v);
                    ogrid.info.origin.orientation.x = q.x();
                    ogrid.info.origin.orientation.y = q.y();
                    ogrid.info.origin.orientation.z = q.z();
                    ogrid.info.origin.orientation.w = q.w();
                    ogrid.data.resize(m_current_map.width()*m_current_map.height());
                    for (size_t i=0; i< ogrid.data.size(); i++)
                    {
                        yarp::dev::MapGrid2D::XYCell cell;
                        cell.x=i%m_current_map.width();
                        cell.y=i/m_current_map.width();
                        m_current_map.getOccupancyData(cell,tmp);
                        ogrid.data[i]=(int)tmp;
                    }
                    m_rosPublisher_occupancyGrid.write();
                }
            }
        }

        m_localization_data.x = loc.x;
        m_localization_data.y = loc.y;
        m_localization_data.theta = loc.theta;
        
        if (m_rosNode)
        {
            //send data to ROS localization module
            geometry_msgs_PoseWithCovarianceStamped& pos = m_rosPublisher_initial_pose.prepare();
            pos.clear();
            pos.header.frame_id=m_frame_robot_id;
            pos.header.seq=0;
            pos.header.stamp.sec=0;
            pos.header.stamp.nsec=0;
            pos.pose.pose.position.x= loc.x;
            pos.pose.pose.position.y= loc.y;
            pos.pose.pose.position.z= 0;
            yarp::math::Quaternion q;
            yarp::sig::Vector v(4);
            v[0]=0;
            v[1]=0;
            v[2]=1;
            v[3]=loc.theta*DEG2RAD;
            q.fromAxisAngle(v);
            pos.pose.pose.orientation.x = q.x();
            pos.pose.pose.orientation.y = q.y();
            pos.pose.pose.orientation.z = q.z();
            pos.pose.pose.orientation.w = q.w();
            m_rosPublisher_initial_pose.write();
        }
        return true;
    }

    bool parse_respond_vocab(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        int request = command.get(1).asVocab();
        if (request == VOCAB_NAV_GET_CURRENT_POS)
        {
            //plannerThread->setNewAbsTarget(loc);
            reply.addVocab(VOCAB_OK);
            reply.addString(m_localization_data.map_id);
            reply.addDouble(m_localization_data.x);
            reply.addDouble(m_localization_data.y);
            reply.addDouble(m_localization_data.theta);
        }
        else if (request == VOCAB_NAV_SET_INITIAL_POS)
        {
            yarp::dev::Map2DLocation loc;
            loc.map_id = command.get(2).asString();
            loc.x = command.get(3).asDouble();
            loc.y = command.get(4).asDouble();
            loc.theta = command.get(5).asDouble();
            initializeLocalization(loc);
            reply.addVocab(VOCAB_OK);
        }
        else
        {
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        yarp::os::LockGuard lock(m_mutex);
        reply.clear(); 
        if (command.get(0).isVocab())
        {
            if(command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
            {
                parse_respond_vocab(command,reply);
            }
            else
            {
                yError() << "Invalid vocab received";
                reply.addVocab(VOCAB_ERR);
            }
        }
        else if (command.get(0).isString())
        {
            if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("getLoc");
                reply.addString("initLoc <map_name> <x> <y> <angle in degrees>");
            }
            else if (command.get(0).isString())
            {
                parse_respond_string(command, reply);
            }
        }
        else
        {
            yError() << "Invalid command type";
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("localization.ini");           //overridden by --from parameter
    rf.setDefaultContext("localization");                  //overridden by --context parameter
    rf.configure(argc,argv);
    std::string debug_rf = rf.toString();

    localizationModule robotLocalizer;

    return robotLocalizer.runModule(rf);
}

 
