/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef PLANNER_THREAD_H
#define PLANNER_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/INavigation2D.h>
#include <string>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>

#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;

#define TIMEOUT_MAX 100

class PlannerThread: public yarp::os::PeriodicThread
{
    protected:
    //parameters affecting navigation behavior
    double m_goal_tolerance_lin;       //m 
    double m_goal_tolerance_ang;       //deg
    double m_waypoint_tolerance_lin;   //m 
    double m_waypoint_tolerance_ang;   //deg
    double m_goal_max_lin_speed;       //m/s
    double m_goal_min_lin_speed;       //m/s
    double m_goal_max_ang_speed;       //deg/s
    double m_goal_min_ang_speed;       //deg/s
    double m_goal_ang_gain;            //deg/s
    double m_goal_lin_gain;            //m/s
    double m_waypoint_max_lin_speed;   //m/s
    double m_waypoint_min_lin_speed;   //m/s
    double m_waypoint_max_ang_speed;   //deg/s
    double m_waypoint_min_ang_speed;   //deg/s
    double m_waypoint_ang_gain;        //deg/s
    double m_waypoint_lin_gain;        //m/s
    int    m_min_waypoint_distance;    //cells

    //semaphore
    public:
    Semaphore m_mutex;

    protected:
    //configuration parameters: robot geometric properties
    double    m_robot_radius;        //m
    double    m_robot_laser_x;       //m
    double    m_robot_laser_y;       //m
    double    m_robot_laser_t;       //deg
    bool      m_use_optimized_path;
    double    m_min_laser_angle;
    double    m_max_laser_angle;
    double    m_laser_angle_of_view;
    string    m_frame_robot_id;
    string    m_frame_map_id;
    double    m_imagemap_refresh_time;
    bool      m_enable_try_recovery;

    //storage for the environment map
    yarp::dev::MapGrid2D m_current_map;
    yarp::dev::MapGrid2D m_temporary_obstacles_map;
    yarp::dev::MapGrid2D m_augmented_map;

    //yarp device drivers and interfaces
    PolyDriver                                             m_ptf;
    PolyDriver                                             m_pLoc;
    PolyDriver                                             m_pLas;
    PolyDriver                                             m_pMap;
    IRangefinder2D*                                        m_iLaser;
    IMap2D*                                                m_iMap;
    ILocalization2D*                                       m_iLoc;

    //yarp ports
    BufferedPort<yarp::os::Bottle>                         m_port_status_input;
    BufferedPort<yarp::os::Bottle>                         m_port_yarpview_target_input;
    BufferedPort<yarp::os::Bottle>                         m_port_yarpview_target_output;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_port_map_output;
    BufferedPort<yarp::os::Bottle>                         m_port_status_output;
    RpcClient                                              m_port_commands_output;

    //internal data
    ResourceFinder                         &m_rf;
    yarp::dev::Map2DLocation               m_localization_data;
    yarp::dev::Map2DLocation               m_final_goal;
    std::queue<yarp::dev::Map2DLocation>   m_sequence_of_goals;
    std::vector<Map2DLocation>             m_locations_list;
    std::string                            m_last_target;
    std::vector<yarp::dev::MapGrid2D::XYCell>   m_laser_map_cells;

    //the path computed by the planner, stored a sequence of waypoints to be reached
    std::queue<MapGrid2D::XYCell>                 m_computed_path;
    std::queue<MapGrid2D::XYCell>                 m_computed_simplified_path;
    std::queue<MapGrid2D::XYCell>*                m_current_path;
    
    //statuses of the internal finite-state machine
    NavigationStatusEnum   m_planner_status;
    NavigationStatusEnum   m_inner_status;

    //timeout counters (watchdog on the communication with external modules)
    protected:
    int                 m_loc_timeout_counter;
    int                 m_laser_timeout_counter;
    int                 m_inner_status_timeout_counter;

    //drawing flags: enable/disable drawing of particular objects on the GUI
    public:
    bool                m_enable_draw_all_locations;
    bool                m_enable_draw_laser_scans;
    bool                m_enable_draw_enlarged_scans;
    
    public:
    /**
    * Sets a new target, expressed in the map reference frame.
    * @param target a three-elements vector containing the robot pose (x,y,theta)
    */
    bool          setNewAbsTarget(yarp::dev::Map2DLocation target);

    /**
    * Sets a new target, expressed in the robot reference frame.
    * @param target a three-elements vector containing the robot pose (x,y,theta)
    */
    bool          setNewRelTarget(yarp::sig::Vector target);

    /**
    * Terminates a previously started navigation task.
    * @return true if the operation was successful, false otherwise.
    */
    bool          stopMovement();

    /**
    * Pauses the robot navigation
    * @param sec the duration of the pause, expressed in seconds. A negative number means forever.
    * @return true if the operation was successful, false otherwise.
    */
    bool          pauseMovement(double secs=1);

    /**
    * Resumes the robot navigation after a previous pauseMovement()
    * @return true if the operation was successful, false otherwise.
    */
    bool          resumeMovement();
    
    /**
    * Returns robot current position
    * @param loc the current absolute position of the robot
    * @return true if the command is executed successfully, false otherwise
    */
    bool          getCurrentPos(Map2DLocation& v);
    
    /**
    * Sets as navigation target a location previously stored into the map server
    * @param location_name the name of the location to be reached
    * @return true if the command is executed successfully, false otherwise
    */
    bool          gotoLocation(std::string location_name);
    
    /**
    * Stores the current robot location in the map server, with the given name. Typically invoked by user through RPC.
    * @param location_name a string representing the location name
    * @return true if the store operation was successful, false otherwise
    */
    bool          storeCurrentLocation(std::string location_name);

    /**
    * Removes the chosen location from the map server. Typically invoked by user through RPC.
    * @param location_name a string representing the location name
    * @return true if the delete operation was successful, false otherwise
    */
    bool          deleteLocation(std::string location_name);
    
    /**
    * Retrieves the current waypoint in the target queue, expressed in absolute (map) reference frame.
    * @param the current goal (computed by the pathplanner algorithm)
    * @return true if the returned target is valid, false otherwise
    */
    bool          getCurrentAbsTarget(Map2DLocation& target);

    /**
    * Retrieves the current waypoint in the target queue, expressed in local (robot) reference frame.
    * @param the current goal (computed by the pathplanner algorithm)
    * @return true if the returned target is valid, false otherwise
    */
    bool          getCurrentRelTarget(Map2DLocation& target);

    /**
    * Retrieves the name of the map to which the current waypoint belongs to.
    * @return the map name
    */
    string        getCurrentMapId();
    
    /**
    * Retrieves the final target, expressed in absolute (map) reference frame.
    * This target is the one provided by the user with setNewAbsTarget()
    * @param the current target (set by a setNewAbsTarget)
    * @return true if the returned target is valid, false otherwise
    */
    bool          getFinalAbsTarget(Map2DLocation& target);

    /**
    * Retrieves the final target, expressed in relative (local) reference frame.
    * This target is the one provided by the user with setNewAbsTarget()/setNewRelTarget()
    * @param the current target (set by a setNewAbsTarget/setNewRelTarget)
    * @return true if the returned target is valid, false otherwise
    */
    bool          getFinalRelTarget(Map2DLocation& target);

    /**
    * Retrieves the name of the map to which the final target, requested by the user, belongs to.
    * @return the map name
    */
    string        getFinalMapId();

    /**
    * Returns the name of the target previously set by a gotoLocation(), if available.
    * If the current target has been set using a different method e.g. setNewAbsTarget(), this method fails.
    * @param location_name the name of the location to be reached
    * @return true if the command is executed successfully, false otherwise
    */
    bool          getLastTarget(string& target_name);

    /**
    * Returns the current navigation status used by the internal finite-state machine
    * @return the internal navigation status, expressed as a string
    */
    string        getNavigationStatusAsString();

    /**
    * Returns the current navigation status, used by the internal finite-state machine
    * @return the internal navigation status, expressed as an enum
    */
    NavigationStatusEnum getNavigationStatusAsInt();

    /**
    * Returns info about the current status of the navigation thread.
    * In particular, it returns the number of timeouts occurred while communicating with external modules.
    * @param localiz the number of timeouts occurred while getting localization data
    * @param laser the number of timeouts occurred while getting laser data
    * @param inner_status the number of timeouts occurred while communicating with the local navigation module
    */
    void          getTimeouts(int& localiz, int& laser, int& inner_status);

    private:
    bool          startPath();
    void          sendWaypoint();
    void          readTargetFromYarpView();
    bool          readLocalizationData();
    void          readLaserData();
    bool          readInnerNavigationStatus();
    void          draw_map();
    bool          getCurrentWaypoint(yarp::dev::MapGrid2D::XYCell &c) const;
    bool          updateLocations();

    public:
    /**
    * Constructor.
    * @param _period the main thread period (typical value 0.020s)
    * @param _rf the resource finder containing the configuration options
    */
    PlannerThread(double _period, ResourceFinder &_rf);

    //methods inherited from yarp::os::RateThread
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    
    /**
    * Prints stats about the internal status of the module
    */
    void printStats();

};

#endif
