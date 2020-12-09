#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudioPlugin.hpp>

// UR RTDE includes
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

// RobWork library includes
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

// Standard includes
#include <iostream>
#include <thread>
#include <utility>
#include <chrono>

// Motor communication
#include "gripper_control.h"

class QPushButton;

class Plugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "Plugin.json")
public:
    Plugin();
    virtual ~Plugin();

    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

    void runRobotMimic();
    void connectRobot();

    void startRobotMimic();
    void startPickPlace();

    void performRRT(std::vector<double> toQ);

    void moveToJ(std::vector<double> goal, double acc, double vel);

    void pickandPlace();

    void printLocation();
    void printArray(std::vector<double> print);

private slots:
    void clickEvent();
    void stateChangedListener(const rw::kinematics::State& state);
    void homeRobot();
    void createPathRRTConnect(std::vector<double> start, std::vector<double> goal, double eps, std::vector<std::vector<double>> &path, rw::kinematics::State state);
    std::vector<double> addMove(std::vector<double> pos, double acc, double vel);

private:
    std::atomic_bool is_connected;

    // UR interface
    std::string ur_robot_ip = "192.168.0.212";
    ur_rtde::RTDEControlInterface   *ur_robot;
    ur_rtde::RTDEIOInterface        *ur_robot_io;
    ur_rtde::RTDEReceiveInterface   *ur_robot_receive;

    // Robwork
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr rws_robot;
    QPushButton *_btnConnect,*_btnHome, *_btnMimic, *_btnPrint, *_btnPickPlace;

    // Gripper control
    GripperControl gripperControl;

    //Home position
    std::vector<double> homeQ = { 2.04046, -1.47102, 1.17649, -1.26727, -1.54478, 0 };

    //Locations
    std::vector<double> pickQ = { 0.844511, -1.46255, 1.79629, -1.94328, -1.52762, -0.168244 };
    std::vector<double> pickTCP = { -0.244121, -0.479221, 0.36019, 0.867022, 3.00104, -0.0799999 };

    std::vector<double> approachQ = { 1.601, -1.62851, 2.17649, -2.09607, -1.59137, -1.92551 };
    std::vector<double> approach = { 0.141468, -0.403892, 0.279862, -2.59882, 1.75947, 0.0550876 };
    std::vector<double> collisionFirstQ = { 1.66542, -1.60411, 2.15296, -2.09532, -1.58986, -1.86103 };
    std::vector<double> collisionFirst = { 0.168286, -0.403899, 0.279878, -2.59883, 1.75953, 0.0550297 };
    std::vector<double> startLiftQ = { 1.62701, -1.61587, 2.16705, -2.09852, -1.42901, -1.9039 };
    std::vector<double> startLift = { 0.168299, -0.403887, 0.279862, -2.51804, 1.70014, -0.143308 };
    std::vector<double> otherStartLiftQ = { 1.63901, -1.62738, 2.14568, -2.0662, -1.47924, -1.89086 };
    std::vector<double> otherStartLift = { 0.168284, -0.403904, 0.290216, -2.54417, 1.71892, -0.0822169 };
    std::vector<double> endLiftQ = { 2.54006, -1.25795, 1.69148, -1.96361, -1.58432, -0.992226 };
    std::vector<double> endLift = { 0.554041, -0.222377, 0.297774, 2.59007, -1.74284, -0.063884 };
    std::vector<double> weDoneQ = { 2.55634, -1.32267, 1.48288, -1.69001, -1.58432, -0.977218 };
    std::vector<double> weDone = { 0.561742, -0.21606, 0.407612, 2.58995, -1.74291, -0.0638374 };

    //Threads
    std::thread robotMimicThread;
    std::thread moveThread;
};

#endif /*PLUGIN_HPP*/
