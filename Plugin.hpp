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

    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr rws_robot;
    QPushButton *_btnConnect,*_btnHome, *_btnMimic, *_btnPrint, *_btnPickPlace;

    std::vector<double> homeQ = { 2.04046, -1.47102, 1.17649, -1.26727, -1.54478, -0.120219 };

    //Locations
    std::vector<double> pickQ = { 0.844511, -1.46255, 1.79629, -1.94328, -1.52762, -0.168244 };
    std::vector<double> pickTCP = { -0.244121, -0.479221, 0.36019, 0.867022, 3.00104, -0.0799999 };

    std::vector<double> placeQ = {2.76091, -1.182, 1.65908, -2.00515, -1.52773, -0.168448};
    std::vector<double> placeTCP = {0.612378, -0.0987183, 0.2712, -1.95912, 2.43967, 0.0864251};

    //Threads
    std::thread robotMimicThread;
    std::thread moveThread;
};

#endif /*PLUGIN_HPP*/
