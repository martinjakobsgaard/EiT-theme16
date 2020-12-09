#include "Plugin.hpp"

// Qt includes
#include <QPushButton>
#include <QGridLayout>

// RobWork includes
#include <RobWorkStudio.hpp>

// standard includes
#include <boost/bind.hpp>

Plugin::Plugin():
    rws::RobWorkStudioPlugin("Plugin", QIcon("../plugin.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btnConnect = new QPushButton("Connect robot");
    pLayout->addWidget(_btnConnect, row++, 0);
    connect(_btnConnect, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btnHome = new QPushButton("Home robot");
    pLayout->addWidget(_btnHome, row++, 0);
    connect(_btnHome, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btnMimic = new QPushButton("Mimic robot");
    pLayout->addWidget(_btnMimic, row++, 0);
    connect(_btnMimic, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btnPrint = new QPushButton("Print location");
    pLayout->addWidget(_btnPrint, row++, 0);
    connect(_btnPrint, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btnPickPlace = new QPushButton("Perform Pick and Place");
    pLayout->addWidget(_btnPickPlace, row++, 0);
    connect(_btnPickPlace, SIGNAL(clicked()), this, SLOT(clickEvent()));

    pLayout->setRowStretch(row,1);

    is_connected = false;
}

Plugin::~Plugin()
{
}

void Plugin::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&Plugin::stateChangedListener, this, boost::arg<1>()), this);
    std::cout << "End of initialize()" << std::endl;
}

void Plugin::open(rw::models::WorkCell* workcell)
{
    // If workcell exists
    if (workcell != NULL)
    {
        // Get rws info
        rws_wc = workcell;
        rws_state = rws_wc->getDefaultState();

        // Locate robot in workcell
        rws_robot = rws_wc->findDevice<rw::models::SerialDevice>("UR5e_2018");

        // Use rws collision checker
        collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    }
    std::cout << "End of open()" << std::endl;
}

void Plugin::close()
{
    std::cout << "End of close()" << std::endl;
}

void Plugin::clickEvent()
{
    QObject *obj = sender();
    if(obj == _btnConnect)
    {
        log().info() << "Connect button pressed!\n";
        connectRobot();

    }
    else if(obj == _btnHome)
    {
        log().info() << "Home robot pressed!\n";
        homeRobot();
    }

    else if(obj == _btnMimic)
    {
        log().info() << "Mimic button pressed!\n";
        startRobotMimic();
    }
    else if (obj == _btnPrint)
    {
        log().info() << "Button Print pressed!\n";
        printLocation();
    }
    else if (obj == _btnPickPlace)
    {
        log().info() << "Button Pick and Place pressed!\n";
        startPickPlace();
    }
}

void Plugin::stateChangedListener(const rw::kinematics::State& state)
{
    log().info() << "State changed!";
}

void Plugin::startRobotMimic()
{
    if(robotMimicThread.joinable())
        robotMimicThread.join();
    robotMimicThread = std::thread(&Plugin::runRobotMimic, this);
}

void Plugin::startPickPlace()
{
    if(moveThread.joinable())
        moveThread.join();
    moveThread = std::thread(&Plugin::pickandPlace, this);
}

void Plugin::runRobotMimic()
{
    if(!is_connected)
    {
        std::cout << "Robot not connected" << std::endl;
        return;
    }

    while(true)
    {
        std::vector<double> currentQ = ur_robot_receive->getActualQ();
        rws_robot->setQ(currentQ, rws_state);
        getRobWorkStudio()->setState(rws_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Plugin::homeRobot()
{
    if(!is_connected)
    {
        std::cout << "No connection to the robot" << std::endl;
        return;
    }

    std::cout << "Moving to 'Home'... " << std::endl;

    std::vector<std::vector<double>> path;
    std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> toQ = homeQ;
    rw::kinematics::State tmp_state = rws_state.clone();

    createPathRRTConnect(fromQ, toQ, 0.05, path, tmp_state);

    std::cout << "Moving robot..." << std::endl;
    printArray(path[0]);
    printArray(path[1]);
    ur_robot->moveJ(path);
}

void Plugin::printArray(std::vector<double> print)
{
    std::cout << "{ ";
    for(size_t i = 0; i < print.size()-1; i++)
        std::cout << print[i] << ", ";
    std::cout << print[print.size()-1] << " }" << std::endl;
}

void Plugin::printLocation()
{
    gripperControl.sendMessage(BACK, 50);

    std::cout << "Joint configuration:" << std::endl;
    std::vector<double> actualQ=ur_robot_receive->getActualQ();
    printArray(actualQ);

    std::cout << "TCP location:" << std::endl;
    std::vector<double> actualL=ur_robot_receive->getActualTCPPose();
    printArray(actualL);
}

void Plugin::performRRT(std::vector<double> toQ)
{
    if(!is_connected)
    {
        std::cout << "No connection to the robot" << std::endl;
        return;
    }

    std::vector<std::vector<double>> path;
    std::vector<double> fromQ = ur_robot_receive->getActualQ();
    rw::kinematics::State tmp_state = rws_state.clone();

    createPathRRTConnect(fromQ, toQ, 0.05, path, tmp_state);

    std::cout << "Moving robot..." << std::endl;

    std::cout << "Size of path is: " << path.size() << std::endl;

    for (size_t i = 0; i < path.size(); i++)
    {
        std::cout << "Performing movement " << i << std::endl;
        std::vector<double> subvector = {path[i].begin(), path[i].end() - 2};
        ur_robot->moveJ(subvector, path[i].at(6), path[i].at(7));
    }
}

void Plugin::connectRobot()
{
    std::cout << "Connecting to " << ur_robot_ip << std::endl;
    if(!is_connected)
    {
        std::cout << "Control interface connecting\t" << std::endl;
        ur_robot = new ur_rtde::RTDEControlInterface(ur_robot_ip);
        std::cout << "Receive interface connecting\t" << std::endl;
        ur_robot_receive = new ur_rtde::RTDEReceiveInterface(ur_robot_ip);
        std::cout << "IO interface connecting\t" << std::endl;
        ur_robot_io = new ur_rtde::RTDEIOInterface(ur_robot_ip);

        is_connected = true;
        std::cout << "Done connecting" << std::endl;

        std::cout << "Connecting to motor" << std::endl;
    }
    else
        std::cout << "Already connected..." << std::endl;
}

void Plugin::createPathRRTConnect(std::vector<double> start, std::vector<double> goal, double eps, std::vector<std::vector<double>> &path, rw::kinematics::State state)
{
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(collisionDetector.get(), rws_robot, state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(rws_robot), constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath qpath;
    std::cout << "Generating path" << std::endl;
    planner->query(start, goal, qpath);
    std::cout << "Found path" << std::endl;

    path.clear();
    //Pushing path to output
    for(const auto &q : qpath)
    {
        std::vector<double> q_copy = q.toStdVector();
        path.push_back(addMove(q_copy, 0.4, 0.4));
    }
}

void Plugin::moveToJ(std::vector<double> goal, double acc, double vel)
{
    if(!is_connected)
    {
        std::cout << "No connection to the robot" << std::endl;
        return;
    }

    ur_robot->moveJ(goal, acc, vel);
}

void Plugin::pickandPlace()
{
    if(!is_connected)
    {
        std::cout << "No connection to the robot" << std::endl;
        return;
    }

    performRRT(homeQ);
    performRRT(pickQ);
    performRRT(approachQ);
    performRRT(collisionFirstQ);
    performRRT(startLiftQ);
    performRRT(otherStartLiftQ);
    performRRT(endLiftQ);
    performRRT(weDoneQ);

    return;
}

std::vector<double> Plugin::addMove(std::vector<double> pos, double acc = 0.5, double vel = 0.5)
{
    std::vector<double> move = {acc, vel};
    std::vector<double> position_and_move;
    position_and_move.reserve(pos.size() + move.size());
    position_and_move.insert( position_and_move.end(), pos.begin(), pos.end() );
    position_and_move.insert( position_and_move.end(), move.begin(), move.end() );
    return position_and_move;
}
