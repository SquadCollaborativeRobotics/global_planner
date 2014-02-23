#include <global_planner/GlobalPlanner.h>

GlobalPlanner::GlobalPlanner()
{

}

GlobalPlanner::~GlobalPlanner()
{

}

// Call setup functions
bool GlobalPlanner::Init(ros::NodeHandle* nh)
{
    m_nh = nh;
    SetupCallbacks();
    RegisterServices();

    FindRobots();

    m_tm.Init(nh, m_robots, "testList1.points");
}

// Executive function
void GlobalPlanner::Execute()
{

}

// System finished
void GlobalPlanner::Finished()
{

}

// setup callbacks, regiser services, load waypoints...
bool GlobalPlanner::SetupCallbacks()
{
    for (std::map<int, Robot_Ptr >::iterator it=m_robots.begin(); it!=m_robots.end(); ++it)
    {
        std::string robotStatusTopic = "/";
        robotStatusTopic += it->second->GetName();
        robotStatusTopic += "/status";
        ros::Subscriber sub = m_nh->subscribe(robotStatusTopic, 10, &GlobalPlanner::cb_robotStatus, this);
    }
}

bool GlobalPlanner::RegisterServices()
{
    //register services
    for (int i=0; i<m_robots.size(); i++)
    {

    }
}

// get robot information
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
{
    int id = msg->id;
    global_planner::RobotStatus status = *msg;
    m_robots[id]->SetData(status);
}

// Send request to all listening robots that
// they should send out their information to be added to the list of bots
int GlobalPlanner::FindRobots()
{
    std::vector< std::string > names;
    names.push_back(std::string("collector1"));
    names.push_back(std::string("collector2"));
    names.push_back(std::string("bin1"));

    m_robots.clear();

    for (int i=0; i<names.size(); i++)
    {
        Robot_Ptr robot (new RobotStatusWrapper());
        int id = i;
        robot->SetName(names[i]);
        robot->SetID(id);
        m_robots[id] = robot;
    }

    return names.size();
}
