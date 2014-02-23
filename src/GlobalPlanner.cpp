#include <global_planner/GlobalPlanner.h>

GlobalPlanner::GlobalPlanner()
{

}

GlobalPlanner::~GlobalPlanner()
{

}

// Call setup functions
bool GlobalPlanner::Init(ros::NodeHandle& nh)
{

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
        ros::Subscriber sub = m_nh->subscribe("chatter", 1000, &GlobalPlanner::cb_robotStatus, this);
    }
}

bool GlobalPlanner::RegisterServices()
{
    //register services
    for (int i=0; i<m_robots.size(); i++)
    {

    }
}

bool GlobalPlanner::LoadWaypoints(std::string filename)
{

}

// get robot information
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
{
    int id = msg->id;
}

// Send request to all listening robots that
// they should send out their information to be added to the list of bots
int GlobalPlanner::FindRobots()
{
    std::vector< std::string > names;
    names.push_back(std::string("collector1"));
    //names.push_back(std::string("collector2"));
    //names.push_back(std::string("bin1"));

    m_robots.clear();

    for (int i=0; i<names.size(); i++)
    {
        boost::shared_ptr< Robot > robot (new Robot());
        int id = i;
        robot->SetName(names[i]);
        robot->SetID(id);
        m_robots[id] = robot;
    }
    return names.size();
}
