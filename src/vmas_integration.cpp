/* Author: Joshua Bryant */

#include "Robot.h"
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"

#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <unordered_map>
#include <math.h>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <iostream>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

std::unordered_map<std::string, std::pair<float, float>> start_map{};
std::unordered_map<std::string, std::pair<float, float>> goal_map{};

int readConfig(std::string filename)
{
    std::ifstream configFile (filename);
    Json::Value root;

    if (configFile.is_open())
    {
        Json::CharReaderBuilder builder;
        builder["collectComments"] = false;
        JSONCPP_STRING errs;

        if (!parseFromStream(builder, configFile, &root, &errs))
        {
            std::cout << errs << std::endl;
            return EXIT_FAILURE;
        }
        //std::cout << root << std::endl;
        //return EXIT_SUCCESS;
        start_map.clear();
        goal_map.clear();

        for (Json::Value::iterator it = root.end(); it != root.begin();)
        {
            it--;
            std::cout << "Parsing " << it.key() << " config" << std::endl;
            Json::Value agent = (*it);
            start_map.insert(std::make_pair(it.key().asString(), std::make_pair(agent["start"][0][0].asFloat(), agent["start"][0][1].asFloat())));
            goal_map.insert(std::make_pair(it.key().asString(), std::make_pair(agent["goal"][0][0].asFloat(), agent["goal"][0][1].asFloat())));
        }
    }
    else
    {
        std::cout << "Unable to open file: " << filename << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void plan(double threshold = 0.1)
{
    std::cout << "Entering Planning Function." << std::endl; 

    // provide start and goals for every robot
    //std::unordered_map<std::string, std::pair<float, float>> start_map{   {"Robot 0", {11.5, 6}},
    //                                                                    };

    //std::unordered_map<std::string, std::pair<int, int>> goal_map{    {"Robot 0", {7, 18}},
    //                                                                   };

    // construct all of the robots (assume square robots with unit length)
    std::unordered_map<std::string, Robot*> robot_map;
    for(auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        //Robot* robot = new RectangularRobot(itr->first, 0.7, 0.5);
        Robot* robot = new RectangularRobot(itr->first, 0.05, 0.05);
        robot_map[itr->first] = robot;
    }
    std::cout << "All robots constructed." << std::endl;

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));
    std::cout << "Multi-robot space information constructed." << std::endl;

    // construct four individuals that operate in SE3
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Constructing for SE3" << std::endl;
        // construct the state space we are planning in
        auto space = createBounded2ndOrderHolonomicStateSpace(2, 2);
        std::cout << "Constructed state space." << std::endl;

        // name the state space parameter
        space->setName(itr->first);
        std::cout << "Named state space." << std::endl;

        // create a control space
        auto cspace = createUniform3DRealVectorControlSpace(space);
        std::cout << "Created control space." << std::endl;

        // construct an instance of space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
        std::cout << "Constructed instance of space information." << std::endl;

        // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneous2ndOrderCarSystemSVC>(si, robot_map));
        std::cout << "Set state validity checker for space." << std::endl;

        // set the state propagation routine
        auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderHolonomicODE));
        si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderHolonomicODEPostIntegration));
        std::cout << "Set the state propagation routine." << std::endl;

        // set the propagation step size
        si->setPropagationStepSize(0.1);
        std::cout << "Set propagation step size." << std::endl;

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);
        std::cout << "Set control duration." << std::endl;

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = start_map.at(itr->first).first;
        start[1] = start_map.at(itr->first).second;
        start[2] = 0.0;
        start[3] = 0.0;
        start[4] = 0.0;
        std::cout << "Set start state." << std::endl;

        // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        std::cout << "Created problem instance" << std::endl;

        // set the start and goal states
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<GoalRegion2ndOrderHolonomic>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second, threshold));
        std::cout << "Set the start and goal states." << std::endl;

        // add the individual information to the multi-robot SpaceInformation and Problem Definition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
        std::cout << "Added individual information the multi-robot space information and problem definitions." << std::endl;
    }

    std::cout << "---------------------------------------------------------------------------" << std::endl;
    // lock the multi-robot Space Information and Problem Definitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();
    std::cout << "Space information and problem definitions locked." << std::endl;

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    ma_si->setPlannerAllocator(allocator);
    std::cout << "Planner allocator set for multi-agent planner" << std::endl;

    // plan with K-CBS
    // plan using kinodynamic conflict based search
    auto planner = std::make_shared<omrc::KCBS>(ma_si);
    planner->setProblemDefinition(ma_pdef); //  bes ure to set the problem definition
    planner->setLowLevelSolveTime(5.);

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = planner->as<omrb::Planner>()->solve(180.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    double duration_s = (duration_ms.count() * 0.001);

    if (solved)
    {
        printf("Found solution in %0.2f seconds!\n", duration_s);
        omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
        std::ofstream MyFile("plan.txt");
        solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "Robot");
    }

    std::ofstream MyFile2("tree.txt");
    planner->printConstraintTree(MyFile2);
}

int main(int argc, char ** argv)
{
    std::cout << "Starting k-cbs from vmas data." << std::endl;
    int ret = readConfig("config.json");
    std::cout << "Successfully parsed in json data." << std::endl;
    plan(0.1);
}
