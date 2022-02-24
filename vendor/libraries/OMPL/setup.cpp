
#include "setup.hpp"
#if NEWOMPL
  #include <ompl/geometric/planners/informedtrees/BITstar.h>
  #include <ompl/geometric/planners/informedtrees/ABITstar.h>
#endif
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#if OMPL_BENCHMARK
  #include <ompl/geometric/planners/informedtrees/BITstar.h>
  #include <ompl/geometric/planners/informedtrees/AITstar.h>
  #include <ompl/geometric/planners/rrt/RRTstar.h>
  #include <ompl/geometric/planners/rrt/RRTsharp.h>
  #include <ompl/geometric/planners/fmt/FMT.h>
  #include <ompl/geometric/planners/cforest/CForest.h>
  #include <ompl/geometric/planners/rrt/InformedRRTstar.h>
  #include <ompl/geometric/planners/rrt/LBTRRT.h>
  #include <ompl/geometric/planners/sst/SST.h>
  #include <ompl/geometric/planners/rrt/TRRT.h>
  #include <ompl/geometric/planners/rrt/RRTXstatic.h>
  #include <ompl/geometric/planners/prm/SPARS.h>
  #include <ompl/geometric/planners/prm/SPARStwo.h>
#endif

#include <algorithm> 
#include <chrono> 
#include <iostream> 
#include <vector> 
#include <utility>

#include <ENCGIS/DBTree.hpp>
#include <OMPL/OMPLfunctions.hpp>
#include <OMPL/OMPLMotionValidator.hpp>
#include <OMPL/OMPLMotionValidator2.hpp>
#include <OMPL/OMPLMotionValidator3.hpp>
namespace OMPLintegrationENCGIS
{
  //! Function searching for path, ends search once first valid path is available
  og::PathGeometric findPath(og::SimpleSetup &ss, double maxPlaningTime, configurations_t config) {

    switch(config) {
#if NEWOMPL
      case C_KBIT:
        //ob::PlannerPtr test= kbitstar( ss.getSpaceInformation(), "kBITstar");
        ss.setPlanner(kbitstar( ss.getSpaceInformation(), "kBITstar"));
        break;
      case C_KABIT:
        ss.setPlanner(ob::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));
        break;
#endif      
      case C_FMT:
        ss.setPlanner(ob::PlannerPtr(new og::FMT(ss.getSpaceInformation())));
        break;
      
#if !NEWOMPL
    case C_KBIT:
    case C_KABIT:
#endif
      case C_DEFAULT:
      default:
        ss.setPlanner(ob::PlannerPtr(new og::LBKPIECE1(ss.getSpaceInformation())));
    };
    // attempt to solve the problem within a given planning time
    ob::PlannerStatus solved = ss.solve(maxPlaningTime);
    if (solved)
    {
        // Simplify path
        ss.simplifySolution();
        // Extract path
        og::PathGeometric states = ss.getSolutionPath();

        return states;
    }
    else
      return ompl::geometric::PathGeometric(ss.getSpaceInformation());
  }


  ompl::geometric::SimpleSetup createSetup(double startY, double startX, double goalY, double goalX, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, unsigned searchDepth) 
  {
    // Construct the state space
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // Define bounds of searching space
    space->setBounds(bounds);

    // Define a simple setup class
    ompl::geometric::SimpleSetup ss(space);

    // Define Motion validator for this space
    ss.getSpaceInformation()->setMotionValidator(std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator2>(ss.getSpaceInformation(), lineCheck, searchDepth));

    // Set state validity checking for this space
    ss.setStateValidityChecker([pointCheck](const ompl::base::State *state) { return isStateValid(state, pointCheck); });

    // Create the start state
    ompl::base::ScopedState<> start(space);
    start[0]=startX;
    start[1]=startY;

    // Create the goal state
    ompl::base::ScopedState<> goal(space);
    goal[0]=goalX;
    goal[1]=goalY;

    // Set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    return ss;
  }
  bool isStateValid(const ompl::base::State *state,  ENCGIS::isPointInLayerStatement *qry)
  {
    if (state != nullptr)
    {
      const auto *rstate = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(state);
      return qry->run(rstate->values[1], rstate->values[0]);
    }
    else
      std::cout << "nullptr" << std::endl;
    return false;
    
  }

#if NEWOMPL
      ompl::base::PlannerPtr kbitstar1(const ompl::base::SpaceInformationPtr &si, std::string name, bool pruning, double rewireFactor, double samplesPerBatch, double PruneThreshold, bool JITS, bool Knearest)
      {
          ompl::geometric::BITstar *planner = new og::BITstar(si);
          planner->setName(name);
          planner->setPruning(pruning); // True
          planner->setRewireFactor(rewireFactor); // graphPtr_ 1.1 Have tried 1.1, 1.5 and 2.1. Increasing seems bad for extralong, but 1.5 gives faster first solution for byneset and nidelven. 15. and 2.1 faster convergence for Froan.
          planner->setSamplesPerBatch(samplesPerBatch); // 100 Increasing this uses more memory but higher solve rate (tested from 100,500,1000)
          planner->setPruneThresholdFraction(PruneThreshold); // 0.05 Large values seem to give worse performance, slightly more seems to have potential benefit time to first solution.
          planner->setJustInTimeSampling(JITS); // graphPtr_ false (Must be used with r-disc)
          planner->setUseKNearest(Knearest); //Tentatively looks to worsen performance

          return ompl::base::PlannerPtr(planner);
      }
      ompl::base::PlannerPtr kbitstar(const ompl::base::SpaceInformationPtr &si, std::string name)
      {
          ompl::geometric::BITstar *planner = new og::BITstar(si);
          planner->setName(name);
          planner->setPruning(false);
          //planner->setUseKNearest(false); Tentatively looks to worsen performance
          //planner->setStopOnSolnImprovement(true);

          return ompl::base::PlannerPtr(planner);
      }

#endif
#if OMPL_BENCHMARK
      //! Function for performing benchmarks on a setup.
      void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount)
      {

          // Bencmarking code
          ompl::tools::Benchmark b(ss, benchmark_name);
          
          // Planners to be tested
          b.addPlannerAllocator(std::bind(&kbitstar, std::placeholders::_1, "kBITstar"));
          
          // Configure planner through the request class
          ompl::tools::Benchmark::Request req = ompl::tools::Benchmark::Request();
          req.maxTime = benchmark_maxTime;
          req.maxMem = benchmark_maxMem;
          req.runCount = benchmark_runCount;
          req.displayProgress = true;

          // Run the configured benchmark
          b.benchmark(req);
          
          // Save result as ompl_host_time.log. This can be further transformed to a DB with ompl_benchmark_statistics.py
          b.saveResultsToFile();
      }

      void multiBmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount)
      {

          // Bencmarking code
          ompl::tools::Benchmark b(ss, benchmark_name);
          // Planners to be tested
//b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(ss.getSpaceInformation())));

//b.addPlannerAllocator(std::bind(&kabitstar, std::placeholders::_1, "kABITstar"));
//std::string name, bool pruning, double rewireFactor, double samplesPerBatch, double PruneThreshold, bool JITS, bool Knearest)
/*
          planner->setPruning(pruning); // True
          planner->setRewireFactor(rewireFactor); // graphPtr_ 1.1
          planner->setSamplesPerBatch(samplesPerBatch); // 100
          planner->setPruneThresholdFraction(PruneThreshold); // 0.05
          planner->setJustInTimeSampling(JITS); // graphPtr_ false (Must be used with r-disc)
          planner->setUseKNearest(Knearest); //Tentatively looks to worsen performance
*/

//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar1" , true, 1.1, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar2" , true, 1.1, 500, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar3" , true, 1.1, 1000, 0.05, false, true));

//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar4" , true, 1.1, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar5" , false, 1.1, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar6" , true, 1.1, 100, 0.05, false, false));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar7" , true, 1.1, 100, 0.05, true, false));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar8" , false, 1.1, 100, 0.05, false, false));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar9" , false, 1.1, 100, 0.05, true, false));
//
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar10", true, 1.1, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar11", true, 1.1, 100, 0.55, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar12", true, 1.1, 100, 0.75, false, true));
//
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar13" , true, 1.1, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar14" , true, 1.5, 100, 0.05, false, true));
//b.addPlannerAllocator(std::bind(&kbitstar1, std::placeholders::_1, "kBITstar15" , true, 2.1, 100, 0.05, false, true));

b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::FMT(ss.getSpaceInformation())));

b.addPlannerAllocator(std::bind(&kbitstar, std::placeholders::_1, "kBITstar1"));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::ABITstar(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::AITstar(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::TRRT(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTXstatic(ss.getSpaceInformation())));
//b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::SST(ss.getSpaceInformation())));
//b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARS(ss.getSpaceInformation())));
//b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARStwo(ss.getSpaceInformation())));

          // Configure planner through the request class
          ompl::tools::Benchmark::Request req = ompl::tools::Benchmark::Request();
          req.maxTime = benchmark_maxTime;
          req.maxMem = benchmark_maxMem;
          req.runCount = benchmark_runCount;
          req.displayProgress = true;

          // Run the configured benchmark
          b.benchmark(req);
          
          // Save result as ompl_host_time.log. This can be further transformed to a DB with ompl_benchmark_statistics.py
          b.saveResultsToFile();
      }
#endif
  /*
  //! Function searching for path, only stops if condition met, else contnues to optimize
  void findPathExtended(og::SimpleSetup &ss)
  {
  //ss.setPlanner(ob::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));
  ob::PlannerPtr test= kabitstar( ss.getSpaceInformation(), "kABITstar");
  ss.setPlanner(test);
  ss.getProblemDefinition()->setIntermediateSolutionCallback(intermediate());

  //std::cout << test->getName() << std::endl;
  //ss.setPlannerAllocator();
  std::shared_ptr<ompl::geometric::ABITstar> planner = std::dynamic_pointer_cast<ompl::geometric::ABITstar> (test);
  //std::shared_ptr<ompl::geometric::ABITstar> planner = static_cast<ompl::geometric::ABITstar>(test);
  //planner->setStopOnSolnImprovement(true);

  // attempt to solve the problem within a given planning time
  ob::PlannerStatus solved = ss.solve( exactSolnPlannerTerminationCondition( &m_intermediate, this) );
  if (solved)
  {
      //ChartsDatabase::DBTree* tree = new ChartsDatabase::DBTree(m_con);
      //debug("Found solution");
      ////ChartsDatabase::printPath(ss.getSolutionPath());
      //ChartsDatabase::pathToTree(ss.getSolutionPath(), "tree", tree);
      // print the path to screen
      ss.simplifySolution();
      //ss.getSolutionPath().print(std::cout);
      og::PathGeometric states = ss.getSolutionPath();
      IMC::PlanDB pdb = OMPL::createPlanDBEntry(states, "autoPlan2", 1.0);
      dispatch(pdb);
      //sendPlan(states, "autoPlan", 1.0);
      activatePlan("autoPlan2");
      //pathToDB(states, "tree3");
      //ChartsDatabase::pathToTree(ss.getSolutionPath(), "tree3", tree);
      //Memory::clear(tree);
  }
  else
      war("No solution found");
  }
*/
/*
/////////////////////////////////////////////////////////////////////
  //! Creates a simplesetup object using the ChartsDB polygons for state and motion validation.
  ompl::geometric::SimpleSetup createSetup2(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck, ENCGIS::getClosestIntersectWithOffset* lineCheck2, unsigned searchDepth) 
  {
  // Construct the state space
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  // Define bounds of searching space
  space->setBounds(bounds);

  // Define a simple setup class
  ompl::geometric::SimpleSetup ss(space);

  // Define Motion validator for this space
  //ss.getSpaceInformation()->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(ss.getSpaceInformation()));
  //ss.getSpaceInformation()->setMotionValidator(std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator3>(ss.getSpaceInformation(), lineCheck, lineCheck2, 4));
  ss.getSpaceInformation()->setMotionValidator(std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator2>(ss.getSpaceInformation(), lineCheck, searchDepth));
  //ss.getSpaceInformation()->setMotionValidator(std::make_shared<OMPLintegrationENCGIS::ChartsDBMotionValidator>(ss.getSpaceInformation(), m_con, 6));


  // Set state validity checking for this space
  ss.setStateValidityChecker([pointCheck](const ompl::base::State *state) { return isStateValid(state, pointCheck); });
  //ss.setStateValidityChecker([&](const ompl::base::State *state) { return OMPLintegrationENCGIS::isStateValid(state, m_con); });

  // Create the start state
  ompl::base::ScopedState<> start(space);
  start[0]=startLon;
  start[1]=startLat;

  // Create the goal state
  ompl::base::ScopedState<> goal(space);
  goal[0]=goalLon;
  goal[1]=goalLat;

  // Set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  return ss;
  }
/////////////////////////////////////////////////////////////////////
*/

}