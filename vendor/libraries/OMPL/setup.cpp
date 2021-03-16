#include "setup.hpp"
namespace OMPLintegrationDUNE
{
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
  //! Function searching for path, ends search once first valid path is available
  og::PathGeometric findPath(og::SimpleSetup &ss, double maxPlaningTime) {
    //ss.setPlanner(ob::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));
    ob::PlannerPtr test= kabitstar( ss.getSpaceInformation(), "kABITstar");
    ss.setPlanner(test);
        // attempt to solve the problem within a given planning time
    ob::PlannerStatus solved = ss.solve(maxPlaningTime);
    if (solved)
    {
        //debug("Found solution");
        // Simplify path
        ss.simplifySolution();
        // Extract path
        og::PathGeometric states = ss.getSolutionPath();

        return states;
    }
    else
      return ompl::geometric::PathGeometric(ss.getSpaceInformation());
  }

  //! Creates a simplesetup object using the ChartsDB polygons for state and motion validation.
  ompl::geometric::SimpleSetup createSetup(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds, ENCGIS::isPointInLayerStatement* pointCheck, ENCGIS::lineIntersectLayerStatement* lineCheck) 
  {
  // Construct the state space
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  // Define bounds of searching space
  space->setBounds(bounds);

  // Define a simple setup class
  ompl::geometric::SimpleSetup ss(space);

  // Define Motion validator for this space
  //ss.getSpaceInformation()->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(ss.getSpaceInformation()));
  ss.getSpaceInformation()->setMotionValidator(std::make_shared<MotionPlanners::OMPL::ChartsDBMotionValidator2>(ss.getSpaceInformation(), lineCheck, 5));
  //ss.getSpaceInformation()->setMotionValidator(std::make_shared<MotionPlanners::OMPL::ChartsDBMotionValidator>(ss.getSpaceInformation(), m_con, 6));


  // Set state validity checking for this space
  ss.setStateValidityChecker([pointCheck](const ompl::base::State *state) { return isStateValid(state, pointCheck); });
  //ss.setStateValidityChecker([&](const ompl::base::State *state) { return MotionPlanners::OMPL::isStateValid(state, m_con); });

  // Set optimization objective to strive for if optimizing planner is used
  ss.setOptimizationObjective(std::make_shared<MotionPlanners::OMPL::LatLonDist>(ss.getSpaceInformation()));

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
      //! Termination condition to be used with ompl. This one stops the planner when the task gets the stop signal, if the stop input is true or if maxTime is reached.
      ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask, double maxPlaningTime)
      {
        double duration = maxPlaningTime; // Maxtime
        const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
          return ompl::base::PlannerTerminationCondition([stop, inTask, endTime]
                                              {
                                                if(*stop) {
                                                  return true;
                                                }

                                                if(ompl::time::now() > endTime){
                                                  inTask->inf("maxtime");
                                                  return true;
                                                }

                                                  return inTask->isStopping();
                                              });
      }


      //! A function that returns the ompl::base::ReportIntermediateSolutionFn type.
      ompl::base::ReportIntermediateSolutionFn intermediate(DUNE::Tasks::Task *inTask, double minPlaningTime) {
        double duration = minPlaningTime; // Mintime
        const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
        return ompl::base::ReportIntermediateSolutionFn([inTask, endTime](const ob::Planner *planner, const std::vector< const ob::State * > &states, const ob::Cost cost) 
        { 
          if(ompl::time::now() > endTime){
            inTask->inf("mintime");
          }
          // Create ompl::geometric::PathGeometric from "states" vector
          auto path = ompl::geometric::PathGeometric(planner->getSpaceInformation());
          for (auto ptr = states.begin(); ptr < states.end(); ptr++) {
            //auto state = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(*ptr);
            //std::cout << state->values[1] <<","<< state->values[0] << std::endl;
            path.append(*ptr);
          }

              //ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_con);
              //MotionPlanners::OMPL::pathToTree(path, "tree", tree);
              //Memory::clear(tree);

          //intermediate(planner,states, cost); 
          inTask->inf("intermediate solution with %f Cost", cost.value());
        //std::cout << "intermediate solution with "<< cost.value() << "Cost"<< std::endl;
        //m_intermediate = true;
        // Check if mintime reached, else return false

        });
      }

      ompl::base::PlannerPtr kabitstar(const ompl::base::SpaceInformationPtr &si, std::string name)
      {
          ompl::geometric::ABITstar *planner = new og::ABITstar(si);
          planner->setName(name);
          planner->setPruning(false);
          //planner->setUseKNearest(false);
          //planner->setStopOnSolnImprovement(true);
          return ompl::base::PlannerPtr(planner);
      }
#if OMPL_BENCHMARK
      //! Function for performing benchmarks on a setup.
      void bmarkPath(og::SimpleSetup &ss, std::string &benchmark_name, double benchmark_maxTime, double benchmark_maxMem,int benchmark_runCount)
      {

          // Bencmarking code
          ompl::tools::Benchmark b(ss, benchmark_name);
          
          // Planners to be tested
          b.addPlannerAllocator(std::bind(&kabitstar, std::placeholders::_1, "kABITstar"));
          
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
}