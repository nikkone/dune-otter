//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <USER/DUNE.hpp>
// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
//#include "ompl/base/DiscreteMotionValidator.h"
#include <ompl/config.h>

#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/tools/benchmark/Benchmark.h> // Only used for benchmarking, remove before deployment

 #include <ompl/util/Time.h>
 #include <utility>
// #include <ctime>

// CPP headers
#include <iostream>
 
namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <algorithm> 
#include <chrono> 
#include <iostream> 
#include<vector> 

namespace MotionPlanners
{
  //! This task demonstrates how OMPL is used in DUNE along with ChartsDatabase
  //! @author Nikolai Lauvås
  namespace OMPL
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! The path of the database.
      std::string dbPath;
      //! How long a planner is run before terminated.
      double maxPlaningTime;
      //! If a valid path is available, allow optimizing until the given time. If non-optimizing planner used, this is ignored.
      double minPlaningTime;
      //! Defines the bounds of the area the path planner operates on.
      std::vector<double> planningBounds;
      //! Defines the start and end point to use while developing
      std::vector<double> startAndEnd;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Database connection
      ChartsDatabase::ChartsDBConnection* m_con;

      bool m_intermediate;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_con(NULL)
      {
        param("DB Path", m_args.dbPath)
        .defaultValue("")
        .description("Path of the db");

        param("Max Planning Time", m_args.maxPlaningTime)
        .units(DUNE::Units::Second)
        .defaultValue("60.0")
        .description("How long a planner is run before terminated");

        param("Min Planning Time", m_args.minPlaningTime)
        .units(DUNE::Units::Second)
        .defaultValue("10.0")
        .description("If a valid path is available, allow optimizing until the given time. If non-optimizing planner used, this is ignored.");

        param("Planning Bounds", m_args.planningBounds)
        .size(4)
        .defaultValue("63.407093, 10.369549, 63.463678, 10.426469")
        .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");

        param("Start and Goal", m_args.startAndEnd)
        .size(4)
        .defaultValue("63.44540, 10.38627, 63.41434, 10.38903")
        .description("A starting point and end point to use while developing");        
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        try{
          m_con = new ChartsDatabase::ChartsDBConnection(m_args.dbPath.c_str(), Database::Connection::CF_CREATE);
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          // Set task state to failure
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Set OMPL to use the console output of this task
        ompl::msg::OutputHandler *oh = new ChartsDatabase::OutputHandlerDUNEConsole(this);
        ompl::msg::useOutputHandler(oh);
        ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        try {
         Memory::clear(m_con);
        }
        catch(std::runtime_error& e) {
          err(DTR("Could not clear charts database class: %s"), e.what());
        }
      }

      //! Termination condition to be used with ompl. This one stops the planner when the task gets the stop signal, if the stop input is true or if maxTime is reached.
      ompl::base::PlannerTerminationCondition exactSolnPlannerTerminationCondition(const bool *stop, DUNE::Tasks::Task *inTask)
      {
        double duration = m_args.maxPlaningTime; // Maxtime
        const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
          return ompl::base::PlannerTerminationCondition([&, stop, inTask, endTime]
                                              {
                                                if(*stop) {
                                                  return true;
                                                }

                                                if(ompl::time::now() > endTime){
                                                  inTask->inf("maxtime");
                                                  return true;
                                                }

                                                  return isStopping();
                                              });
      }


      //! A function that returns the ompl::base::ReportIntermediateSolutionFn type.
      ompl::base::ReportIntermediateSolutionFn intermediate() {
        double duration = m_args.minPlaningTime; // Mintime
        const ompl::time::point endTime(ompl::time::now() + ompl::time::seconds(duration));
        return ompl::base::ReportIntermediateSolutionFn([&, endTime](const ob::Planner *planner, const std::vector< const ob::State * > &states, const ob::Cost cost) 
        { 
          if(ompl::time::now() > endTime){
            inf("mintime");
          }
          // Create ompl::geometric::PathGeometric from "states" vector
          auto path = ompl::geometric::PathGeometric(planner->getSpaceInformation());
          for (auto ptr = states.begin(); ptr < states.end(); ptr++) {
            auto state = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(*ptr);
            std::cout << state->values[1] <<","<< state->values[0] << std::endl;
            path.append(*ptr);
          }

              ChartsDatabase::DBTree* tree = new ChartsDatabase::DBTree(m_con);
              ChartsDatabase::pathToTree(path, "tree", tree);
              Memory::clear(tree);

          //intermediate(planner,states, cost); 
        std::cout << "intermediate solution with "<< cost.value() << "Cost"<< std::endl;
        m_intermediate = true;
        // Check if mintime reached, else return false

        });
      }

      static ompl::base::PlannerPtr kabitstar(const ompl::base::SpaceInformationPtr &si, std::string name)
      {
          ompl::geometric::ABITstar *planner = new og::ABITstar(si);
          planner->setName(name);
          planner->setPruning(false);
          planner->setStopOnSolnImprovement(true);
          return ompl::base::PlannerPtr(planner);
      }

      void activatePlan(std::string plan_id) {
        bool ignore_errors = true;
        IMC::PlanControl pcontrol;
        pcontrol.type = IMC::PlanControl::PC_REQUEST;
        pcontrol.op = IMC::PlanControl::PC_START;
        pcontrol.plan_id = plan_id;
        pcontrol.setDestination(m_ctx.resolver.id());
        if (ignore_errors)
          pcontrol.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
        dispatch(pcontrol);
        spew("Plan start request sent");
      }

      //! Function for performing benchmarks on a setup.
      void bmarkPath(og::SimpleSetup &ss)
      {

          // Bencmarking code
          ompl::tools::Benchmark b(ss, "kBITstar");
          
          // For planners that we want to configure in specific ways,
          // the ompl::base::PlannerAllocator should be used:
          b.addPlannerAllocator(std::bind(&kabitstar, std::placeholders::_1, "kABITstar"));
          
          ompl::tools::Benchmark::Request req = ompl::tools::Benchmark::Request();
          req.maxTime = m_args.maxPlaningTime;
          req.maxMem = 5000.0;
          req.runCount = 2;
          req.displayProgress = true;
          b.benchmark(req);
          
          // This will generate a file of the form ompl_host_time.log
          b.saveResultsToFile();
      }

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
          ChartsDatabase::DBTree* tree = new ChartsDatabase::DBTree(m_con);
          debug("Found solution");
          //ChartsDatabase::printPath(ss.getSolutionPath());
          ChartsDatabase::pathToTree(ss.getSolutionPath(), "tree", tree);
          // print the path to screen
          ss.simplifySolution();
          //ss.getSolutionPath().print(std::cout);
          og::PathGeometric states = ss.getSolutionPath();
          IMC::PlanDB pdb = ChartsDatabase::createPlanDBEntry(states, "autoPlan2", 1.0);
          dispatch(pdb);
          //sendPlan(states, "autoPlan", 1.0);
          activatePlan("autoPlan2");
          //pathToDB(states, "tree3");
          ChartsDatabase::pathToTree(ss.getSolutionPath(), "tree3", tree);
          Memory::clear(tree);
        }
        else
          war("No solution found");
      }

      //! Function searching for path, ends search once first valid path is available
      void findPath(og::SimpleSetup &ss) {
        //ss.setPlanner(ob::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));
        ob::PlannerPtr test= kabitstar( ss.getSpaceInformation(), "kABITstar");
        ss.setPlanner(test);
        ss.print();
          // attempt to solve the problem within a given planning time
        ob::PlannerStatus solved = ss.solve(m_args.maxPlaningTime);
        if (solved)
        {
          debug("Found solution");
          // Simplify path
          ss.simplifySolution();
          // Extract path
          og::PathGeometric states = ss.getSolutionPath();

          // Dispatch and activate returned path
          IMC::PlanDB pdb = ChartsDatabase::createPlanDBEntry(states, "autoPlan", 1.0);
          dispatch(pdb);
          activatePlan("autoPlan");

          // Write path to DB for visualization purposes
          ChartsDatabase::DBTree* tree = new ChartsDatabase::DBTree(m_con);
          ChartsDatabase::pathToTree(ss.getSolutionPath(), "tree3", tree);
          Memory::clear(tree);
        }
        else
          war("No solution found within %f seconds", m_args.maxPlaningTime);
      }

      //! Creates a simplesetup object using the ChartsDB polygons for state and motion validation.
      og::SimpleSetup createSetup(double startLat, double startLon, double goalLat, double goalLon, ob::RealVectorBounds &bounds) 
      {
        // Construct the state space
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));
        // Define bounds of searching space
        space->setBounds(bounds);

        // Define a simple setup class
        og::SimpleSetup ss(space);

        // Define Motion validator for this space
        //ss.getSpaceInformation()->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(ss.getSpaceInformation()));
        ss.getSpaceInformation()->setMotionValidator(std::make_shared<DUNE::ChartsDatabase::ChartsDBMotionValidator>(ss.getSpaceInformation(), m_con, 6));

        // Set state validity checking for this space
        ss.setStateValidityChecker([&](const ob::State *state) { return DUNE::ChartsDatabase::isStateValid(state, m_con); });
        // Set optimization objective to strive for if optimizing planner is used
        ss.setOptimizationObjective(std::make_shared<DUNE::ChartsDatabase::LatLonDist>(ss.getSpaceInformation()));

        // Create the start state
        ob::ScopedState<> start(space);
        start[0]=startLon;
        start[1]=startLat;

        // Create the goal state
        ob::ScopedState<> goal(space);
        goal[0]=goalLon;
        goal[1]=goalLat;

        // Set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        return ss;
      }

      //! Main loop.
      void
      onMain(void)
      {
        // Create bounds
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0,m_args.planningBounds[1]);
        bounds.setHigh(0,m_args.planningBounds[3]);
        bounds.setLow(1,m_args.planningBounds[0]);
        bounds.setHigh(1,m_args.planningBounds[2]);

        og::SimpleSetup setup = createSetup(m_args.startAndEnd[0],m_args.startAndEnd[1],m_args.startAndEnd[2],m_args.startAndEnd[3], bounds); // Ned nidelven

        // Run/benchmark current setup
        findPath(setup);
        //bmarkPath(setup);

        inf("findPath finnished");
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
