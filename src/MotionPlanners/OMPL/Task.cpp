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

#define BENCHMARK 1

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/DBTree.hpp>
#include "OMPLfunctions.hpp"
#include "OMPLMotionValidator.hpp"
#include "OMPLMotionValidator2.hpp"
#include "LatLonOptimizationObjective.hpp"

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
//#include "ompl/base/DiscreteMotionValidator.h"
#include <ompl/config.h>

#include <ompl/geometric/planners/informedtrees/ABITstar.h>

#if BENCHMARK
#include <ompl/tools/benchmark/Benchmark.h> // Only used for benchmarking, remove before deployment
#endif

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
#if BENCHMARK
      std::string benchmark_name;
      double benchmark_maxTime;
      double benchmark_maxMem;
      int benchmark_runCount;
#endif
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Database connection
      ENCGIS::DBconnection* m_con;
      ENCGIS::isPointInLayer2       *qry;//("deparetable", m_con->db);
      ENCGIS::checkTransectLanding2 *qry2;//2("lndaretable", m_con->db);
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
#if BENCHMARK
        param("Benchmark Name", m_args.benchmark_name)
        .defaultValue("")
        .description("Path of the db");

        param("Benchmark Runtime", m_args.benchmark_maxTime)
        .defaultValue("")
        .description("Path of the db");

        param("Benchmark Max Memory", m_args.benchmark_maxMem)
        .defaultValue("")
        .description("Path of the db");

        param("Benchmark Runs", m_args.benchmark_runCount)
        .defaultValue("")
        .description("Path of the db"); 
#endif
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
          m_con = new ENCGIS::DBconnection(m_args.dbPath);
          qry =  new ENCGIS::isPointInLayer2("deparetable", m_con->db);
          qry2 = new ENCGIS::checkTransectLanding2("lndaretable", m_con->db);
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
        ompl::msg::OutputHandler *oh = new MotionPlanners::OMPL::OutputHandlerDUNEConsole(this);
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
            //std::cout << state->values[1] <<","<< state->values[0] << std::endl;
            path.append(*ptr);
          }

              ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_con);
              MotionPlanners::OMPL::pathToTree(path, "tree", tree);
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
          //planner->setUseKNearest(false);
          //planner->setStopOnSolnImprovement(true);
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
#if BENCHMARK
      //! Function for performing benchmarks on a setup.
      void bmarkPath(og::SimpleSetup &ss)
      {

          // Bencmarking code
          ompl::tools::Benchmark b(ss, m_args.benchmark_name);
          
          // For planners that we want to configure in specific ways,
          // the ompl::base::PlannerAllocator should be used:
          b.addPlannerAllocator(std::bind(&kabitstar, std::placeholders::_1, "kABITstar"));
          
          ompl::tools::Benchmark::Request req = ompl::tools::Benchmark::Request();
          req.maxTime = m_args.benchmark_maxTime;
          req.maxMem = m_args.benchmark_maxMem;
          req.runCount = m_args.benchmark_runCount;
          req.displayProgress = true;
          b.benchmark(req);
          
          // This will generate a file of the form ompl_host_time.log
          b.saveResultsToFile();
      }
#endif
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

      //! Function searching for path, ends search once first valid path is available
      void findPath(og::SimpleSetup &ss) {
        //ss.setPlanner(ob::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));
        ob::PlannerPtr test= kabitstar( ss.getSpaceInformation(), "kABITstar");
        ss.setPlanner(test);
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
          IMC::PlanDB pdb = MotionPlanners::OMPL::createPlanDBEntry(states, "autoPlan", 1.0);
          dispatch(pdb);
          activatePlan("autoPlan");

          // Write path to DB for visualization purposes
          ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_con);
          MotionPlanners::OMPL::pathToTree(ss.getSolutionPath(), "tree3", tree);
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
        ss.getSpaceInformation()->setMotionValidator(std::make_shared<MotionPlanners::OMPL::ChartsDBMotionValidator2>(ss.getSpaceInformation(), qry2, 6));
      //ss.getSpaceInformation()->setMotionValidator(std::make_shared<MotionPlanners::OMPL::ChartsDBMotionValidator>(ss.getSpaceInformation(), m_con, 6));


        // Set state validity checking for this space
        ss.setStateValidityChecker([&](const ob::State *state) { return isStateValid(state, m_con); });
        //ss.setStateValidityChecker([&](const ob::State *state) { return MotionPlanners::OMPL::isStateValid(state, m_con); });

        // Set optimization objective to strive for if optimizing planner is used
        ss.setOptimizationObjective(std::make_shared<MotionPlanners::OMPL::LatLonDist>(ss.getSpaceInformation()));

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
    bool isStateValid(const ompl::base::State *state,  ENCGIS::DBconnection *dbCon)
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
        #if BENCHMARK
          bmarkPath(setup);
        #else
        findPath(setup);
        #endif
        //findPathExtended(setup);
        
        //std::cout << isPointInLayer(63.8091, 8.9979, "deparetable") << std::endl;
        //char* c_stmt = "select ID, X(geom), Y(geom) from tree3 where id=?;";
        /*
        char c_stmt[] = "select sum(intersects(MakePoint(?1, ?2, 4326), geom)) as c from (SELECT geom FROM deparetable WHERE ROWID IN (SELECT ROWID FROM SpatialIndex WHERE f_table_name = 'deparetable' AND search_frame = BuildMbr(?1, ?2,?1,?2, 4326)));";

        sqlite3_stmt* m_handle;
        
        printf("%d \n" ,sqlite3_prepare_v2(m_con->db, c_stmt, -1,&m_handle, 0));
    int v1, v2;
    // Get starting timepoint 
    auto start = std::chrono::high_resolution_clock::now(); 
  
        //printf("The statement has %d parameter(s).\n",sqlite3_bind_parameter_count(m_handle));
        //printf("%d \n", sqlite3_bind_text(m_handle,1,"tree1",5,NULL));
        //printf("%d \n", sqlite3_bind_int(m_handle,1,1));
        //printf("%d \n", sqlite3_bind_double(m_handle,1,8.9979));
        //printf("%d \n", sqlite3_bind_double(m_handle,2,63.8091));
        sqlite3_bind_double(m_handle,1,8.9979);
        sqlite3_bind_double(m_handle,2,63.8091);
      // Execute
      if(sqlite3_step(m_handle) == SQLITE_ROW) {
        v1 = sqlite3_column_int(m_handle, 0);
      }
        sqlite3_clear_bindings(m_handle);
        sqlite3_reset(m_handle);
        //printf("%d \n", sqlite3_bind_double(m_handle,1,9.06304));
        //printf("%d \n", sqlite3_bind_double(m_handle,2,63.97589));
        sqlite3_bind_double(m_handle,1,9.06304);
        sqlite3_bind_double(m_handle,2,63.97589);
      // Execute
      if(sqlite3_step(m_handle) == SQLITE_ROW) {
        v2 = sqlite3_column_int(m_handle, 0);
      }
      // Get ending timepoint 
      auto stop = std::chrono::high_resolution_clock::now(); 
    
      // Get duration. Substart timepoints to  
      // get durarion. To cast it to proper unit 
      // use duration cast method 
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
    
      std::cout << "Time taken by function: "
          << duration.count() << " microseconds. Result: "<< v1 << "," << v2<< std::endl; 


        // Teardown
        if (m_handle)
          sqlite3_finalize(m_handle);
*/
/*
        int v[3];

        ENCGIS::isPointInLayer2 qry("deparetable", m_con->db);
        ENCGIS::checkTransectLanding2 qry2("lndaretable", m_con->db);
    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0;i<1000;i++) {
        v[0] = qry2.run(64.00922, 9.15689, 63.98924, 9.17263);
        v[1] = qry2.run(64.00922, 9.15689, 64.00995, 9.15842);
        //v[0]= qry.run(63.97589, 9.06304);
        //v[1]= qry.run(63.8091, 8.9979);
        //v[2]= qry.run(42.51,2.62);

        
    }
        //v1 = m_con->isPointInLayer(63.8091, 8.9979, "deparetable");
        //v2 = m_con->isPointInLayer(63.97589, 9.06304, "deparetable");
      // Get ending timepoint 
      auto stop = std::chrono::high_resolution_clock::now(); 
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
//qry.run(63.97589, 9.06304);
      std::cout << "Time taken by function: " << duration.count() << " microseconds. Result: "<< v[0] << "," << v[1] << "," << v[2]<< std::endl; 


            // Get starting timepoint 
    auto start1 = std::chrono::high_resolution_clock::now();
        //1= qry.run(63.8091, 8.9979);
        //2= qry.run(63.97589, 9.06304);
        for(int i=0;i<1000;i++) {
          v[0] = m_con->checkTransectLanding(64.00922, 9.15689, 63.98924, 9.17263);
          v[1] = m_con->checkTransectLanding(64.00922, 9.15689, 64.00995, 9.15842);
          //v[0] = m_con->isPointInLayer(63.97589, 9.06304, "deparetable");
          //v[1] = m_con->isPointInLayer(63.8091, 8.9979, "deparetable");
          //v[2] = m_con->isPointInLayer(42.51,2.62, "deparetable");
        }

      // Get ending timepoint 
      auto stop1 = std::chrono::high_resolution_clock::now(); 
      auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1); 
          std::cout << "Time taken by function: "
          << duration1.count() << " microseconds. Result: "<< v[0] << "," << v[1] << "," << v[2]<< std::endl; 
*/
        inf("findPath finnished");
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
  /*std::string isPointInLayer(double lat, double lon, std::string table) {
        return "select sum(intersects(MakePoint(" + std::to_string(lon) + ", " + std::to_string(lat) + ", 4326), geom)) as c from (SELECT geom FROM " + table + " "
      "WHERE ROWID IN ("
        "SELECT ROWID FROM SpatialIndex "
        "WHERE f_table_name = '" + table + "' AND "
          "search_frame = BuildMbr(" + std::to_string(lon) + ", " + std::to_string(lat) + "," + std::to_string(lon) + ", " + std::to_string(lat) + ", 4326)));"; 
  }*/
    };
  }
}

DUNE_TASK
