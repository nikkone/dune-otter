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

// ENC database to use with the OMPL integration for DUNE
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/DBTree.hpp>

// OMPL integration for DUNE
#include <OMPL/setup.hpp>

namespace MotionPlanners
{
  //! This task demonstrates how OMPL is used in DUNE along with ENCGIS
  //! @author Nikolai Lauvås
  namespace OMPL
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! The path of the database.
      std::string dbPath;
      std::string resultsDBpath;
      //! How long a planner is run before terminated.
      double maxPlaningTime;
      //! If a valid path is available, allow optimizing until the given time. If non-optimizing planner used, this is ignored.
      double minPlaningTime;
      //! Defines the bounds of the area the path planner operates on.
      std::vector<double> planningBounds;
      //! Defines the start and end point to use while developing
      std::vector<double> startAndEnd;
#if OMPL_BENCHMARK
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
      ENCGIS::isPointInLayerStatement       *pointCheck;
      ENCGIS::lineIntersectLayerStatement *lineCheck;
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
      
        param("Result DB Path", m_args.resultsDBpath)
        .defaultValue("")
        .description("If set, the results are written to trees in this db.");

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
        .defaultValue("568399.476507, 7031678.685762, 571101.488332, 7038044.467683")
        .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");

        param("Start and Goal", m_args.startAndEnd)
        .size(4)
        .defaultValue("569142.113652, 7035964.208531, 569354.798021, 7032506.975707")
        .description("A starting point and end point to use while developing");   
#if OMPL_BENCHMARK
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
          m_con = new ENCGIS::DBconnection(m_args.dbPath, SQLITE_OPEN_READONLY, 32632);
          pointCheck = new ENCGIS::isPointInLayerStatement("deparepolygon", "geometry", m_con->db, 32632);
          lineCheck = new ENCGIS::lineIntersectLayerStatement("lndarepolygon", "geometry", m_con->db,32632);
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

        //og::SimpleSetup setup = createSetup(m_args.startAndEnd[0],m_args.startAndEnd[1],m_args.startAndEnd[2],m_args.startAndEnd[3], bounds, pointCheck, lineCheck); // Ned nidelven
        og::SimpleSetup setup = OMPLintegrationDUNE::createSetup(m_args.startAndEnd[0],m_args.startAndEnd[1],m_args.startAndEnd[2],m_args.startAndEnd[3], bounds, pointCheck, lineCheck); // Ned nidelven

        // Run/benchmark current setup
        #if OMPL_BENCHMARK
          OMPLintegrationDUNE::bmarkPath(setup, m_args.benchmark_name, m_args.benchmark_maxTime, m_args.benchmark_maxMem, m_args.benchmark_runCount);
        #else
          og::PathGeometric states = OMPLintegrationDUNE::findPath(setup, m_args.maxPlaningTime);
          //// Dispatch and activate returned path
          IMC::PlanDB pdb = MotionPlanners::OMPL::createPlanDBEntryUTM(states, "autoPlan", 1.0, 32);
          dispatch(pdb);
          activatePlan("autoPlan");
          //MotionPlanners::OMPL::printPath(states);
          // Write path to DB for visualization purposes
          ENCGIS::DBconnection* m_writable = new ENCGIS::DBconnection(m_args.resultsDBpath, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 32632);
          ENCGIS::DBTree* tree = new ENCGIS::DBTree(m_writable);
          m_writable->runNoOutputQuery("select InitSpatialMetadata(1);");
          tree->resetTree("tree");
          tree->createTree("tree");
          MotionPlanners::OMPL::pathToTree(states, "tree", tree);
          inf("Wrote to tree");
          Memory::clear(tree);
          Memory::clear(m_writable);
        #endif
        //findPathExtended(setup);
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
