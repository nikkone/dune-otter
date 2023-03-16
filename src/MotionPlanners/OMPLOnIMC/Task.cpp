//***************************************************************************
// Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and the Department of Engineering          *
// Cybernetics at the Norwegian University of Science and Technology        *
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
#include <ENCGIS/isPointInLayerStatement.hpp>
#include <ENCGIS/lineIntersectLayerStatement.hpp>
//#include <ENCGIS/getClosestIntersectWithOffset.hpp>

// OMPL integration for DUNE
#include <OMPL/setup.hpp>
#include <OMPL/OMPLfunctions.hpp>

namespace MotionPlanners
{
  //! This task demonstrates how OMPL is used in DUNE along with ENCGIS
  //! @author Nikolai Lauvås
  namespace OMPLOnIMC
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! The path of the database.
      std::string dbPath;
      std::string resultsDBpath;
      //! Navigable Layer Name
      std::string dbNavigableLayerName;
      //! Innavigable Layer Name
      std::string dbInnavigableLayerName;
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
      ENCGIS::isPointInLayerStatement *pointCheck;
      ENCGIS::lineIntersectLayerStatement *lineCheck;
      bool m_intermediate;

      //! Target vehicle.
      uint32_t vehicle;
      //! Speed.
      fp32_t speed;
      //! Speed Units.
      uint8_t speed_units;
      //! How long a planner is run before terminated.
      double maxPlaningTime;
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

        param("DB Path", m_args.dbPath)
        .defaultValue("")
        .description("Path of the db");

        param("Navigable Layer Name", m_args.dbNavigableLayerName)
        .defaultValue("navigable")
        .description("Navigable Layer Name");

        param("Innavigable Layer Name", m_args.dbInnavigableLayerName)
        .defaultValue("innavigable")
        .description("Innavigable Layer Name");

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
        .defaultValue("unnamed")
        .description("Path of the db");

        param("Benchmark Runtime", m_args.benchmark_maxTime)
        .defaultValue("1")
        .description("Path of the db");

        param("Benchmark Max Memory", m_args.benchmark_maxMem)
        .defaultValue("100")
        .description("Path of the db");

        param("Benchmark Runs", m_args.benchmark_runCount)
        .defaultValue("1")
        .description("Path of the db"); 
#endif
        bind<IMC::PlanProbSpec>(this);
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
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          // Set task state to failure
        }
        try{
          pointCheck = new ENCGIS::isPointInLayerStatement(m_args.dbNavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for navigable layer: %s"), e.what());
          // Set task state to failure
        }

        try{
          lineCheck = new ENCGIS::lineIntersectLayerStatement(m_args.dbInnavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for innavigable layer: %s"), e.what());
          // Set task state to failure
        }  
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Set OMPL to use the console output of this task
        ompl::msg::OutputHandler *oh = new OMPLforDUNE::OutputHandlerDUNEConsole(this);
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

      void
      consume(const IMC::PlanProbSpec* msg)
      {
        spew("Message received");
        spew("Destination: %i", msg->getDestination());
        spew("Problem Type%i", msg->problem_type);

        // Only accept messages to this system
        if (msg->getDestination() != getSystemId())
          return;

        /*if (msg->getDestinationEntity() != getEntityId())
          return;*/

        // Only accept feasible path problems
        if (msg->problem_type != IMC::PlanProbSpec::TypeEnum::PPT_fpath)
          return;

        spew("Checking size");
        if(msg->area.size() != 2)
          return;
        spew("All checks correct");

        // Parse Custom Parameters
        /*
          Supported custom parameters:
            a = [0,x], activate resulting plan
            p = [], Planning algorithm/configuration to use, follows enum OMPLintegrationENCGIS::configurations_t
            t = [0.0,inf), Max planning time
        */
        DUNE::Utils::TupleList custom = DUNE::Utils::TupleList(msg->custom);
        std::map<std::string, std::string> custommap = custom.getMapReversed();
        unsigned planner = 0;
        auto parameterit = custommap.find("t");
        if (parameterit != custommap.end()) {
          try{
            maxPlaningTime = std::stof(parameterit->second);
            spew("Found t=%f", maxPlaningTime);
          } catch(...) {
            err("Parameter \'t\' not float");
          }
        }
        parameterit = custommap.find(std::string("p"));
        if (parameterit != custommap.end()) {
          spew("Found p=%s", parameterit->second.c_str());
          try{
            planner = std::stoul(parameterit->second);
          } catch(...) {
            err("Parameter \'p\' not unsigned");
          }
        }
        parameterit = custommap.find(std::string("a"));
        bool activateResultingPlan= false;
        if (parameterit != custommap.end()) {
          try{
            spew("Found a=%i", std::stoi(parameterit->second));
            activateResultingPlan = (std::stoi(parameterit->second)) ? true : false;
          } catch(...) {
            err("Parameter \a\' not bool(int)");
          }
        }
        // Store plan specific parameters
        vehicle = msg->vehicle;
        speed = msg->speed;
        speed_units = msg->speed_units;

        // Create planning bound
        IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = msg->area.begin();
          for (unsigned i = 0; itr != msg->area.end(); ++itr, ++i)
          {
            spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
          }
        itr = msg->area.begin();
        double planningBounds[4];
        m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[1], planningBounds[0], 32632);
        ++itr;
        m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[3], planningBounds[2], 32632);
        /*ob::RealVectorBounds bounds(2);

        bounds.setLow(0,planningBounds[0]);
        bounds.setHigh(0,planningBounds[2]);
        bounds.setLow(1,planningBounds[1]);
        bounds.setHigh(1,planningBounds[3]);*/

        // Convert from WGS-84 to EPSG32632
        double start_northing, start_easting, end_northing, end_easting;
        m_con->transformSRID(Math::Angles::degrees(msg->start_lon), Math::Angles::degrees(msg->start_lat), 4326, start_easting, start_northing, 32632);
        m_con->transformSRID(Math::Angles::degrees(msg->end_lon), Math::Angles::degrees(msg->end_lat), 4326, end_easting, end_northing, 32632);

        spew("Planning start/goal: %f, %f, %f, %f", start_easting, start_northing, end_easting, end_northing);
        spew("Args bounds:  %f, %f, %f, %f", m_args.planningBounds[1], m_args.planningBounds[3], m_args.planningBounds[0], m_args.planningBounds[2]);
        spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3]);

        //og::SimpleSetup setup = OMPLintegrationENCGIS::createSetup(start_easting, start_northing, end_easting, end_northing, bounds, pointCheck, lineCheck);
        og::SimpleSetup setup = OMPLintegrationENCGIS::createSetup(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], pointCheck, lineCheck);
        OMPLintegrationENCGIS::setStartAndGoalStates(setup, start_easting, start_northing, end_easting, end_northing);

        og::PathGeometric states = OMPLintegrationENCGIS::findPath(setup, maxPlaningTime, OMPLintegrationENCGIS::configurations_t(planner));

        if (states.getStateCount()) {
          auto planVec = OMPLforDUNE::pathToVector(states);
          auto planVec4326 = m_con->transformSRIDVector(planVec, 32632,4326);
            // Make maneuvers
            for(auto i = planVec4326.begin(); i < planVec4326.end();i++) {
                inf("%f, %f", i->first, i->second);
            }
          //// Dispatch and activate returned path
          IMC::PlanDB pdb = OMPLforDUNE::createPlanDBEntryUTM(states, "autoPlan", speed, 32);
          dispatch(pdb);
          if(activateResultingPlan) {
            activatePlan("autoPlan");
          }
        } else {
          err("Could not find valid path within %f seconds.", maxPlaningTime);
          err("Error finding path from: %f, %f to %f ,%f", start_easting, start_northing, end_easting, end_northing);
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

        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
