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

// Additional headers
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/SearchGridCoverageState.hpp>
#include <ENCGIS/SearchGridPlanner.hpp>

namespace Control
{
  //! This task implements a fish searching algorithm for 
  /*
  DONE: Create separate grid configurations for search and PD and CEFT
  DONE: Implement way to update desired position/maneuver etc.. 
  Done: Maneuver or FollowReference (TREX)
  DONE: Implement supersampling/downsampling
  Integrate greedy approaches for choosing desired positions
  Implement better combination of effort and prior
  Implement a StationKeeping Operation
  Implement interface to change between stationKeeping and GoTo
  Implement random path generator to unsearched cells
  Fjerne negative vekter (max(weight, 0))
  Make effort/prior grid cover planning grid.
  Legg til flere modus for å finne best cell
  ?? OMPL integration?
  */
  //! @author Nikolai Lauvås
  namespace Search
  {
    namespace FishSearch
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //!
        std::string encDBpath;
        //!
        std::string effortDBpath;
        //!
        float initialSensorRange;
        //! Size of the grid cells
        unsigned gridSize;
        //! Geometry type of grid, see ENCGIS::SearchGrid::gridtypes_t
        unsigned gridType;
        //!
        std::vector<std::string> otherVehicles;
        //!
        float timestepConstantDecrease;
        //!
        float timestepFactorDecrease;
      };
        struct Task: public DUNE::Tasks::Periodic
        {
        //! Task arguments.
        Arguments m_args;
        //! Database connection
        ENCGIS::DBconnection* m_con;
        //!
        ENCGIS::SearchGrid* m_searchGrid;
        //!
        ENCGIS::SearchGridCoverageState* m_searchGridCoverage;
        //!
        ENCGIS::SearchGridPlanner* m_GridPlanner;
        //!
        std::map<uint16_t, std::pair<double,double>> pendingUpdates;
        //!
        std::vector<uint16_t> monitoredVehicles;
        //!
        unsigned rpm;

        //! Size of the grid cells
        unsigned m_gridSize;
        //! Geometry type of grid, see ENCGIS::SearchGrid::gridtypes_t
        unsigned m_gridType;
        //!
        unsigned m_planner = 0;
        //!
        float m_distanceWeight = 0;
        //!
        float m_azimuthWeight = 0;
        //! Last plan control state
        IMC::PlanControlState m_last_plan_state;
        //! Latest estimated state from self
        IMC::EstimatedState m_esta;
        //!
        bool m_fishSearch_control;

        //! Store latest transmitted reference
        IMC::Reference m_cur_ref;
        IMC::Reference m_last_ref;
        IMC::FollowRefState m_last_follow_ref;

        int m_lastVisitedCell;

        unsigned m_hooverStartRunCount;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
            DUNE::Tasks::Periodic(name, ctx),
            m_con(NULL),
            m_searchGrid(NULL),
            m_searchGridCoverage(NULL),
            m_fishSearch_control(false)
        {
            param("Other Vehicles", m_args.otherVehicles)
            .description("The source/vehicle names of other entities in the system.")
            .defaultValue("");

            param("Initial Sensor Range", m_args.initialSensorRange)
            .description("The initial maximum range of the sensor")
            .defaultValue("50.0");  

            param("Timestep Grid Constant Decrease", m_args.timestepConstantDecrease)
            .defaultValue("0.0")
            .description("The value substracted from each grid cell at each timestep.");

            param("Timestep Grid Factor Decrease", m_args.timestepFactorDecrease)
            .defaultValue("1.0")
            .description("The value multiplied with each grid cell at each timestep. [0.0, 1.0]");

            param("ENC DB Path", m_args.encDBpath)
            .defaultValue("")
            .description("The path of the DB to read ENC from.");

            param("Working DB Path", m_args.effortDBpath)
            .defaultValue("")
            .description("The path of a DB to store the effort and prior distribution grid in.");

            param("Grid Size", m_args.gridSize)
            .defaultValue("50")
            .description("Size of the grid cells");

            param("Grid Type", m_args.gridType)
            .defaultValue("1")
            .description("Geometry type of grid, 0=HEX, 1=Square, 2=Triangular.");


            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
            bind<IMC::EstimatedState>(this);
            bind<IMC::PlanProbSpec>(this);
            bind<IMC::Rpm>(this);
            bind<IMC::VehicleState>(this);
            bind<IMC::Abort>(this);
            bind<IMC::PlanControl>(this);
            bind<IMC::PlanControlState>(this);
            bind<IMC::FollowRefState>(this);


        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
            if(paramChanged(m_args.otherVehicles)) {
            monitoredVehicles.clear();
            for(auto iter = m_args.otherVehicles.begin(); iter != m_args.otherVehicles.end(); iter++) {
                monitoredVehicles.push_back(resolveSystemName(*iter));
            }
            for(auto iter = monitoredVehicles.begin(); iter != monitoredVehicles.end(); iter++) {
                inf("%u", *iter);
            }
            }
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
          std::string attachedDb = "db1";
          try{
            m_con = new ENCGIS::DBconnection(m_args.effortDBpath, SQLITE_OPEN_READWRITE, 32632);
            m_con->runNoOutputQuery("attach '" + m_args.encDBpath + "' as " + attachedDb + "");
            //m_con->runQuery("select * from db1.coalne limit 10");
          } catch(std::runtime_error& e) {
            err(DTR("Problem opening charts database: %s"), e.what());
            // Set task state to failure
          }

          m_searchGrid = new ENCGIS::SearchGrid(m_con, "FishSearch");
          m_searchGridCoverage = new ENCGIS::SearchGridCoverageState(m_con, std::string("coverage"));
          
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {

        }

        //! Release resources.
        void
        onResourceRelease(void) {
            inf("Release");
            //if(m_searchGridCoverage != NULL)
            //  m_searchGridCoverage->deleteGrid();
            //if(m_searchGrid != NULL)
            //  m_searchGrid->deleteGrid();
            try {
            Memory::clear(m_con);
            Memory::clear(m_searchGrid);
            Memory::clear(m_searchGridCoverage);
            }
            catch(std::runtime_error& e) {
            err(DTR("Could not clear charts database class: %s"), e.what());
            }
        }


      void
      onActivation(void)
      {
        inf("Starting FishSearch plan...");
        IMC::PlanControl startPlan;
        startPlan.type = IMC::PlanControl::PC_REQUEST;
        startPlan.op = IMC::PlanControl::PC_START;
        startPlan.plan_id = "fishSearch_plan";
        IMC::FollowReference man;
        man.control_ent = getEntityId();
        man.control_src = getSystemId();
        man.altitude_interval = 0;
        man.timeout = 10;

        IMC::PlanSpecification spec;

        spec.plan_id = "fishSearch_plan";
        spec.start_man_id = "follow_fishSearch";

        IMC::PlanManeuver pm;
        pm.data.set(man);
        pm.maneuver_id = "follow_fishSearch";
        spec.maneuvers.push_back(pm);
        startPlan.arg.set(spec);
        startPlan.request_id = 0;
        startPlan.flags = 0;
        startPlan.setDestination(m_ctx.resolver.id());
        dispatch(startPlan);

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      onDeactivation(void)
      {
        inf("%s", DTR(Status::getString(Status::CODE_IDLE)));

        inf("Stopping fishSearch_plan plan.");
        IMC::PlanControl stopPlan;
        stopPlan.type = IMC::PlanControl::PC_REQUEST;
        stopPlan.op = IMC::PlanControl::PC_STOP;
        stopPlan.plan_id = "fishSearch_plan";
        dispatch(stopPlan);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }
      
      void
      consume(const IMC::FollowRefState * msg) {
        if(isActive()) {
          if(msg->control_ent == getEntityId()) {
            m_last_follow_ref = *msg;
          }
        }
      }


      void
      consume(const IMC::VehicleState * msg)
      {
        // if the vehicle is in error mode, set FishSearch task to inactive
        if (msg->op_mode == IMC::VehicleState::VS_ERROR)
          requestDeactivation();
      }

      void
      consume(const IMC::Abort* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;
        
        war(DTR("Abort detected. Disabling FishSearch control."));
        requestDeactivation();
      }

      void
      consume(const IMC::PlanControlState * msg)
      {
        m_last_plan_state = *msg;

        m_fishSearch_control = msg->state == IMC::PlanControlState::PCS_EXECUTING
        && msg->plan_id == "fishSearch_plan";
      }

      void
      consume(const IMC::PlanControl* msg)
      {
        if (msg->type == PlanControl::PC_REQUEST && msg->op == PlanControl::PC_STOP)
        {
          m_fishSearch_control = m_last_plan_state.plan_id == "fishSearch_plan"
          && m_last_plan_state.state == IMC::PlanControlState::PCS_EXECUTING;

          if (m_fishSearch_control)
          {
            requestDeactivation();
            war(DTR("Stop fishSearch detected. Disabling control."));
          }
        }
        if(isActive()) {
          if (msg->type == PlanControl::PC_FAILURE && msg->plan_id == "fishSearch_plan") {
            requestDeactivation();
              war(DTR("Failure in PlanControl detected during fishSearch, disabling control."));
          }
        }

      }

        void consume(const IMC::EstimatedState* msg) {
          if(msg->getSource() != getSystemId()) {
            if(!monitoredVehicles.empty()) {
              if(std::find(monitoredVehicles.begin(), monitoredVehicles.end(), msg->getSource()) == monitoredVehicles.end()) {
                return;
              }
            } // Else update on all vehicles
          } else {
            // Store latest vehicle position
            m_esta = *msg;
          }
            pendingUpdates[msg->getSource()] = std::pair<double,double>(msg->lon, msg->lat);
        }

        void consume(const IMC::Rpm* msg) {
            rpm = std::abs(msg->value); // TODO: Need to keep track of rpm for all vehicles
        }

        std::string polygonToEWKT(const IMC::MessageList<IMC::PolygonVertex> &polygon) {
          std::string EWKT = "SRID=4326;POLYGON((";
          for(IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = polygon.begin();itr < polygon.end();itr++) {
              //spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
              EWKT += std::to_string(DUNE::Math::Angles::degrees((*itr)->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees((*itr)->lat)) + ",";
          }
          EWKT += std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lat)) + "))";
          return EWKT;
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

          // Only proceed for feasible path problems
          if (msg->problem_type != IMC::PlanProbSpec::TypeEnum::PPT_coverage)
            return;

          spew("Starting processing");
          // Parse Custom Parameters
          /*
          Supported custom parameters:
              a = [0,x], activate resulting plan
              gg = Grid geometry
              gs = Grid geometry edge size
              p = Planner
              paw = Planner azimuth weight
              pdw = Planner distance weight
          */
          DUNE::Utils::TupleList custom = DUNE::Utils::TupleList(msg->custom);
          std::map<std::string, std::string> custommap = custom.getMapReversed();

          auto parameterit = custommap.find(std::string("a"));
/*          bool activateResultingPlan= false;
          if (parameterit != custommap.end()) {
          try{
              spew("Found a=%i", std::stoi(parameterit->second));
              activateResultingPlan = (std::stoi(parameterit->second)) ? true : false;
          } catch(...) {
              err("Parameter \a\' not bool(int)");
          }
          }*/

          parameterit = custommap.find(std::string("gg"));
          if (parameterit != custommap.end()) {
          spew("Found gg=%s", parameterit->second.c_str());
          try{
              m_gridType = std::stoul(parameterit->second);
          } catch(...) {
              err("Parameter \'gg\' not unsigned");
          }
          }

          parameterit = custommap.find(std::string("gs"));
          if (parameterit != custommap.end()) {
          spew("Found gs=%s", parameterit->second.c_str());
          try{
              m_gridSize = std::stof(parameterit->second);
          } catch(...) {
              err("Parameter \'gs\' not float");
          }
          }
          parameterit = custommap.find(std::string("p"));
          if (parameterit != custommap.end()) {
          try{
              m_planner = std::stoi(parameterit->second);
              spew("Found p=%d", m_planner);
          } catch(...) {
              err("Parameter \'p\' not unsigned");
          }
          }

          parameterit = custommap.find(std::string("paw"));
          if (parameterit != custommap.end()) {
          spew("Found paw=%s", parameterit->second.c_str());
          try{
              m_azimuthWeight = std::stof(parameterit->second);
          } catch(...) {
              err("Parameter \'paw\' not unsigned");
          }
          }

          parameterit = custommap.find(std::string("pdw"));
          if (parameterit != custommap.end()) {
          spew("Found pdw=%s", parameterit->second.c_str());
          try{
              m_distanceWeight = std::stof(parameterit->second);
          } catch(...) {
              err("Parameter \'pdw\' not unsigned");
          }
          }
          // Start time for grid creation.
          //auto startg = std::chrono::high_resolution_clock::now();

          // Convert from WGS-84 to EPSG32632
          double start_northing, start_easting;
          m_con->transformSRID(Math::Angles::degrees(msg->start_lon), Math::Angles::degrees(msg->start_lat), 4326, start_easting, start_northing, 32632);
          spew("Planning start: %f, %f", start_easting, start_northing);

          spew("Checking size");
          if(msg->area.size() == 2) {
              m_searchGrid->deleteGrid();
              m_searchGridCoverage->deleteGrid();
              // Create planning bound
              IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = msg->area.begin();
              for (unsigned i = 0; itr != msg->area.end(); ++itr, ++i)
              {
                  spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
              }
              itr = msg->area.begin();
              double planningBounds[4];
              m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[0], planningBounds[1], 32632);
              ++itr;
              spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[2], planningBounds[1], planningBounds[3]);
              m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[2], planningBounds[3], 32632);
              m_searchGrid->createGrid(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType));
              m_searchGridCoverage->createGrid(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType), true);
          } else if(msg->area.size() > 2) {
              m_searchGrid->deleteGrid();
              m_searchGridCoverage->deleteGrid();
              std::string EWKT = polygonToEWKT(msg->area);
              m_searchGrid->createGrid(EWKT, m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType));
              m_searchGridCoverage->createGrid(EWKT, m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType), true);
              m_searchGridCoverage->updateDetectionProbability("fishsearch");
              debug("Grid Created from EKWT");
              
              spew("Weights of grid set");
          } else {
              spew("Polygon too small.");
              return;
          }
          // Create prior distribution from land distance
          m_searchGridCoverage->setGridMetricFromLandDistance();
          m_searchGridCoverage->normalizeMetric(true);


          m_lastVisitedCell = m_searchGrid->getClosestCell(start_easting, start_northing);
          auto initial_pos = m_searchGrid->getCellLocation(m_lastVisitedCell);
          m_cur_ref.lon = DUNE::Math::Angles::radians(initial_pos.first);
          m_cur_ref.lat = DUNE::Math::Angles::radians(initial_pos.second);
          dispatch(m_cur_ref);

          m_GridPlanner = new ENCGIS::SearchGridPlanner(m_searchGrid);
          // Find coverage path
          m_GridPlanner->setinitialCell(m_lastVisitedCell);
          m_GridPlanner->setinitialAzimuth(0.0);
          m_GridPlanner->setdistibutionWeight(1);
          m_GridPlanner->setazimuthWeight(m_azimuthWeight);
          m_GridPlanner->setdistanceWeight(m_distanceWeight);

          // Cumulative search effort tracking setup
          pendingUpdates.clear();
          if(!isActive()) {
            requestActivation();
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }
          m_hooverStartRunCount = 0;
          inf("init");
          
        }

        int calculateNextCell(int lastVisitedCell) {
            //m_lastVisitedCell = m_GridPlanner->getGlobalOptimalCell(m_lastVisitedCell, 0.0);
            int nextCell = 0;
            nextCell = m_GridPlanner->getLocalOptimalNeighbour(lastVisitedCell);
            if(m_lastVisitedCell == 0) {
              nextCell = m_searchGrid->getClosestUnsearchedCell(lastVisitedCell);
            }
            return nextCell;
        }

        //! Main loop.
        void
        task(void)
        {
          consumeMessages();
          if(isActive()) {
            // Update cummulative effort
            //m_searchGridCoverage->decreaseAll(m_args.timestepDecrease, std::string("effort"));
            //m_searchGridCoverage->decreaseAll(m_args.timestepConstantDecrease, m_args.timestepFactorDecrease, std::string("effort"));
            for(auto iter = pendingUpdates.begin();iter != pendingUpdates.end();iter++) {
              // Convert from WGS-84 to EPSG32632
              double northing, easting;
              m_con->transformSRID(Math::Angles::degrees(iter->second.first), Math::Angles::degrees(iter->second.second), 4326, easting, northing, 32632);
              //m_searchGridCoverage->update(easting, northing, m_args.initialSensorRange);
              m_searchGridCoverage->updateLogarithmic(easting, northing, rpm, (1/getFrequency())/14, 670, 0.05, std::string("effort")); // Assuming 90 sec is needed to guarantee detection
              inf("%f %f", easting, northing);
            }
            pendingUpdates.clear();

            m_searchGridCoverage->updateDetectionProbability("fishsearch");
            m_searchGrid->normalizeMetric(false);

            // Planner update
            spew("Frefstate: %u", m_last_follow_ref.state);

              
              
            switch(m_last_follow_ref.state) {
              //! Waiting for first reference. (Set at when parsing planProbSpec)
              case IMC::FollowRefState::FR_WAIT:
              break;
              //! Going towards received reference.
              case IMC::FollowRefState::FR_GOTO: // Could look at proximity to start calculating next step already
              break;
              //! Loitering after arriving at the reference.
              case IMC::FollowRefState::FR_LOITER:
              break;
              //! Hovering after arriving at the reference.
              case IMC::FollowRefState::FR_HOVER:
                if(m_hooverStartRunCount) {
                  if(getRunCount() - m_hooverStartRunCount > 14) {
                    m_hooverStartRunCount = 0;
                    // Run planner
                    m_lastVisitedCell = calculateNextCell(m_lastVisitedCell);
                    if(!m_lastVisitedCell) { // If zero is the last visited cell, the search is finished
                      spew("Zero cell received, ending planner");
                      requestDeactivation();
                      return;
                    }
                    
                    // Get position of next cell and set it as next position to go to
                    std::pair<double, double> initial_pos = m_searchGrid->getCellLocation(m_lastVisitedCell);
                    m_cur_ref.lon = DUNE::Math::Angles::radians(initial_pos.first);
                    m_cur_ref.lat = DUNE::Math::Angles::radians(initial_pos.second);
                  }
                } else {
                  m_hooverStartRunCount = getRunCount();
                }
              break;
              //! Moving in z after arriving at the target cylinder. (Not used)
              case IMC::FollowRefState::FR_ELEVATOR:
              break;
              //! Controlling system timed out. Only occurs when this task is not active, so this should never happen.
              case IMC::FollowRefState::FR_TIMEOUT:
                requestDeactivation();
              break;
            }
            dispatch(m_cur_ref);
/*
            if(m_last_follow_ref.state == IMC::FollowRefState::FR_HOVER) {
              // Wait X time before updating reference
            }



            inf("Runtime: %u", getRunCount());
            if(getRunCount()%40 == 0 || getRunCount() < 20) {
              double curlat = m_esta.lat;
              double curlon = m_esta.lon;
              double trash = 0.0;
              DUNE::Coordinates::WGS84::displace(30.0,30.0,0.0,&curlat, &curlon, &trash);
              m_cur_ref.lat = curlat;
              m_cur_ref.lon = curlon;
              dispatch(m_cur_ref);
              m_searchGridCoverage->updateDetectionProbability("fishsearch");
            } else {
              dispatch(m_cur_ref);
            }
*/
            inf("Time Update");
          }
        }
      };
    }
  }
}
DUNE_TASK

