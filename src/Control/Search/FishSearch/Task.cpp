//***************************************************************************
// Copyright 2022-2023 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
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
#if SEARCHGRID_USEOPP_OMPL
  #include <ENCGIS/isPointInLayerStatement.hpp>
  #include <ENCGIS/lineIntersectLayerStatement.hpp>

  // OMPL integration for DUNE
  #include <OMPL/setup.hpp>
  #include <OMPL/OMPLfunctions.hpp>
#endif

#include <cmath>
namespace Control
{
  //! This task implements a fish searching algorithm that continously updates an effort map.
  //! Next step is found by using a cost function with prior distribution, past effort and required motion (Translational/rotational).
  //! Relies on Spatialite integration to remove land cells from grid, and uses OMPL to verify/create paths between cells.
  /*
  DONE: Create separate grid configurations for search and PD and CEFT
  DONE: Implement way to update desired position/maneuver etc.. 
  DONE: Maneuver or FollowReference (TREX)
  DONE: Implement supersampling/downsampling
  DONE: Integrate greedy approaches for choosing desired positions
  DONE: OMPL, and set is as parameter instead of using the define
  DONE: Start calculations once HOVERING is started
  DONE: Fjerne negative vekter (max(weight, 0))
  DONE: Make effort/prior grid cover planning grid.
  DONE: Legg til flere modus for å finne best cell
  DONE: Implement a StationKeeping Operation
  DONE: Only need to perform fishSearch update before next cell is to be found. 
  DONE: Pause/continue implementation?
  DONE: Ensure numeric types in database are used
  DONE: Implement better/reasoning combination of effort and prior. RESULT: Using Bayes rule
  DONE: Oppdatere Neptus interface med nye parametre
  
Fix:
  Check for and implement SpatialIndex amd ramge limits for all operations
  Nye queries i stedet for -1 som blir brukt i offlineplanner
  Mulig problem FollowRef timeout hvis for lang OMPL planning time.
  Need to keep track of rpm for all vehicles
  Problem med finne sti i starten av venteperiode, tar ikke hensyn til effort gjort i venteperiode. tar ikke hensyn til evt rotasjon som skjer etterpå

TODO: 
  Avslutt med stationkeep.
  Implement random path generator to unsearched cells
  Implement interface to change between stationKeeping and GoTo
  Test OMPL disable
  Ny FishSearch planner type i IMC
  Sjekk om hover fungerer som stationKeep, og evt. hvor radius settes

Consider:
  Run fishsearch update at XY_NEAR or something
  Use timer instead of counter to set wait time for stationkeep. This includes planningtime.
  For debugging, make update to fishsearcheffort to see weighting distribution.
  Some way to omit cells that are deemed unreachable, could remove them if ompl fails repeatedly.
  Run OMPL in separate thread/process to avoid blocking sending Reference messages

  */
  //! @author Nikolai Lauvås
  namespace Search
  {
    namespace FishSearch
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! The path of the Spatialite database containing the electronic navigational charts
        std::string encDBpath;

        //! The path of the Spatialite database to work in (create search, effort and weight grids)
        std::string effortDBpath;

        //! Size of the grid cells used in the effort and weight grids
        unsigned gridSize;

        //! Geometry type of grid used in the effort and weight grids. See ENCGIS::SearchGrid::gridtypes_t for supported values
        unsigned gridType;

        //! The names of otherVehicles participating in the search operation. Used to include effort of these vehicles.
        std::vector<std::string> otherVehicles;

        //! A constant that is subtracted from the effort metric at each timestep to account for the moving targets
        float timestepConstantDecrease;

        //! A factor that is multiplied with the effort metric at each timestep to account for the moving targets
        float timestepFactorDecrease;

        //! The maximum time to wait in order to guarantee that the Fish tag has transmitted at least one transmission.
        unsigned tagMaxTransmissionInterval;

        //! Limiting range of the effort calculation
        unsigned maxConsideredRange;

        //! Once a search cell is reached, the vehicle will wait this many timesteps before starting to move towards the next cell.
        unsigned waitingSteps;

        //! Navigable Layer/table Name from encDBpath
        std::string dbNavigableLayerName;

        //! Innavigable Layer/table Name from encDBpath
        std::string dbInnavigableLayerName;

        //! Variable to enable/disable the use of OMPL path finder between cells
        bool useOMPL;

        //! The maxmum planning time allowed for OMPL planning
        float OMPLmaxPlanningTime;

        //! Acceptable waiting radius when a new location is searched
        float waitingRadius;
      };

      struct Task: public DUNE::Tasks::Periodic
        {
        //! Task arguments.
        Arguments m_args;
        //! Database connection
        std::shared_ptr<ENCGIS::DBconnection> m_con;
        //! The search grid the planner operates on
        std::unique_ptr<ENCGIS::SearchGrid> m_searchGrid;
        //! The grid object used for effort and weight storage
        std::unique_ptr<ENCGIS::SearchGridCoverageState> m_searchGridCoverage;
        //! Planner instance operating on m_searchGrid
        std::unique_ptr<ENCGIS::SearchGridPlanner> m_GridPlanner;
#if SEARCHGRID_USEOPP_OMPL
        //! Point collision check For use in path planner
        std::unique_ptr<ENCGIS::isPointInLayerStatement> pointCheck;
        //! Line segment collision check For use in path planner
        std::unique_ptr<ENCGIS::lineIntersectLayerStatement> lineCheck;
        //! OMPL instance to use for running the path planning on
        std::unique_ptr<og::SimpleSetup> m_OMPLsetup;

        std::unique_ptr<ompl::msg::OutputHandler> m_omplMsgOutputHandler;

        //! Storage for path found by OMPL. Keept empty if the cell can be traveled to without collision
        std::vector<std::pair<double,double>> m_OMPLpath;
#endif

        //! Map with vehicle id as index, and a the location used while updating the effort grid weights
        std::map<uint16_t, std::pair<double,double>> pendingUpdates;
        //! Source identifiers for the monitiored vehicles
        std::vector<uint16_t> monitoredVehicles;
        //! Current rpm of this vehicle used for range calculations (This should be changed, or be marked as a simplification for multi-vehicle operations.)
        unsigned m_rpm;

        //! Parsed size of the grid cells from last received IMC::PlanProbSpec
        unsigned m_gridSize;
        //! Parsed Geometry type of grid from last received IMC::PlanProbSpec.
        unsigned m_gridType;
        //! Parsed planner type from last received IMC::PlanProbSpec
        unsigned m_planner;
        //! Parsed distance weight from last received IMC::PlanProbSpec. Used in planner.
        float m_distanceWeight;
        //! Parsed azimuth weight from last received IMC::PlanProbSpec. Used in planner.
        float m_azimuthWeight;

        bool m_reuseMap;

        //! Last plan control state
        IMC::PlanControlState m_last_plan_state;
        //! Latest estimated state from self
        IMC::EstimatedState m_esta;
        //! The current Reference being sent in Task()
        IMC::Reference m_cur_ref;
        //! The morst recent FollowRefState received
        IMC::FollowRefState m_last_follow_ref;

        //! The current target cell
        int m_currentCell;
        //! Counter used to enable waiting at each cell 
        unsigned m_hooverStartRunCount;
        //! Stores waitingSteps parsed from IMC::PlanProbSpec if given in customParameters, else takes from arguments
        unsigned m_waitingSteps;

        //! Stores OMPLmaxPlanningTime parsed from IMC::PlanProbSpec if given in customParameters, else takes from arguments
        float m_OMPLmaxPlanningTime;

        //! Variable to enable/disable the use of OMPL path finder between cells
        bool m_useOMPL;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Periodic(name, ctx)
        {
          param("Other Vehicles", m_args.otherVehicles)
          .description("The source/vehicle names of other entities in the system.")
          .defaultValue("");

          param("Timestep Grid Constant Decrease", m_args.timestepConstantDecrease)
          .defaultValue("0.0")
          .description("The value substracted from each grid cell at each timestep.");

          param("Timestep Grid Factor Decrease", m_args.timestepFactorDecrease)
          .defaultValue("1.0")
          .description("The value multiplied with each grid cell at each timestep. [0.0, 1.0]");

          param("StationKeep steps", m_args.waitingSteps)
          .defaultValue("14")
          .description("Passive listening time at each cell, calculated by multiplying with task execution time");

          param("StationKeep Radius", m_args.waitingRadius)
          .defaultValue("60")
          .description("Passive listening radius during waiting");

          param("Tag Max Transmission Interval", m_args.tagMaxTransmissionInterval)
          .defaultValue("90")
          .description("The maximum transmission intervall expected for the targeted fish tags/transmitters");

          param("Max Range Considered", m_args.maxConsideredRange)
          .defaultValue("670")
          .description("The maximum range to perform effort updates on.");

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

          param("Navigable Layer Name", m_args.dbNavigableLayerName)
          .defaultValue("navigable")
          .description("Navigable Layer Name");

          param("Innavigable Layer Name", m_args.dbInnavigableLayerName)
          .defaultValue("innavigable")
          .description("Innavigable Layer Name");

          param("Use OMPL", m_args.useOMPL)
          .defaultValue("true")
          .description("Toggle if the OMPL path finder should be used to verify/create safe paths between cells when searching.");

          param("OMPL Max Time", m_args.OMPLmaxPlanningTime)
          .defaultValue("2.0")
          .description("Time cutoff for OMPL planner.");


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
        void onUpdateParameters(void)
        {
          if(paramChanged(m_args.otherVehicles)) {
            monitoredVehicles.clear();
            for(auto iter : m_args.otherVehicles) {
                monitoredVehicles.push_back(resolveSystemName(iter));
            }

            for(auto iter : monitoredVehicles) {
                inf("%u", iter);
            }
          }

          if(paramChanged(m_args.useOMPL)) {
            m_useOMPL = m_args.useOMPL;
          }

          if(paramChanged(m_args.OMPLmaxPlanningTime)) {
            m_OMPLmaxPlanningTime = m_args.OMPLmaxPlanningTime;
          }
        }

        //! Reserve entity identifiers.
        void onEntityReservation(void)
        {
        }

        //! Resolve entity names.
        void onEntityResolution(void)
        {

        }

        //! Acquire resources.
        void onResourceAcquisition(void)
        {
          std::string attachedDb = "db1";
          try{
            m_con = std::make_shared<ENCGIS::DBconnection>(m_args.effortDBpath, SQLITE_OPEN_READWRITE, 32632);
            m_con->runNoOutputQuery("attach '" + m_args.encDBpath + "' as " + attachedDb + "");
            //m_con->runQuery("select * from db1.coalne limit 10");
          } catch(std::runtime_error& e) {
            err(DTR("Problem opening charts database: %s"), e.what());
            // Set task state to failure
          }

          m_searchGrid = std::make_unique<ENCGIS::SearchGrid>(m_con.get(), "FishSearch");
          m_searchGridCoverage = std::make_unique<ENCGIS::SearchGridCoverageState>(m_con.get(), std::string("coverage"));
          
          try{
            pointCheck = std::make_unique<ENCGIS::isPointInLayerStatement>(m_args.dbNavigableLayerName, "geometry", m_con->db, 32632, attachedDb);
          } catch(std::runtime_error& e) {
            err(DTR("Problem creating query for navigable layer: %s"), e.what());
            // Set task state to failure
          }

          try{
            lineCheck = std::make_unique<ENCGIS::lineIntersectLayerStatement>(m_args.dbInnavigableLayerName, "geometry", m_con->db, 32632, attachedDb);
          } catch(std::runtime_error& e) {
            err(DTR("Problem creating query for innavigable layer: %s"), e.what());
            // Set task state to failure
          }  
        }

        //! Initialize resources.
        void onResourceInitialization(void)
        {
          // Set OMPL to use the console output of this task
          m_omplMsgOutputHandler = std::make_unique<OMPLforDUNE::OutputHandlerDUNEConsole>(this);
          ompl::msg::useOutputHandler(m_omplMsgOutputHandler.get());
          ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
        }

        //! Release resources.
        void onResourceRelease(void) {
            inf("Release");
            //if(m_searchGridCoverage)
            //  m_searchGridCoverage->deleteGrid();
            //if(m_searchGrid)
            //  m_searchGrid->deleteGrid();
            try {
            m_con.reset();
            m_searchGrid.reset();
            m_searchGridCoverage.reset();
            m_OMPLsetup.reset();
            }
            catch(std::runtime_error& e) {
            err(DTR("Could not clear charts database class: %s"), e.what());
            }
        }

        void onActivation(void)
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

        void onDeactivation(void)
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
        
        void consume(const IMC::FollowRefState * msg) {
          if(isActive()) {
            if(msg->control_ent == getEntityId()) {
              m_last_follow_ref = *msg;
            }
          }
        }

        void consume(const IMC::VehicleState * msg)
        {
          // if the vehicle is in error mode, set FishSearch task to inactive
          if (msg->op_mode == IMC::VehicleState::VS_ERROR)
            requestDeactivation();
        }

        void consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;
          
          war(DTR("Abort detected. Disabling FishSearch control."));
          requestDeactivation();
        }

        void consume(const IMC::PlanControlState * msg)
        {
          m_last_plan_state = *msg;

          //m_fishSearch_control = msg->state == IMC::PlanControlState::PCS_EXECUTING
          //&& msg->plan_id == "fishSearch_plan";
        }

        void consume(const IMC::PlanControl* msg)
        {
          if (msg->type == PlanControl::PC_REQUEST && msg->op == PlanControl::PC_STOP)
          {
            if (m_last_plan_state.plan_id == "fishSearch_plan" &&
                m_last_plan_state.state == IMC::PlanControlState::PCS_EXECUTING)
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
            m_rpm = std::abs(msg->value);
        }

        std::string polygonToEWKT(const IMC::MessageList<IMC::PolygonVertex> &polygon) {
          std::string EWKT = "SRID=4326;POLYGON((";
          for(const auto itr : polygon) {
              EWKT += std::to_string(DUNE::Math::Angles::degrees(itr->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees(itr->lat)) + ",";
          }
          EWKT += std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lon)) + " " + std::to_string(DUNE::Math::Angles::degrees((*(polygon.begin()))->lat)) + "))";
          return EWKT;
        }

        void consume(const IMC::PlanProbSpec* msg)
        {
          spew("Message received");
          spew("Destination: %i", msg->getDestination());
          spew("Problem Type%i", msg->problem_type);

          // Only accept messages to this system
          if (msg->getDestination() != getSystemId())
            return;

          // Only proceed for coverage problems
          if (msg->problem_type != IMC::PlanProbSpec::TypeEnum::PPT_coverage)
            return;

          spew("Starting processing");
          // Parse Custom Parameters
          /*
          Supported custom parameters:
              gg = Grid geometry
              gs = Grid geometry edge size
              p = Planner
              paw = Planner azimuth weight
              pdw = Planner distance weight
          To be Implemented: 
              sk = Station keep steps
              r = reuse previous map
              t = [0.0,inf), Max planning time on OMPL
          */

          
          DUNE::Utils::TupleList custom = DUNE::Utils::TupleList(msg->custom);
          std::map<std::string, std::string> custommap = custom.getMapReversed();

          //auto parameterit = custommap.find(std::string("a"));

 //gg = Grid geometry
          auto parameterit = custommap.find(std::string("gg"));
          if (parameterit != custommap.end()) {
            spew("Found gg=%s", parameterit->second.c_str());
            try{
              m_gridType = std::stoul(parameterit->second);
            } catch(...) {
              err("Parameter \'gg\' not unsigned");
            }
          } else {
            m_gridType = m_args.gridType;
          }
// gs = Grid geometry edge size
          parameterit = custommap.find(std::string("gs"));
          if (parameterit != custommap.end()) {
            spew("Found gs=%s", parameterit->second.c_str());
            try{
              m_gridSize = std::stof(parameterit->second);
            } catch(...) {
              err("Parameter \'gs\' not float");
            }
          } else {
            m_gridSize = m_args.gridSize;
          }
// p = Planner
          parameterit = custommap.find(std::string("p"));
          if (parameterit != custommap.end()) {
            try{
              m_planner = std::stoi(parameterit->second);
              spew("Found p=%d", m_planner);
            } catch(...) {
              err("Parameter \'p\' not unsigned");
            }
          } else {
            m_planner = 0;
          }
// paw = Planner azimuth weight
          parameterit = custommap.find(std::string("paw"));
          if (parameterit != custommap.end()) {
            spew("Found paw=%s", parameterit->second.c_str());
            try{
              m_azimuthWeight = std::stof(parameterit->second);
            } catch(...) {
              err("Parameter \'paw\' not unsigned");
            }
          } else {
            m_azimuthWeight = 0.0;
          }
// pdw = Planner distance weight
          parameterit = custommap.find(std::string("pdw"));
          if (parameterit != custommap.end()) {
            spew("Found pdw=%s", parameterit->second.c_str());
            try{
              m_distanceWeight = std::stof(parameterit->second);
            } catch(...) {
              err("Parameter \'pdw\' not unsigned");
            }
          } else {
            m_distanceWeight = 0.0;
          }
// sk = Station keep steps
          parameterit = custommap.find(std::string("sk"));
          if (parameterit != custommap.end()) {
            spew("Found sk=%s", parameterit->second.c_str());
            try{
              m_waitingSteps = std::stoi(parameterit->second);
            } catch(...) {
              err("Parameter \'sk\' not unsigned");
            }
          } else {
            m_waitingSteps = m_args.waitingSteps;
          }
// r = reuse previous map
          parameterit = custommap.find(std::string("r"));
          if (parameterit != custommap.end()) {
            spew("Found r=%s", parameterit->second.c_str());
            try{
              m_reuseMap = (std::stoi(parameterit->second)) ? true : false;
            } catch(...) {
              err("Parameter \'r\' not unsigned");
            }
          } else {
            m_reuseMap = false;
          }
// t = [0.0,inf), Max planning time on OMPL
          parameterit = custommap.find(std::string("t"));
          if (parameterit != custommap.end()) {
            spew("Found t=%s", parameterit->second.c_str());
            try{
              m_OMPLmaxPlanningTime = std::stof(parameterit->second);
            } catch(...) {
              err("Parameter \'t\' not unsigned");
            }
          } else {
            m_OMPLmaxPlanningTime = m_args.OMPLmaxPlanningTime;
          }
// o = Use OMPL or go direct to next cell
          parameterit = custommap.find(std::string("o"));
          if (parameterit != custommap.end()) {
            spew("Found o=%s", parameterit->second.c_str());
            try{
              m_useOMPL = (std::stoi(parameterit->second)) ? true : false;
            } catch(...) {
              err("Parameter \'o\' not unsigned");
            }
          } else {
            m_useOMPL = m_args.useOMPL;
          }



          double planningBounds[4];
          // Convert initial position from WGS-84 radians to EPSG32632
          double start_northing, start_easting;
          m_con->transformSRID(Math::Angles::degrees(msg->start_lon), Math::Angles::degrees(msg->start_lat), 4326, start_easting, start_northing, 32632);
          spew("Planning start: %f, %f", start_easting, start_northing);

          if(m_reuseMap){
            if(m_con->checkSpatialIndex("coverage", "geometry")) {
              inf("Found previous map, resuming.");
            } else {
              err("Previous map not found or not correct, search will not start.");
            }
          } else {
            // Create new effort/weight and fishsearch maps

            spew("Creating Search Map, checking input polygon.");
            if(msg->area.size() == 2) {
                m_searchGrid->deleteGrid();
                m_searchGridCoverage->deleteGrid();
                // Create planning bound
                //IMC::MessageList<IMC::PolygonVertex>::const_iterator itr = msg->area.begin();
                //for (unsigned i = 0; itr != msg->area.end(); ++itr, ++i)
                //{
                //    spew("lat %f, lon %f", (*itr)->lat, (*itr)->lon);
                //}
                auto itr = msg->area.begin();
                m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[0], planningBounds[1], 32632);
                ++itr;
                spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[2], planningBounds[1], planningBounds[3]);
                m_con->transformSRID(Math::Angles::degrees((*itr)->lon), Math::Angles::degrees((*itr)->lat), 4326, planningBounds[2], planningBounds[3], 32632);
                m_searchGrid->createGrid(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType), true);
                m_searchGridCoverage->createGrid(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType), true);
            } else if(msg->area.size() > 2) {
                m_searchGrid->deleteGrid();
                m_searchGridCoverage->deleteGrid();
                std::string EWKT = polygonToEWKT(msg->area);
                m_searchGrid->createGrid(EWKT, m_gridSize, ENCGIS::SearchGrid::gridtypes_t(m_gridType), true);
                debug("Search Grid Created from EKWT");
                m_searchGridCoverage->createGrid(EWKT, m_args.gridSize, ENCGIS::SearchGrid::gridtypes_t(m_args.gridType), true);
                debug("Coverage Grid Created from EKWT");
                m_searchGridCoverage->updateDetectionProbability("fishsearch");
                debug("Init updateDetectionProbability");
                
                spew("Weights of grid set");
            } else {
                spew("Polygon too small.");
                return;
            }

          // Create prior distribution from land distance
          m_searchGridCoverage->setGridMetricFromLandDistance();
          m_searchGridCoverage->normalizeMetric(true);
          m_searchGridCoverage->makeMetricSumToOne("weight");
          }



#if SEARCHGRID_USEOPP_OMPL
          // Find square covering bounds of search area
          
          m_con->getExtent("fishsearchraw", planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3]);
          spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3]);
          if(m_OMPLsetup) {
            m_OMPLsetup.reset();
          }
          spew("OMPL clear sucess");
          m_OMPLsetup = std::make_unique<og::SimpleSetup>(OMPLintegrationENCGIS::createSetup(planningBounds[1], planningBounds[0], planningBounds[3], planningBounds[2], pointCheck.get(), lineCheck.get()));
          spew("OMPL init sucess 1");
#endif
 


          m_currentCell = m_searchGrid->getClosestCell(start_easting, start_northing);

          m_GridPlanner = std::make_unique<ENCGIS::SearchGridPlanner>(m_searchGrid.get());
          // Find coverage path
          m_GridPlanner->setinitialCell(m_currentCell);
          m_GridPlanner->setinitialAzimuth(m_esta.psi);
          m_GridPlanner->setdistributionWeight(1);
          m_GridPlanner->setazimuthWeight(m_azimuthWeight);
          m_GridPlanner->setdistanceWeight(m_distanceWeight);
          //m_GridPlanner->setmaxPlaningTime(2.0);


          // Cumulative search effort tracking setup
          pendingUpdates.clear();
          if(!isActive()) {
            requestActivation();
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }
          m_hooverStartRunCount = 0;
          auto initial_pos = m_searchGrid->getCellLocation(m_currentCell);

          m_cur_ref.lon = DUNE::Math::Angles::radians(initial_pos.first);
          m_cur_ref.lat = DUNE::Math::Angles::radians(initial_pos.second);
          DUNE::IMC::DesiredSpeed m_dsp;
          //DUNE::IMC::DesiredZ m_dz;
          m_dsp.value = msg->speed;
          m_dsp.speed_units = IMC::SUNITS_METERS_PS;
          m_cur_ref.speed.set(m_dsp);
          //m_dz.value = 0.0;
          //m_dz.z_units = IMC::Z_ALTITUDE;
          //m_cur_ref.z.set(m_dz);
          m_cur_ref.flags = IMC::Reference::FlagsBits::FLAG_LOCATION | IMC::Reference::FlagsBits::FLAG_SPEED;
          dispatch(m_cur_ref);
        }

        int calculateNextCell(int lastVisitedCell) {
          m_searchGridCoverage->updateDetectionProbability("fishsearch");
          spew("makeMetricSumToOne");
          m_searchGridCoverage->makeMetricSumToOne("fishsearch");
        int nextCell = 0;
        switch (m_planner) {
          case 0:
              spew("Using getLocalOptimalNeighbour");
                
                nextCell = m_GridPlanner->getLocalOptimalNeighbour(lastVisitedCell);
                if(nextCell == 0) {
                  nextCell = m_searchGrid->getClosestUnsearchedCell(lastVisitedCell);
                  spew("Trying getClosestUnsearchedCell instead");
                }
              break;
          case 1:
            {
              spew("Using calculateSearchPathAzimuth");
                double currentAzimuth = m_esta.psi;
                nextCell = m_GridPlanner->getLocalOptimalNeighbourAzimuth(lastVisitedCell, currentAzimuth);
                if(nextCell == 0) {
                  nextCell = m_searchGrid->getClosestUnsearchedCell(lastVisitedCell);
                  spew("Trying getClosestUnsearchedCell instead");
                }
            }
              break;
          case 2:
              spew("Using getDistanceOptimalNextCell");
                nextCell = m_GridPlanner->getDistanceOptimalNextCell(lastVisitedCell);
                if(nextCell == 0) {
                  nextCell = m_searchGrid->getClosestUnsearchedCell(lastVisitedCell);
                  spew("Trying getClosestUnsearchedCell instead");
                }
              break;
          case 3:
              break;
          case 4:
              // code
              break;
          case 5:
              // code
              break;
          case 6:
              // code
              break; 
          case 7:
              spew("Using getGlobalOptimalCell");
              nextCell = m_GridPlanner->getGlobalOptimalCell(m_currentCell, 0.0);
              break;                  
          default:
              break;
        }

#if SEARCHGRID_USEOPP_OMPL
        if(m_useOMPL && nextCell != 0) {
          // Start path from current location
          std::pair<double,double> start;
          m_con->transformSRID(Math::Angles::degrees(m_esta.lon), Math::Angles::degrees(m_esta.lat), 4326, start.first, start.second, 32632);
          // End path in next cell
          auto end = m_searchGrid->getCellLocation(nextCell,m_searchGrid->getSRID());

          OMPLintegrationENCGIS::setStartAndGoalStates(*m_OMPLsetup, start.first, start.second, end.first, end.second);
          og::PathGeometric states = OMPLintegrationENCGIS::findPath(*m_OMPLsetup, m_OMPLmaxPlanningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
          if (states.getStateCount()) {
              m_OMPLpath = OMPLforDUNE::pathToVector(states);
              m_OMPLpath = m_con->transformSRIDVector(m_OMPLpath, 32632,4326);
              std::reverse(m_OMPLpath.begin(), m_OMPLpath.end()); // Reverse so that pop back will give the most recent post
              m_OMPLpath.pop_back(); // Remove first waypoint (Current position)
              inf("Path Vector: ");
              for(auto iter : m_OMPLpath) {
                inf("%f, %f", iter.first, iter.second);
              }
          } else {
              err("Error finding path from: %f, %f to %f ,%f", start.first, start.second, end.first, end.second);
          }

        // Compute path with OMPL
        // Add to global std::vector<std::pair<double,double>>
        // In main, IMC::FollowRefState::FR_HOVER, add if the vector is not empty, pop top and set ref to that value.
        }
#endif
        spew("end calcnext");
        return nextCell;
      }

        void updateGrid() {
            // Update cummulative effort
            if(m_args.timestepFactorDecrease != 1.0) {
              m_searchGridCoverage->decreaseAll(m_args.timestepConstantDecrease, m_args.timestepFactorDecrease, std::string("effort"));
            } else if(m_args.timestepConstantDecrease > 0.0) {
              m_searchGridCoverage->decreaseAll(m_args.timestepConstantDecrease, std::string("effort"));
            }
            
            for(auto iter : pendingUpdates) {
              // Convert from WGS-84 to EPSG32632
              double northing, easting;
              m_con->transformSRID(Math::Angles::degrees(iter.second.first), Math::Angles::degrees(iter.second.second), 4326, easting, northing, 32632);
              m_searchGridCoverage->updateLogarithmic(easting, northing, m_rpm, (1/getFrequency())/m_args.tagMaxTransmissionInterval, m_args.maxConsideredRange, 0.05, std::string("effort"));
              inf("%f %f", easting, northing);
            }
            pendingUpdates.clear();
        }
        //! Main loop.
        void task(void)
        {
          consumeMessages();
          if(isActive()) {
            updateGrid();
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
                // Planner update
                spew("Frefstate: %u", m_last_follow_ref.state);
                spew("LastCell: %d, hooverstart: %d", m_currentCell, m_hooverStartRunCount);
                war("Initial pos: %f, %f", m_cur_ref.lon, m_cur_ref.lat);
                if(m_hooverStartRunCount) {
                  m_hooverStartRunCount--;
                } else {
                  if(!m_OMPLpath.empty()) {
                    // Followin previously found path 
                    std::pair<double, double> omplpathpos = m_OMPLpath.back();
                    m_cur_ref.lon = DUNE::Math::Angles::radians(omplpathpos.first);
                    m_cur_ref.lat = DUNE::Math::Angles::radians(omplpathpos.second);
                    spew("Using m_OMPLpath");
                    m_OMPLpath.pop_back();
                  } else {
                    m_currentCell = calculateNextCell(m_currentCell);
                    if(!m_currentCell) { // If zero is the last visited cell, the search is finished
                      spew("Zero cell received, ending planner");
                      requestDeactivation();
                      return;
                    }
                    m_hooverStartRunCount = m_waitingSteps;
                  }
                } 
              break;
              //! Moving in z after arriving at the target cylinder. (Not used)
              case IMC::FollowRefState::FR_ELEVATOR:
              break;
              //! Controlling system timed out. Only occurs when this task is not active, so this should never happen.
              case IMC::FollowRefState::FR_TIMEOUT:
                err("Got FollowRefState::FR_TIMEOUT");
                requestDeactivation();
              break;
            }
            dispatch(m_cur_ref);
            //inf("Time Update");
          }
        }
      };
    }
  }
}
DUNE_TASK

