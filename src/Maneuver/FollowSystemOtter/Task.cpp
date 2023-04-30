//***************************************************************************
// Copyright 2013-2023 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is an extension to the DUNE: Unified Navigation Environment.   *
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
// Author: Nikolai Lauvås (Based on the task created by Pedro Calado)
/* Main changes:
Complete:
Fixed orientation following for announce
Adaptive speed control
Sjekk end condition og legg til taskStopp/abort/stop
Bare send DesiredSpeed når desiredPathz har blitt sendt
Closest safe spot generator
Safe path generation
Add collision avoidance with selected IMC repporting vehicles

TODO:
Bedre replanning, f.eks hver gang en får announce
Sjekk replanning i OMPL for å optimere. 
Todo: Hvem skal forstsette i collision avoidance?

*/
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ENCGIS
#include <ENCGIS/isPointInLayerStatement.hpp>
#include <ENCGIS/lineIntersectLayerStatement.hpp>
// OMPL integration for DUNE
#include <OMPL/setup.hpp>
#include <OMPL/OMPLfunctions.hpp>

namespace Maneuver
{
  namespace FollowSystemOtter
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! The path of the Spatialite database containing the electronic navigational charts
      std::string encDBpath;
      //! The maxmum planning time allowed for OMPL planning
      float OMPLmaxPlanningTime;
      //! Navigable Layer/table Name from encDBpath
      std::string dbNavigableLayerName;
      //! Innavigable Layer/table Name from encDBpath
      std::string dbInnavigableLayerName;

      //! The names of other IMC vehicles to avoid while performing this maneuver.
      std::vector<std::string> otherVehicles;

      //! PID gains for mps controller.
      std::vector<float> mps_pid_gains;
      double mps_pid_min;
      double mps_pid_max;
      double mps_pid_max_int;
      //! Minimum time to wait between speed updates
      double speed_update_ts_min;

      double loiter_radius;
      double timeout;
      bool announce_active;
      bool remote_active;
      double min_displace;
      double heading_cooldown;
      double safe_distance;
      bool use_orientation;
      bool use_speed_PID;
      bool use_ompl;
    };

    struct Task: public DUNE::Maneuvers::Maneuver
    {

      //! Database connection
      std::shared_ptr<ENCGIS::DBconnection> m_con;
      //! Point collision check For use in path planner
      std::unique_ptr<ENCGIS::isPointInLayerStatement> pointCheck;
      //! Line segment collision check For use in path planner
      std::unique_ptr<ENCGIS::lineIntersectLayerStatement> lineCheck;
      //! OMPL instance to use for running the path planning on
      std::unique_ptr<og::SimpleSetup> m_OMPLsetup;
      //! Object that lets OMPL write to the DUNE prompt
      std::unique_ptr<ompl::msg::OutputHandler> m_omplMsgOutputHandler;

      //! Speed PID taking distance from desired position and returns desired speed in mps
      DiscretePID m_mps_pid; // TODO: Change to USER::PID
      //! Time of last estimated state message used for speed controller.
      Delta m_replanDelta;
      //! Time of last estimated state message used for speed controller.
      Delta m_delta;
      //! Variable to save the maneuver's data
      IMC::FollowSystem m_maneuver;
      //! Vehicle's Estimated State
      IMC::EstimatedState m_estate;
      //! Desired path to be thrown
      IMC::DesiredPath m_path;
      //! Remote State computed heading's timestamp, for evaluating the best heading to be used
      Counter<double> m_heading_timestamp;
      //! this variable will hold the value of the heading computed when using the announce method instead of the remote state.
      double m_remote_heading;
      //! the start time of the maneuver measured at consume maneuver
      double m_start_time;
      //! the last Clock::get() when the neighbor system's position was updated
      Counter<double> m_last_update;
      //! variable to hold the last known bearing
      double m_last_known_bearing;
      //! variable that will hold the last known latittude
      double m_last_known_lat;
      //! variable that will hold the last known longitude
      double m_last_known_lon;
      //! is it the first time consume announce is being ran?
      bool m_first_announce;
      //! this boolean tells us if we have an estimated state already
      bool m_has_estimated_state;
      //! The desired speed the vehicle should hold
      float m_desired_speed;
      //! The m_last_known_lat offset according to the m_maneuver
      double m_offset_target_lat;
      //! The m_last_known_lon offset according to the m_maneuver
      double m_offset_target_lon;
      //! Set if a PlanControlState message has been received
      bool m_has_pcs;

      bool distanceLimitBreached;
      //! Source identifiers for the monitiored vehicles
      std::map<uint16_t, std::tuple<fp64_t, fp64_t, DUNE::Time::Delta>> monitoredVehicles;

      //! the last Clock::get() when the pcs was received
      Counter<double> m_last_pcs;
      bool m_target_moved;
      //! Path to the desired position calculated from the followed system
      std::vector<std::pair<double,double>> m_path_to_target;
      //! Task Arguments
      Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx),
        m_last_known_bearing(0.0),
        m_last_known_lat(0.0),
        m_last_known_lon(0.0),
        m_first_announce(true),
        m_has_estimated_state(false),
        m_desired_speed(0.0), 
        m_has_pcs(false),
        distanceLimitBreached(false)
      {
        param("Loitering Radius", m_args.loiter_radius)
        .defaultValue("-1.0")
        .units(Units::Meter)
        .description("Radius of the loiter when waiting for new waypoint");

        param("Timeout", m_args.timeout)
        .defaultValue("60.0")
        .units(Units::Second)
        .description("Maneuver timeout");

        param("Offset Follow Orientation", m_args.use_orientation)
        .defaultValue("false")
        .description("If the offset follows the orientation of the followed system.");

        param("Use Speed Controller", m_args.use_speed_PID)
        .defaultValue("False")
        .description("Use PID control to controll speed according to distance from setpoint.");

        param("MPS PID Gains", m_args.mps_pid_gains)
        .defaultValue("0.05, 0.0, 0.0")
        .size(3)
        .description("PID gains for MPS controller");

        param("MPS PID Lower Limit", m_args.mps_pid_min)
        .defaultValue("0.4")
        .description("Use PID control to controll speed according to distance from setpoint.");

        param("MPS PID Upper Limit", m_args.mps_pid_max)
        .defaultValue("2.2")
        .description("Use PID control to controll speed according to distance from setpoint.");

        param("MPS PID Integral Limit", m_args.mps_pid_max_int)
        .defaultValue("1")
        .description("Use PID control to controll speed accordingt o distance from setpoint.");

        param("Use OMPL", m_args.use_ompl)
        .defaultValue("False")
        .description("Use OMPL to plan safe ways towards desired position.");

        param("ENC DB Path", m_args.encDBpath)
        .defaultValue("/home/nikolai/lststools/dune/misc/re4utmfinal.sqlite")
        .description("The path of the DB to read ENC from.");

        param("Navigable Layer Name", m_args.dbNavigableLayerName)
        .defaultValue("navigable")
        .description("Navigable Layer Name");

        param("Innavigable Layer Name", m_args.dbInnavigableLayerName)
        .defaultValue("innavigable")
        .description("Innavigable Layer Name");

        param("OMPL Max Time", m_args.OMPLmaxPlanningTime)
        .defaultValue("1.0")
        .description("Time cutoff for OMPL planner.");

        param("Using RemoteState", m_args.remote_active)
        .defaultValue("false")
        .description("Using remote state for tracking system");

        param("Using Announce", m_args.announce_active)
        .defaultValue("false")
        .description("Using announce to track system");

        param("Min Displace", m_args.min_displace)
        .defaultValue("2.0")
        .units(Units::Meter)
        .description("Minimum target displacement for computing new heading");

        param("Heading Cooldown", m_args.heading_cooldown)
        .defaultValue("15.0")
        .units(Units::Second)
        .description("");

        param("Minimum Safe Distance", m_args.safe_distance)
        .defaultValue("15.0")
        .units(Units::Meter)
        .description("Minimum safe distance to target system");

        param("Minimum Speed Update Interval", m_args.speed_update_ts_min)
        .defaultValue("5.0")
        .units(Units::Second)
        .description("Minimum time to wait between speed updates");

        param("Other Vehicles", m_args.otherVehicles)
        .description("The source/vehicle names of other entities in the system.")
        .defaultValue("ntnu-otter-03");

        bindToManeuver<Task, IMC::FollowSystem>();
        //bind<IMC::RemoteState>(this);
        bind<IMC::EstimatedState>(this); // Not needed so disabled(consume even if inactive)
        bind<IMC::Announce>(this);
      }

      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.timeout))
          m_last_update.setTop(m_args.timeout);

        if (paramChanged(m_args.heading_cooldown))
          m_heading_timestamp.setTop(m_args.heading_cooldown);

        if (paramChanged(m_args.use_speed_PID)) {
          m_mps_pid.reset();
          m_delta.reset();
        }

        if((paramChanged(m_args.mps_pid_min)) || (paramChanged(m_args.mps_pid_max))) {
          m_mps_pid.setOutputLimits(m_args.mps_pid_min, m_args.mps_pid_max);
        }

        if (paramChanged(m_args.mps_pid_gains)) {
          m_mps_pid.setGains(m_args.mps_pid_gains);
        }

        if (paramChanged(m_args.mps_pid_max_int)) {
          m_mps_pid.setIntegralLimits(m_args.mps_pid_max_int);
        }

        if(paramChanged(m_args.otherVehicles)) {
          monitoredVehicles.clear();
          for(auto iter : m_args.otherVehicles) {
              monitoredVehicles[resolveSystemName(iter)] = std::tuple<fp64_t, fp64_t, DUNE::Time::Delta>{0.0,0.0,DUNE::Time::Delta()};
          }
        }

        if(paramChanged(m_args.loiter_radius)) {
          m_path.lradius = m_args.loiter_radius;
        }

        m_last_pcs.setTop(3); // TODO: Parameter
      }

      //! Acquire resources.
      void onResourceAcquisition(void)
      {
        try{
          m_con = std::make_shared<ENCGIS::DBconnection>(m_args.encDBpath, SQLITE_OPEN_READWRITE, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem opening charts database: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }

        try{
          pointCheck = std::make_unique<ENCGIS::isPointInLayerStatement>(m_args.dbNavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for navigable layer: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }

        try{
          lineCheck = std::make_unique<ENCGIS::lineIntersectLayerStatement>(m_args.dbInnavigableLayerName, "geometry", m_con->db, 32632);
        } catch(std::runtime_error& e) {
          err(DTR("Problem creating query for innavigable layer: %s"), e.what());
          setEntityState(IMC::EntityState::ESTA_FAULT, Status::CODE_MISSING_DATA);
        }  
      }

      //! Initialize resources.
      void onResourceInitialization(void)
      {
        // Call the default maneuver initialization
        DUNE::Maneuvers::Maneuver::onResourceInitialization();
        // Set OMPL to use the console output of this task
        m_omplMsgOutputHandler = std::make_unique<OMPLforDUNE::OutputHandlerDUNEConsole>(this);
        ompl::msg::useOutputHandler(m_omplMsgOutputHandler.get());
        ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);

        m_path.flags = IMC::DesiredPath::FL_NO_Z;
      }

      //! Release resources.
      void onResourceRelease(void) {
        inf("Release");
        try {
          m_con.reset();
          m_OMPLsetup.reset();
        }
          catch(std::runtime_error& e) {
          err(DTR("Could not clear charts database class: %s"), e.what());
        }
      }

      void
      onManeuverDeactivation(void)
      {
        m_first_announce = true;
        m_has_estimated_state = false;
        m_path_to_target.clear();
        m_has_pcs = false;
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        //inf("Message from %x, followed system: %x", msg->getSource(), m_maneuver.system);
        if (msg->getSource() == getSystemId()) {
          // do not do a thing if the announce method is not active
          if (!m_args.announce_active)
            return;

          m_estate = *msg;
          m_has_estimated_state = true;

          if(checkDistanceToMonitoredVehicles()) {
            spew("Distance Limit Violated, disabling movement");
            enableMovement(false);
            m_path_to_target.clear();
            distanceLimitBreached = true;
            return;
          } else {
            distanceLimitBreached = false;
          }
          // Speed PID update
          if(!m_last_pcs.overflow() && m_has_pcs) { // Only update speed pid when plan control is responsive
            double tstep = m_delta.check();
            if(m_args.speed_update_ts_min < tstep) {
              double x,y;
              WGS84::displacement(msg->lat, msg->lon, 0.0, m_offset_target_lat, m_offset_target_lon, 0.0, &x, &y);
              double error = Math::norm((x - m_estate.x), (y - m_estate.y));
              m_path.speed = m_mps_pid.step(tstep, error);
              dispatch(m_path);
              m_delta.reset();
            }
          }
        } else if (msg->getSource() == m_maneuver.system) {
          // Received EstimatedState of followed system
          inf("Received EstimatedState from followed system: Psi: %f", msg->psi);
        }
      }

      void
      consume(const IMC::FollowSystem* maneuver)
      {
        enableMovement(false);

        m_maneuver = *maneuver;
        m_heading_timestamp.reset();
        m_start_time = Clock::get();
        // Initialize the variable last update to the beggining of the maneuver
        m_last_update.reset();

        m_desired_speed = m_maneuver.speed;

        debug("loitering radius is %0.2f meters", m_args.loiter_radius);
        debug("offsets are %0.2f %0.2f %0.2f", m_maneuver.x, m_maneuver.y, m_maneuver.z);
        debug("speed is %0.2f units %d", m_maneuver.speed, (int)m_maneuver.speed_units);
      }

      void
      consume(const IMC::Announce* msg)
      {
        // Not the vehicle we are following
        if (msg->getSource() != m_maneuver.system) {
          if(msg->getSource() != getSystemId()) {
            if(!monitoredVehicles.empty()) {
              auto current = monitoredVehicles.find(msg->getSource());
              if( current != monitoredVehicles.end()) {
                // Add position to list, then check for collisions in EstimatedState
                std::get<0>(current->second) = msg->lat;
                std::get<1>(current->second) = msg->lon; 
                std::get<2>(current->second).reset(); 
              }
            }
          }
          return;
        }
        // The announce method is inactive
        if(!m_args.announce_active) {
          return;
        }

        // update the variable last update
        m_last_update.reset();

        // Check distance to target
        if (!checkSafety(msg->lat, msg->lon))
        {
          // leave this consume function but first update "last" variables
          m_last_known_lat = msg->lat;
          m_last_known_lon = msg->lon;
          spew("Checksafety returned false, disabling movement");
          enableMovement(false);
          m_path_to_target.clear();
          return;
        }

        // compute the bearing with the announced data using previous data
        double announced_bearing;
        double announced_displace = 0;

        if (!m_first_announce)
        {
          WGS84::getNEBearingAndRange(m_last_known_lat, m_last_known_lon, msg->lat, msg->lon, &announced_bearing, &announced_displace);

          // if the announcing system has not moved much, use the previously computed bearing
          if (announced_displace < m_args.min_displace) {
            announced_bearing = m_last_known_bearing;
            m_target_moved = false;
          } else {
            m_target_moved = true;
          }
            

          // check if this bearing should be given more emphasis than the one computed using remote state
          if (!m_heading_timestamp.overflow())
            announced_bearing = m_remote_heading;

          // change the offset_target position according to the offsets in m_maneuver
          m_offset_target_lat = msg->lat;
          m_offset_target_lon = msg->lon;

          computeNEDOffsets(m_offset_target_lat, m_offset_target_lon, 0.0, announced_bearing);

          m_last_known_bearing = announced_bearing;
        }
        else // it is the first time announce is running
        {
          // compute lat and lon of the desired path
          //m_offset_target_lat = msg->lat;
          //m_offset_target_lon = msg->lon;
          //computeNEDOffsets(m_offset_target_lat, m_offset_target_lon, 0.0, 0.0);
          m_first_announce = false;
          debug("Waiting for first announce");
          return;
        }

        if(IMC::SUNITS_METERS_PS == m_maneuver.speed_units && m_args.use_speed_PID) {
          m_path.speed = m_desired_speed;
          m_path.speed_units = IMC::SUNITS_METERS_PS;
        } else {
          m_path.speed = m_maneuver.speed;
          m_path.speed_units = m_maneuver.speed_units;
        }

        // update "last" variables
        m_last_known_lat = msg->lat;
        m_last_known_lon = msg->lon;
          if(distanceLimitBreached) {
            return;
          }
        // Only enable movement when distance to offset target pos is sufficiently large TODO: Parameter
        if(DUNE::Coordinates::WGS84::distance(m_estate.lat, m_estate.lon,0.0, m_offset_target_lat, m_offset_target_lon,0.0) > 10) {
          enableMovement(true);

          if(m_args.use_ompl) { // TODO: Check straight line, and only start OMPL if collision detected
            m_path.flags |= IMC::DesiredPath::FL_START;
            if(m_target_moved && !m_path_to_target.empty()) {
              m_path_to_target.clear();
              inf("Target moved, recalculating path");
            }
            if(m_path_to_target.empty()) {
              //////////////////////////////////////////////////// Find end point of planner
              std::pair<double,double> utmend;
              m_con->transformSRID(Math::Angles::degrees(m_offset_target_lon), Math::Angles::degrees(m_offset_target_lat), 4326, utmend.first, utmend.second, 32632);

                if(m_con->findClosestSafePointUTM(utmend.first, utmend.second)) {
                  inf("Original Pos: %f, %f", m_offset_target_lat, m_offset_target_lon);
                  inf("Safe Pos: %f, %f", utmend.first, utmend.second);
                  spew("End Pointinlayer: %d", pointCheck->run(utmend.first, utmend.second));
                } else {
                  err("findClosestSafePoint failed, probably DB error.");
                  return;
                }
              ////////////////////////////////////////////////////
              std::pair<double,double> utmstart;
              m_con->transformSRID(Math::Angles::degrees(m_estate.lon), Math::Angles::degrees(m_estate.lat), 4326, utmstart.first, utmstart.second, 32632);
              if(!pointCheck->run(utmstart.first, utmstart.second)) {
                war("Startpoint collison");
                if(m_con->findClosestSafePointUTM(utmstart.first, utmstart.second)) {
                  inf("Original Pos: %f, %f", m_offset_target_lat, m_offset_target_lon);
                  inf("Safe Pos: %f, %f", utmstart.first, utmstart.second);
                  spew("Start Pointinlayer: %d", pointCheck->run(utmstart.first, utmstart.second));
                } else {
                  err("findClosestSafePoint failed, probably DB error.");
                  return;
                }
              }


              
              if(runOMPLUTM(utmstart.first, utmstart.second, utmend.first, utmend.second)) {
                  //m_path_to_target.pop_back(); // Remove first waypoint (Current position)
                  m_path.start_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
                  m_path.start_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
                  m_path_to_target.pop_back(); // Remove first waypoint (Current position)
                  m_path.end_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
                  m_path.end_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
                  dispatch(m_path);
              } else {
                m_path.end_lat = m_estate.lat;
                m_path.end_lon = m_estate.lon;
                war("OMPL failed, doing nothing");
              }
            } 
          } else {
            m_path.flags |= IMC::DesiredPath::FL_DIRECT;
            inf("Going direct (announce)");
            m_path.end_lat = m_offset_target_lat;
            m_path.end_lon = m_offset_target_lon;
            dispatch(m_path);
          }
        } else {
          spew("Close to desired position, doing nothing");
        }
        //trace("system being pursued has heading: %0.2f and was displaced %0.2f", m_last_known_bearing, announced_displace);
        //trace("this is %0.4f seconds past the maneuver's initial time", Clock::get() - m_start_time);
        //trace("announce data: lat %0.5f, lon %0.5f, %0.2f, %0.4f", msg->lat, msg->lon, msg->height, msg->getTimeStamp());
        //double offx, offy;
        //WGS84::displacement(msg->lat, msg->lon, 0.0, m_path.end_lat, m_path.end_lon, 0.0, &offx, &offy);
        //trace("offset: x %0.2f, %0.2f, %0.2f", offx, offy, m_estate.z);
      }

      //! Updates m_path_to_target with an OMPL generated path from start to end
/*
      bool runOMPL(double startLat, double startLon, double endLat, double endLon) {
        // Convert EPSG4326 radians to EPSG32632 meters
        std::pair<double,double> start, end;
        m_con->transformSRID(Math::Angles::degrees(startLon), Math::Angles::degrees(startLat), 4326, start.first, start.second, 32632);
        m_con->transformSRID(Math::Angles::degrees(endLon), Math::Angles::degrees(endLat), 4326, end.first, end.second, 32632);
        return runOMPLUTM(start.first, start.second, end.first, end.second);
      }
*/
      bool runOMPLUTM(double startX, double startY, double endX, double endY) {
        if(m_OMPLsetup) {
          m_OMPLsetup.reset();
        }
        double extension = 300;
        double planningBounds[4] = {std::min(startY, endY)-extension, std::min(startX, endX)-extension, std::max(startY, endY)+extension, std::max(startX, endX)+extension};
        spew("Planning bounds:  %f, %f, %f, %f", planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3]);        

        try {
          m_OMPLsetup = std::make_unique<og::SimpleSetup>(OMPLintegrationENCGIS::createSetup(planningBounds[0], planningBounds[1], planningBounds[2], planningBounds[3], pointCheck.get(), lineCheck.get()));
          inf("OMPL init sucess 1");

          std::cout << std::setprecision(12) << "Increase printpres to 12 from bitstars setprecision(5) call" << std::endl;
          OMPLintegrationENCGIS::setStartAndGoalStates(*m_OMPLsetup, startX, startY, endX, endY);

          og::PathGeometric states = OMPLintegrationENCGIS::findPath(*m_OMPLsetup, m_args.OMPLmaxPlanningTime, OMPLintegrationENCGIS::configurations_t::C_KBIT);
          if (states.getStateCount()) {
              m_path_to_target = OMPLforDUNE::pathToVector(states);
              m_path_to_target = m_con->transformSRIDVector(m_path_to_target, 32632,4326);
              std::reverse(m_path_to_target.begin(), m_path_to_target.end()); // Reverse so that pop back will give the most recent post
              inf("Created path from: %f, %f to %f ,%f", startX, startY, endX, endY);
              return true;
          } else {
              err("Error finding path from: %f, %f to %f ,%f", startX, startY, endX, endY);
              spew("Start Pointinlayer: %d", pointCheck->run(startX, startY));
              spew("End Pointinlayer: %d", pointCheck->run(endX, endY));
              return false; 
          }   
        } catch(...) {
          err("Error using OMPL");
          return false;
        }
        return false;
      }

      bool checkDistanceToMonitoredVehicles() {
        for(auto iter : monitoredVehicles) {
          // Check if timestamp is recent
          if(std::get<2>(iter.second).getDelta() > 30.0) {
            continue;
          }
          // Check if distance threashold too large
          if(!checkSafety(std::get<0>(iter.second), std::get<1>(iter.second))) {
            return true; // Distance limit violated
          }
        }
        return false; // No distance limits violated
      }

      //! Function to check if the vehicle is getting near to the next waypoint
      void
      onPathControlState(const IMC::PathControlState* pcs)
      {
        
        //static bool path_recalculated = false;
        m_has_pcs = true;
        m_last_pcs.reset();
        if (pcs->flags & IMC::PathControlState::FL_NEAR) {
          if(m_path_to_target.empty()) {
            spew("m_path_to_target empty, disabling movement");
            enableMovement(false);
          } else if( 1 == m_path_to_target.size()) {
            m_path_to_target.pop_back();
          } else {
            m_path.start_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
            m_path.start_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
            m_path_to_target.pop_back(); // Remove fullfilled waypoint
            m_path.end_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
            m_path.end_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
            dispatch(m_path);
            inf("Using m_path_to_target");
                  for(auto iter : m_path_to_target) {
                    inf("%f, %f", iter.first, iter.second);
                  }
          }
        }
      }

      void
      onStateReport(void)
      {
        if (!m_maneuver.duration)
          return;

        // if the present location is unsafe then disable movement
        if (!checkSafety(m_last_known_lat, m_last_known_lon))
          enableMovement(false);

        if (m_last_update.overflow())
          signalError(DTR("timeout to receive new remote info was exceeded."));

        double delta = Clock::get() - m_start_time - m_maneuver.duration;
        if (delta >= 0)
          signalCompletion();
        else
          signalProgress((uint16_t)(Math::round(delta)));
      }

      //! Function to compute new point to send to vehicle considering offsets
      void
      computeNEDOffsets(double &lat, double &lon, double depth, double psi)
      {
        double offx, offy;

        m_path.end_z = depth + m_maneuver.z;
        m_path.end_z_units = m_maneuver.z_units;

        // compute the offsets in the BODY frame
        offx = std::cos(psi) * m_maneuver.x
        + std::cos(Angles::normalizeRadian(psi - DUNE::Math::c_half_pi)) * m_maneuver.y;

        offy = std::sin(psi) * m_maneuver.x
        + std::sin(Angles::normalizeRadian(psi - DUNE::Math::c_half_pi)) * m_maneuver.y;

        WGS84::displace(offx, offy, &lat, &lon);
        //debug("ComputeNEDOffsets offx: %f offy %f, psi: %f", offx, offy, psi);
      }

      //! Routine for checking the safety of the vehicle's position
      //! this routine return true if the present location is safe
      //! and returns false otherwise
      bool
      checkSafety(double lat, double lon)
      {
        if (m_has_estimated_state)
        {
          double x, y, r;

          WGS84::displacement(m_estate.lat, m_estate.lon, 0.0, lat, lon, 0.0, &x, &y);

          r = Math::norm((x - m_estate.x), (y - m_estate.y));

          // if the distance between them is below the safe distance
          if (r < m_args.safe_distance)
          {
            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          return true;
        }
      }

      //! Function for enabling and disabling the control loops
      void
      enableMovement(bool enable)
      {
        const uint32_t mask = IMC::CL_PATH;
        if (enable) {
          setControl(mask);
        } else {
          setControl(0);
        }
          
      }
    };
  }
}

DUNE_TASK
