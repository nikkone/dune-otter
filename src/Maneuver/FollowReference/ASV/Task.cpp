//***************************************************************************
// Copyright 2007-2023 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Jose Pinto (Edited by Nikolai Lauvås to fit ASV)                 *
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
  namespace FollowReference
  {
    namespace ASV
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //!
        float horizontal_tolerance;
        //!
        float loitering_radius;
        //!
        float default_speed;
        //!
        std::string default_speed_units;
        //! Enable/disable the anti-obstacle mechanism
        bool anti_obstacle;
        //! Enable/disable the anti-collision mechanism
        bool anti_collision;
        //! The path of the Spatialite database containing the electronic navigational charts
        std::string encDBpath;
        //! The maxmum planning time allowed for OMPL planning
        float OMPLmaxPlanningTime;
        //! Navigable Layer/table Name from encDBpath
        std::string dbNavigableLayerName;
        //! Innavigable Layer/table Name from encDBpath
        std::string dbInnavigableLayerName;

        bool useClosestSafePoint;

        //! The names of other IMC vehicles to avoid while performing this maneuver.
        std::vector<std::string> otherVehicles;

        double safe_distance;
      };

      struct Task : public DUNE::Maneuvers::Maneuver
      {
        //! Task arguments.
        Arguments m_args;
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
        //! Store maneuver specification
        IMC::FollowReference m_spec;
        //! Store latest received reference
        IMC::Reference m_cur_ref;
        IMC::Reference m_last_ref;
        //! Estimated state
        IMC::EstimatedState m_estate;
        //! Path Control state
        IMC::PathControlState m_pcs;
        //! FollowRefState
        IMC::FollowRefState m_fref_state;
        //! Did we get a reference already?
        bool m_got_reference;
        //! Did we get a reference start loc already?
        bool m_got_reference_start;
        double m_start_lat;
        double m_start_lon;
        double m_start_z;
        //! Are we moving or idle (floating)
        bool m_moving;
        //! Store last timestamp when reference was received
        double m_last_ref_time;
        //! sent path ref
        int m_path_ref;
        //! Path to the desired position calculated from the followed system
        std::vector<std::pair<double,double>> m_path_to_target;
        //! The most recent reference endpoint in UTM found by the closestSafePoint algorithm
        std::pair<double,double> m_last_utm_pos_end;
        //! Source identifiers for the monitiored vehicles
        std::map<uint16_t, std::tuple<fp64_t, fp64_t, DUNE::Time::Delta>> monitoredVehicles;
        //! this boolean tells us if we have an estimated state already
        bool m_has_estimated_state;

        bool m_anti_collision;

        bool distanceLimitBreached;

        IMC::DesiredPath m_path;
        bool m_path_sent;
        static const uint8_t Z_CHANGED         = 1;
        static const uint8_t SPEED_CHANGED     = 2;
        static const uint8_t LOC_CHANGED       = 4;
        static const uint8_t RADIUS_CHANGED    = 8;

        Task(const std::string& name, Tasks::Context& ctx) :
          DUNE::Maneuvers::Maneuver(name, ctx),
          m_has_estimated_state(false),
          distanceLimitBreached(false)
        {

          param("Loitering Radius", m_args.loitering_radius)
          .defaultValue("7.5")
          .units(Units::Meter)
          .description("Radius of loitering circle after arriving at destination");

          param("Horizontal Tolerance", m_args.horizontal_tolerance)
          .defaultValue("15.0").units(Units::Meter)
          .description("Minimum distance required to consider that the vehicle has arrived at the reference (XY)");

          param("Default Speed", m_args.default_speed)
          .defaultValue("2")
          .description("Speed to use in case no speed is given by reference source.");

          param("Default Speed Units", m_args.default_speed_units)
          .defaultValue("m/s")
          .description("Units to use for default speed (one of 'm/s', 'rpm' or '%').");

          param("Use anti-obstacle", m_args.anti_obstacle)
              .defaultValue("false")
              .description("Period between sync messages");

          param("Use anti-collision", m_args.anti_collision)
              .defaultValue("false")
              .description("Period between sync messages");

          param("Use ClosestSafePoint", m_args.useClosestSafePoint)
              .defaultValue("true")
              .description("Period between sync messages");

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

          param("Other Vehicles", m_args.otherVehicles)
          .description("The source/vehicle names of other entities in the system.")
          .defaultValue("ntnu-otter-03");

          param("Minimum Safe Distance", m_args.safe_distance)
          .defaultValue("15.0")
          .units(Units::Meter)
          .description("Minimum safe distance to other vehicles");

          m_got_reference = false;
          m_got_reference_start = false;
          m_start_lat = 0;
          m_start_lon = 0;
          m_moving = false;
          m_last_ref_time = 0;
          m_path_sent = false;
          m_path_ref = 0;

          bindToManeuver<Task, IMC::FollowReference>();
          bind<IMC::Announce>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::Reference>(this);
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }

      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.otherVehicles)) {
          monitoredVehicles.clear();
          for(auto iter : m_args.otherVehicles) {
              monitoredVehicles[resolveSystemName(iter)] = std::tuple<fp64_t, fp64_t, DUNE::Time::Delta>{0.0,0.0,DUNE::Time::Delta()};
          }
          for(auto iter : monitoredVehicles) {
              inf("Monitored Vehicle: %d", iter.first);
          }
        }
        if(paramChanged(m_args.anti_collision)) {
          m_anti_collision = m_args.anti_collision;
        }
      }

        //! Acquire resources.
        void onResourceAcquisition(void)
        {
          try{
            m_con = std::make_shared<ENCGIS::DBconnection>(m_args.encDBpath, SQLITE_OPEN_READONLY, 32632);
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

        if(!monitoredVehicles.empty()) 
          monitoredVehicles.clear();

        for(auto iter : m_args.otherVehicles) {
            monitoredVehicles[resolveSystemName(iter)] = std::tuple<fp64_t, fp64_t, DUNE::Time::Delta>{0.0,0.0,DUNE::Time::Delta()};
        }
        for(auto iter : monitoredVehicles) {
            inf("Monitored Vehicle: %d", iter.first);
        }
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
          m_has_estimated_state = false;
          m_path_to_target.clear();
          //m_has_pcs = false;
          m_path_sent = false;
        }

        void
        consume(const IMC::Announce* msg)
        {
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
        }

        void
        consume(const IMC::FollowReference* msg)
        {
          m_moving = false;
          m_got_reference = false;
          m_got_reference_start = false;
          m_start_lat = 0;
          m_start_lon = 0;
          m_spec = *msg;
          m_last_ref_time = Clock::get();

          m_fref_state.proximity = IMC::FollowRefState::PROX_FAR;
          m_fref_state.state = IMC::FollowRefState::FR_WAIT;
          m_fref_state.control_ent = msg->control_ent;
          m_fref_state.control_src = msg->control_src;

          // send a notify to controlling peer that the maneuver was activated
          dispatch(m_fref_state);
          m_last_ref = IMC::Reference();
          m_path = IMC::DesiredPath();
          m_path_sent = false;
          inf(DTR("waiting for first reference"));
        }

        //! Consume Reference messages and generate DesiredPath messages accordingly
        //! Whenever a new Reference is received from a valid source, a new desired_path
        //! gets commanded to the vehicle.
        //! @see https://!whale.fe.up.pt/imc/doc/trunk/Maneuvering.html#follow-reference-maneuver
        //! @param msg the Reference message to be processed
        void
        consume(const IMC::Reference* msg)
        {
          if(msg->getDestination() != getSystemId()) {
            //spew("Ignored reference not to this vehicle");
            return;
          }

          // verify that the source is acceptable
          if (m_spec.control_src != 0xFFFF
              && m_spec.control_src != msg->getSource())
          {
            inf(DTR("ignored reference from non-authorized source: %d"),
                msg->getSource());
            return;
          }

          // verify that the source entity is acceptable
          if (m_spec.control_ent != 0xFF
              && m_spec.control_ent != msg->getSourceEntity())
          {
            inf(DTR("ignored reference from non-authorized entity: %d"),
                msg->getSource());
            return;
          }

          m_got_reference = true;
          m_last_ref_time = Clock::get();

          m_last_ref = m_cur_ref;
          if (msg->flags & IMC::Reference::FLAG_LOCATION) {
            m_cur_ref = *msg;
          }

          if (msg->flags & IMC::Reference::FLAG_MANDONE)
          {
            m_fref_state.proximity = IMC::FollowRefState::PROX_FAR;
            m_fref_state.state = IMC::FollowRefState::FR_WAIT;

            signalCompletion("maneuver terminated by reference source");
            return;
          }

          IMC::Reference ref = *msg;
          guide(&m_pcs, &ref, &m_estate);
        }

        void
        consume(const IMC::EstimatedState* msg)
        {
          if (msg->getSource() != getSystemId())
            return;
          m_estate = *msg;
          m_has_estimated_state = true;

          if(m_anti_collision && checkDistanceToMonitoredVehicles()) {
            spew("Distance Limit Violated, disabling movement");
            enableMovement(false);
            m_path_to_target.clear();
            distanceLimitBreached = true;
          } else {
            distanceLimitBreached = false;
            //if(!m_moving && ) {
            //  enableMovement(true);
            //}
          }

          double delta = 0;
          if (m_spec.timeout != 0)
            delta = Clock::get() - m_last_ref_time;

          if (delta > m_spec.timeout)
          {
            m_fref_state.state = IMC::FollowRefState::FR_TIMEOUT;
            dispatch(m_fref_state);
            signalError("reference source timed out");
          }
        }

        void
        onPathControlState(const IMC::PathControlState* pcs)
        {
          m_pcs = *pcs;
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
              dispatchDesiredPath(m_path);
              inf("Using m_path_to_target");
                    for(auto iter : m_path_to_target) {
                      inf("%f, %f", iter.first, iter.second);
                    }
            }
          }

        }

        void
        guide(const IMC::PathControlState* pcs, IMC::Reference* ref,
              const IMC::EstimatedState* state)
        {
          // start building the DesiredPath message to be commanded
          IMC::DesiredPath desired_path;
          desired_path.start_z = 0;
          desired_path.start_z_units = IMC::Z_DEPTH;
          desired_path.end_z = 0;
          desired_path.end_z_units = IMC::Z_DEPTH;

          double curlat = state->lat;
          double curlon = state->lon;
          bool near_ref = (pcs == NULL) || pcs->path_ref != m_path.path_ref ? false :
          (pcs->flags & IMC::PathControlState::FL_NEAR) != 0;

          WGS84::displace(state->x, state->y, &curlat, &curlon); // Only usefull if there is a x,y offset in state. Not needed for purely GPS navigation

          // command start corresponds to current position

          // set attributes in desired path according to flags
          updateStartLoc(ref, desired_path, curlat, curlon);
          updateEndLoc(ref, desired_path, curlat, curlon);
          updateSpeed(ref, desired_path);


          if(m_args.useClosestSafePoint) {
            m_con->transformSRID(Math::Angles::degrees(desired_path.end_lon), Math::Angles::degrees(desired_path.end_lat), 4326, m_last_utm_pos_end.first, m_last_utm_pos_end.second, 32632);
            inf("Original Pos: %f, %f", m_last_utm_pos_end.first, m_last_utm_pos_end.second);
            if(m_con->findClosestSafePointUTM(m_last_utm_pos_end.first, m_last_utm_pos_end.second)) {
              //inf("Original Pos: %f, %f", desired_path.end_lat, desired_path.end_lon);
              inf("Safe Pos: %f, %f", m_last_utm_pos_end.first, m_last_utm_pos_end.second);
              spew("End Pointinlayer: %d", pointCheck->run(m_last_utm_pos_end.first, m_last_utm_pos_end.second));
            } else {
              err("findClosestSafePoint failed, probably DB error.");
              //inf("Original Pos: %f, %f", desired_path.end_lat, desired_path.end_lon);
              inf("Safe Pos: %f, %f", m_last_utm_pos_end.first, m_last_utm_pos_end.second);
              return;
            }

            m_con->transformSRID(m_last_utm_pos_end.first, m_last_utm_pos_end.second, 32632, desired_path.end_lon, desired_path.end_lat, 4326);
            desired_path.end_lat = DUNE::Math::Angles::radians(desired_path.end_lat);
            desired_path.end_lon = DUNE::Math::Angles::radians(desired_path.end_lon);
          }
          // check to see if we are already at the target...
          double xy_dist = WGS84::distance(desired_path.end_lat,
                                           desired_path.end_lon, 0, curlat,
                                           curlon, 0);
          bool at_xy_target = xy_dist < std::fabs(ref->radius) + m_args.horizontal_tolerance;

          bool still_same_reference = (ref->flags & IMC::Reference::FLAG_START_POINT) ||
                                      sameReference(ref, &m_last_ref);

          if(!at_xy_target) {
            bool unchangedSpeed = sameSpeed(ref, &m_last_ref);
            bool unchangedRadius = true;
            if(std::fabs(ref->radius) != m_last_ref.radius) {
              unchangedRadius = false;
            }
            still_same_reference = still_same_reference && unchangedSpeed;
            if ((!unchangedSpeed || !unchangedRadius) && m_path_sent) {
              IMC::DesiredSpeed desSpeed;
              desSpeed.value = desired_path.speed;
              desSpeed.speed_units = desired_path.speed_units;
              inf(DTR("Speed reference changed to %f"), desSpeed.value);
              dispatch(desSpeed);
            }
          }

          updateRadius(ref, desired_path);
          int prev_mode = m_fref_state.state;

          debug("Mode: %s, XY_DIST: %f/%d, SAME_REF: %d",
          				modeToStr(prev_mode).c_str(), xy_dist, at_xy_target, still_same_reference);

// Update m_fref_state.state
          if (still_same_reference && prev_mode != IMC::FollowRefState::FR_WAIT)
          {
            switch (prev_mode)
            {
              case (IMC::FollowRefState::FR_GOTO):
                if (near_ref)
                {
                  m_fref_state.state = IMC::FollowRefState::FR_HOVER;
                }
                break;
              case (IMC::FollowRefState::FR_HOVER):
                if (!at_xy_target)
                  m_fref_state.state = IMC::FollowRefState::FR_GOTO;
                break;
              case (IMC::FollowRefState::FR_LOITER):
                err("Loiter encountered.");
                break;
              default:
                err("Unsuported prev_mode received.");
                return;
            }
          }
          else
          {
            if (!at_xy_target)
              m_fref_state.state = IMC::FollowRefState::FR_GOTO;
          }

          desired_path.lradius = 0;

          m_fref_state.proximity = 0;

          if (at_xy_target)
            m_fref_state.proximity |= IMC::FollowRefState::PROX_XY_NEAR;

          if (!at_xy_target)
            m_fref_state.proximity = IMC::FollowRefState::PROX_FAR;

          m_fref_state.reference.set(*ref);
          dispatch(m_fref_state);

          if (sameReference(ref, &m_last_ref) && m_fref_state.state == prev_mode
              && m_fref_state.state != IMC::FollowRefState::FR_WAIT)
          {
            // nothing to do
            return;
          }

          if ( (!ref->speed.isNull() && ref->speed.get()->value < 0.1)) {
            war("Movement disabled, speed is zero");
            enableMovement(false);
          } else {
            updateDesiredPath(desired_path, at_xy_target);
          } 
        }

        //! Function for enabling and disabling the control loops
        void
        enableMovement(bool enable)
        {
          const uint32_t mask = IMC::CL_PATH;

          if (enable)
          {
            // set control loops in order to move
            bool was_moving = m_moving;
            setControl(mask);
            m_moving = true;
            if (!was_moving)
            {
              m_path_sent = false;
              updateDesiredPath(m_path, false);
            }
          }
          else
          {
            // stop moving by setting control loops to zero
            m_moving = false;
            setControl(0);
          }
        }

      bool findSafePath(double desired_lat, double desired_lon) {
            m_path.flags |= IMC::DesiredPath::FL_START;
            if(!m_path_to_target.empty()) {
              m_path_to_target.clear();
              inf("Target moved, recalculating path");
            }
            if(m_path_to_target.empty()) {
              //////////////////////////////////////////////////// Find end point of planner
              std::pair<double,double> utmend;
              m_con->transformSRID(Math::Angles::degrees(desired_lon), Math::Angles::degrees(desired_lat), 4326, utmend.first, utmend.second, 32632);
                inf("Original Pos: %f, %f", utmend.first, utmend.second);
              if(m_con->findClosestSafePointUTM(utmend.first, utmend.second)) {
                inf("Safe Pos: %f, %f", utmend.first, utmend.second);
                spew("End Pointinlayer: %d", pointCheck->run(utmend.first, utmend.second));
              } else {
                err("findClosestSafePoint failed on endpoint, probably DB error.");
                return false;
              }
              ////////////////////////////////////////////////////
              std::pair<double,double> utmstart;
              m_con->transformSRID(Math::Angles::degrees(m_estate.lon), Math::Angles::degrees(m_estate.lat), 4326, utmstart.first, utmstart.second, 32632);
              if(!pointCheck->run(utmstart.first, utmstart.second)) {
                war("Startpoint collison");
                inf("Original Pos: %f, %f", utmstart.first, utmstart.second);
                if(m_con->findClosestSafePointUTM(utmstart.first, utmstart.second)) {
                  //inf("Original Pos: %f, %f", desired_lat, desired_lon);
                  inf("Safe Pos: %f, %f", utmstart.first, utmstart.second);
                  spew("Start Pointinlayer: %d", pointCheck->run(utmstart.first, utmstart.second));
                } else {
                  err("findClosestSafePoint failed on current location, probably DB error.");
                  return false;
                }
              }


              
              if(runOMPLUTM(utmstart.first, utmstart.second, utmend.first, utmend.second)) {
                  //m_path_to_target.pop_back(); // Remove first waypoint (Current position)
                  m_path.start_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
                  m_path.start_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
                  m_path_to_target.pop_back(); // Remove first waypoint (Current position)
                  m_path.end_lat = DUNE::Math::Angles::radians(m_path_to_target.back().second);
                  m_path.end_lon = DUNE::Math::Angles::radians(m_path_to_target.back().first);
                  if(!m_moving) {
                    enableMovement(true);
                  }
                  dispatchDesiredPath(m_path);
                  m_path_sent = true;
                  return true;
              } else {
                m_path.end_lat = m_estate.lat;
                m_path.end_lon = m_estate.lon;
                war("OMPL failed, doing nothing");
                return false;
              }
            }
        return false;
      }
      
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

          //std::cout << std::setprecision(12) << "Increase printpres to 12 from bitstars setprecision(5) call" << std::endl;
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

      private:

        uint8_t pathDifferences(const IMC::DesiredPath *msg1, const IMC::DesiredPath *msg2)
        {
          uint8_t flags = 0;

          if (msg1->end_lat != msg2->end_lat)
            flags |= LOC_CHANGED;
          if (msg1->end_lon != msg2->end_lon)
            flags |= LOC_CHANGED;
          if (msg1->lradius != msg2->lradius)
            flags |= RADIUS_CHANGED;
          if (msg1->end_z != msg2->end_z || msg1->end_z_units != msg2->end_z_units)
            flags |= Z_CHANGED;

          if (msg1->speed != msg2->speed || msg1->speed_units != msg2->speed_units)
            flags |= SPEED_CHANGED;

          return flags;
        }

        bool
        sameReference(const IMC::Reference *msg1, const IMC::Reference *msg2)
        {
          if (msg1->flags != msg2->flags)
            return false;
          if (msg1->lat != msg2->lat)
            return false;
          if (msg1->lon != msg2->lon)
            return false;
          if (msg1->radius != msg2->radius)
            return false;

          if (msg1->z.isNull() != msg2->z.isNull())
          {
            return false;
          }
          else if (!msg1->z.isNull())
          {
            const IMC::DesiredZ *z1 = msg1->z.get();
            const IMC::DesiredZ *z2 = msg2->z.get();

            if (!z1->fieldsEqual(*z2))
              return false;
          }

          return true;
        }
        
        bool sameSpeed(const IMC::Reference *msg1, const IMC::Reference *msg2) {
          if (msg1->speed.isNull() != msg2->speed.isNull())
          {
            return false;
          }
          else if (!msg1->speed.isNull())
          {
            const IMC::DesiredSpeed *s1 = msg1->speed.get();
            const IMC::DesiredSpeed *s2 = msg2->speed.get();

            if (!s1->fieldsEqual(*s2))
              return false;
          }
          return true;
        }

        IMC::SpeedUnits
        parseSpeedUnitsStr(std::string sunits_str)
        {
          if (sunits_str == "m/s")
            return IMC::SUNITS_METERS_PS;
          else if (sunits_str == "rpm")
            return IMC::SUNITS_RPM;
          else
            return IMC::SUNITS_PERCENTAGE;
        }

        std::string modeToStr(int prev_mode) {
           std::string mode;
          switch (prev_mode)
          {
          case IMC::FollowRefState::FR_GOTO:
        	mode = "GOTO";
        	break;
          case IMC::FollowRefState::FR_HOVER:
        	mode = "Hover";
            break;
          case IMC::FollowRefState::FR_LOITER:
            mode = "Loiter";
        	break;
          case IMC::FollowRefState::FR_ELEVATOR:
            mode = "Elevator";
            break;
          default:
            mode = "Hover";
            break;
          }
          return mode;
        }

        void
        updateStartLoc(const IMC::Reference* ref, IMC::DesiredPath &desired_path,
                     double curlat, double curlon)
        {
          // set end location according to received reference
          if (ref->flags & IMC::Reference::FLAG_DIRECT)
          {
            spew("Using direct path following");
            m_got_reference_start = false;
            m_start_lat = 0;
            m_start_lon = 0;
            m_start_z = 0;
            // just stay where we are
            desired_path.start_lat = curlat;
            desired_path.start_lon = curlon;
            desired_path.start_z = 0;
            desired_path.start_z_units = ZUnits::Z_NONE;
            desired_path.flags = IMC::DesiredPath::FL_DIRECT;
          }
          else if (ref->flags & IMC::Reference::FLAG_START_POINT)
          {
            spew("Using sent start point path following");
            m_got_reference_start = true;
            m_start_lat = ref->lat;
            m_start_lon = ref->lon;
            desired_path.start_lat = m_start_lat;
            desired_path.start_lon = m_start_lon;
            desired_path.start_z = m_start_z;
            desired_path.start_z_units = m_start_z >= 0 ? ZUnits::Z_DEPTH : ZUnits::Z_NONE;
            desired_path.flags = IMC::DesiredPath::FL_START;
          }
          else if (m_got_reference_start && m_start_lat != 0 && m_start_lon != 0)
          {
            spew("Keeping last sent start point path following");
            // use previously received reference
            desired_path.start_lat = m_start_lat;
            desired_path.start_lon = m_start_lon;
            desired_path.start_z = m_start_z;
            desired_path.start_z_units = m_start_z >= 0 ? ZUnits::Z_DEPTH : ZUnits::Z_NONE;
            desired_path.flags = IMC::DesiredPath::FL_START;
          }
          else
          {
            spew("Failing back to using direct path following");
            desired_path.flags = IMC::DesiredPath::FL_DIRECT;
          }
        }

        void
        updateEndLoc(const IMC::Reference* ref, IMC::DesiredPath &desired_path,
                     double curlat, double curlon)
        {
          // set end location according to received reference
          if (ref->flags & IMC::Reference::FLAG_LOCATION)
          {
            // use new reference
            desired_path.end_lat = ref->lat;
            desired_path.end_lon = ref->lon;
          }
          else if (m_got_reference && (m_cur_ref.flags & IMC::Reference::FLAG_LOCATION))
          {
            // use previously received reference
            desired_path.end_lat = m_cur_ref.lat;
            desired_path.end_lon = m_cur_ref.lon;
          }
          else
          {
            // just stay where we are
            desired_path.end_lat = curlat;
            desired_path.end_lon = curlon;
          }
        }

        void
        updateSpeed(const IMC::Reference* ref, IMC::DesiredPath &desired_path)
        {
          // set speed according to received reference. If the reference does not
          // provide a desired speed, use last sent speed
          if ((ref->flags & IMC::Reference::FLAG_SPEED) && !(ref->speed.isNull()))
          {
            desired_path.speed = ref->speed->value;
            desired_path.speed_units = ref->speed->speed_units;
          }
          else if (m_got_reference && !m_cur_ref.speed.isNull())
          {
            desired_path.speed = m_cur_ref.speed->value;
            desired_path.speed_units = m_cur_ref.speed->speed_units;
          }
          else
          {
            // default speed
            desired_path.speed = m_args.default_speed;
            desired_path.speed_units = parseSpeedUnitsStr(m_args.default_speed_units);
          }
        }

        void
        updateRadius(IMC::Reference* ref, IMC::DesiredPath &desired_path)
        {
          //std::cout<< " starting radius " << ref->radius << "  -  " << desired_path.lradius << "\n";
          // set speed according to received reference. If the reference does not
          // provide a desired speed, use last sent speed
          if (ref->flags & IMC::Reference::FLAG_RADIUS)
          {
            desired_path.lradius = ref->radius;
          }
          else if (m_got_reference && m_cur_ref.radius != 0)
          {
            desired_path.lradius = m_cur_ref.radius;
          }
          else
          {
            // default radius
            desired_path.lradius = m_args.loitering_radius;
          }

          if(desired_path.lradius < 0)
          {
            desired_path.flags |= DesiredPath::FL_CCLOCKW;
            desired_path.lradius = desired_path.lradius * -1;
          }
        }

        void
        dispatchDesiredPath(IMC::DesiredPath desired_path)
        {
          desired_path.path_ref = ++m_path_ref;
          dispatch(desired_path);
          m_path = desired_path;
        }

        void
        updateDesiredPath(IMC::DesiredPath desired_path, bool at_xy_target)
        {

          int diff = pathDifferences(&m_path, &desired_path);
          desired_path.flags &= ~DesiredPath::FL_NO_Z;

          m_path = desired_path;


          bool changedLoc = (diff & LOC_CHANGED) != 0;
          bool changedSpeed = (diff & SPEED_CHANGED) != 0;
          bool changedRadius = (diff & RADIUS_CHANGED) != 0;

          if (changedRadius)
          {
            inf(DTR("Loiter radius reference changed to %f"), desired_path.lradius);
          }

          bool send_desired_path = (changedSpeed && !at_xy_target) || changedRadius || changedLoc || !m_path_sent;

          // dispatch new desired path
          switch (m_fref_state.state)
          {
            case (IMC::FollowRefState::FR_LOITER):
              enableMovement(true);
              if (send_desired_path)
              {
                dispatchDesiredPath(desired_path);
                inf(DTR("loitering around (%f, %f, %f, %f)."),
                    Angles::degrees(desired_path.end_lat), Angles::degrees(desired_path.end_lon),
                    desired_path.end_z, desired_path.lradius);
              }
              break;
            case (IMC::FollowRefState::FR_ELEVATOR):
              enableMovement(true);
              if (send_desired_path)
              {
                dispatchDesiredPath(desired_path);
                inf(DTR("loitering (elevator) towards (%f, %f, %f, %f)."),
                    Angles::degrees(desired_path.end_lat), Angles::degrees(desired_path.end_lon),
                    desired_path.end_z, desired_path.lradius);
              }

              break;
            case (IMC::FollowRefState::FR_GOTO):
              enableMovement(true);
              if (send_desired_path)
              {
                if(m_args.anti_obstacle) {
                  findSafePath(desired_path.end_lat, desired_path.end_lon);
                } else {
                  dispatchDesiredPath(desired_path);
                  inf(DTR("going towards (%f, %f, %f)."), Angles::degrees(desired_path.end_lat),
                      Angles::degrees(desired_path.end_lon), desired_path.end_z);
                }

              }
              break;
            case (IMC::FollowRefState::FR_WAIT):
            case (IMC::FollowRefState::FR_HOVER):
              if (send_desired_path && !m_args.anti_obstacle)
              {
            	dispatchDesiredPath(desired_path);
            	enableMovement(true);
                inf(DTR("hovering next to (%f, %f)."), Angles::degrees(desired_path.end_lat),
                    Angles::degrees(desired_path.end_lon));
              }
              enableMovement(false);
              break;
            default:
              err("Unknown FollowRefState");
              return;
          }
          if(send_desired_path)
            m_path_sent = true;
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

      };
    }
  }
}

DUNE_TASK
