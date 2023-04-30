//***************************************************************************
// Copyright 2013-2023 Norwegian University of Science and Technology (NTNU)*
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

// DUNE::Control::Formation::FishTracker::Task::updateReference
#include <ENCGIS/DBconnection.hpp>
#include <ENCGIS/isPointInLayerStatement.hpp>
namespace Control
{
  //! Device driver for ThelmaHydrophone
  namespace Formation
  {
    namespace FishTracker
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Sync Period;
        double fishtag_min_interval;
        //! Sync Period;
        double ref_send_interval;
        //!
        double formation_rotation_step;
        //! The path of the Spatialite database containing the electronic navigational charts
        std::string encDBpath;
        //! Navigable Layer/table Name from encDBpath
        std::string dbNavigableLayerName;
        //!
        bool useCollisionMitigation;

        std::string participants;

        unsigned formationAllocator;
      };

      struct Task : public DUNE::Tasks::Task
      {
        Arguments m_args;
        //! Database connection
        std::shared_ptr<ENCGIS::DBconnection> m_con;
        //! Point collision check For use in path planner
        std::unique_ptr<ENCGIS::isPointInLayerStatement> pointCheck;
        //! Timer that is reset when a RemoteSensorInfo for the following entity is received. Top is m_minTagInterval
        Time::Counter<float> m_last_tag_timer;
        //! Timer.
        Time::Counter<float> m_ref_send_timer;
        //!
        IMC::RemoteSensorInfo m_last_rs_msg;

        IMC::otterFormation m_last_of_msg;

        DUNE::IMC::DesiredSpeed m_dsp;
        //! The rotation state of the entire formation
        double m_formation_rotate_rad;
        //! Initialized from args, may be overwritten by custom parameter
        double m_formation_rotation_step_rad;
        //! Initialized from args, may be overwritten by custom parameter
        double m_minTagInterval;
        //! TODO: Not sure if needed
        double m_maxTagInterval;
        //! The timeout used in the FollowReference maneuvers sent by this task
        double m_timeout;
        //! Initialized from args, may be overwritten by custom parameter
        double m_FollowRefInterval;
        //!
        bool m_formation_started;
        //!
        bool m_useCollisionMitigation;
        //! 
        double m_formation_radius;
        //! vehicleid - Lat(rad), Lon(rad), LastReference, time since last position update
        std::map<uint16_t, std::tuple<fp64_t, fp64_t, IMC::Reference, DUNE::Time::Delta>> m_participants; 
        typedef std::map<uint16_t, std::tuple<fp64_t, fp64_t, IMC::Reference, DUNE::Time::Delta>>::iterator participant_t;

        std::vector<participant_t> m_allocation;
        Task(const std::string &name, Tasks::Context &ctx) : 
        DUNE::Tasks::Task(name, ctx),
        m_formation_rotate_rad(0.0),
        m_formation_rotation_step_rad(M_PI/2) // 90deg
        {
          param("FishTag min interval", m_args.fishtag_min_interval)
              .units(Units::Second)
              .defaultValue("30.0")
              .minimumValue("0.0")
              .description("Period between sync messages");

          param("Reference Sending Interval", m_args.ref_send_interval)
              .units(Units::Second)
              .defaultValue("5.0")
              .minimumValue("0.0")
              .description("Period between sync messages");

          param("Rotation Step", m_args.formation_rotation_step)
              .units(Units::Radian)
              .defaultValue("0.0")
              .minimumValue("0.0")
              .maximumValue("6.284") // ~2PI
              .description("Period between sync messages");


          param("Use anti-collision strategies", m_args.useCollisionMitigation)
              .defaultValue("false")
              .description("Activate or deactivate the collision mitigation strategies");

          param("Formation Allocator", m_args.formationAllocator)
              .defaultValue("1")
              .description("0 for minmax, 1 for mintot");

          param("ENC DB Path", m_args.encDBpath)
          .defaultValue("/home/nikolai/lststools/dune/misc/re4utmfinal.sqlite")
          .description("The path of the DB to read ENC from.");

          param("Navigable Layer Name", m_args.dbNavigableLayerName)
          .defaultValue("navigable")
          .description("Navigable Layer Name");

          param("Participants", m_args.participants)
              .defaultValue("ntnu-otter-01,ntnu-otter-02,ntnu-otter-03")
              .description("Default participants if otterFormation message field is empty");

          bind<IMC::Abort>(this);
          bind<IMC::Announce>(this);
          bind<IMC::otterFormation>(this);
          bind<IMC::RemoteSensorInfo>(this);
          //bind<IMC::TBRFishTag>(this);
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }
        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if(paramChanged(m_args.ref_send_interval)) {
            m_ref_send_timer.setTop(m_args.ref_send_interval);
            m_FollowRefInterval = m_args.ref_send_interval;
          }
          if(paramChanged(m_args.formation_rotation_step))
            m_formation_rotation_step_rad = m_args.formation_rotation_step;
          if(paramChanged(m_args.fishtag_min_interval)) {
            m_last_tag_timer.setTop(m_args.fishtag_min_interval);
            m_minTagInterval =  m_args.fishtag_min_interval; 
          }
          if(paramChanged(m_args.useCollisionMitigation))
            m_useCollisionMitigation = m_args.useCollisionMitigation;
        }
        
        
        void
        onResourceAcquisition(void)
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

        }

        void
        onResourceRelease(void)
        {
        }

        void
        onResourceInitialization(void)
        {
          addVehicles(m_args.participants);
        }

        void dispatchToVehicle(uint16_t vehicle, IMC::Message *msg)
        {
          msg->setDestination(vehicle);
          dispatch(*msg);
        }

        void onActivation(void)
        {
          inf("Starting FishTracking plan...");
          IMC::PlanControl startPlan;
          startPlan.type = IMC::PlanControl::PC_REQUEST;
          startPlan.op = IMC::PlanControl::PC_START;
          startPlan.plan_id = "follow_fishTracker";
          IMC::FollowReference man;
          man.control_ent = getEntityId();
          man.control_src = getSystemId();
          man.altitude_interval = 0;
          man.timeout = m_timeout;

          IMC::PlanSpecification spec;

          spec.plan_id = "follow_fishTracker";
          spec.start_man_id = "formation";

          IMC::PlanManeuver pm;
          pm.data.set(man);
          pm.maneuver_id = "formation";
          spec.maneuvers.push_back(pm);
          startPlan.arg.set(spec);
          startPlan.request_id = 0;
          startPlan.flags = 0;
          startPlan.setDestination(m_ctx.resolver.id());
          for (const auto &participant : m_participants)
          {
            dispatchToVehicle(participant.first, &startPlan);
          }
          m_ref_send_timer.setTop(m_FollowRefInterval);
          m_last_tag_timer.setTop(m_minTagInterval);
          m_last_tag_timer.reset();
          m_ref_send_timer.reset();


          m_formation_rotate_rad = 0;
          m_formation_started = false;
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
          for (const auto &participant : m_participants)
          {
            dispatchToVehicle(participant.first, &stopPlan);
          }
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
          m_formation_started = false;
          m_allocation.clear();
        }

        void consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;
          
          war(DTR("Abort detected. Disabling FishSearch control."));
          requestDeactivation();
        }

        void consume(const IMC::Announce* msg)
        {
          if(!m_participants.empty()) {
            auto current = m_participants.find(msg->getSource());
            if( current != m_participants.end()) {
              // Add position to list, then check for collisions in EstimatedState
              std::get<0>(current->second) = msg->lat;
              std::get<1>(current->second) = msg->lon; 
              std::get<3>(current->second).reset(); 
            }
          }
        }

        void consume(const IMC::otterFormation* msg) {
          inf("Got otterFormation following %s", msg->target.c_str());
          
          switch(msg->msg_type) {
            case IMC::otterFormation::MessageTypeEnum::T_start:
              m_last_of_msg = *msg;
              m_formation_radius = msg->maxradius;
              m_dsp.value = msg->maxspeed;
              m_dsp.speed_units = msg->speed_units;
              if(!isActive()) {

                if(!msg->participants.empty()) {
                  m_participants.clear();
                }
                addVehicles(msg->participants);
                
                for (auto &participant : m_participants)
                {
                  std::get<2>(participant.second).speed.set(m_dsp);
                  std::get<2>(participant.second).flags = IMC::Reference::FlagsBits::FLAG_LOCATION | IMC::Reference::FlagsBits::FLAG_SPEED;
                }
                parseCustomParameters(msg->custom);
                requestActivation();
              }
              break;
            case IMC::otterFormation::MessageTypeEnum::T_stop:
              if(isActive()) {
                requestDeactivation();
              }
              break;
            case IMC::otterFormation::MessageTypeEnum::T_param_change:
              inf("Updating controller with parameter changes");
              m_dsp.value = msg->maxspeed;
              m_dsp.speed_units = msg->speed_units;
              parseCustomParameters(msg->custom);
              m_formation_radius = msg->maxradius;
              //TODO: Update speed on each participant
              break;
            default:
            err("Unsupported msg_type, doing nothing.");
          }
        }

        void consume(const IMC::RemoteSensorInfo *msg)
        {
          if(isActive()) {
            if(m_last_of_msg.target == msg->id) {
              m_formation_started = true;
              m_last_rs_msg = *msg;
              m_last_tag_timer.reset();
              if(m_formation_rotation_step_rad > 0.0) {
                spew("Rotating %f", m_formation_rotate_rad);
                m_formation_rotate_rad += m_formation_rotation_step_rad;
              }
            }
          }
        }

        void addVehicles(std::string participants) {
                std::vector<std::string> parts;
                String::split(participants, ",", parts);

                for (const auto &participant_name : parts) {
                  
                  uint16_t participant_id = resolveSystemName(participant_name);
                  spew("Vehicle added: %d", participant_id);
                  if(m_participants.find(participant_id) == m_participants.end()) {
                    m_participants[participant_id] = std::tuple<fp64_t, fp64_t, IMC::Reference, DUNE::Time::Delta>{0.0,0.0, IMC::Reference(),DUNE::Time::Delta()};
                    std::get<2>(m_participants[participant_id]).setDestination(participant_id);
                  }
                }
        }

        void parseCustomParameters(std::string parameters) {
          DUNE::Utils::TupleList custom = DUNE::Utils::TupleList(parameters);
          // Parse parameters
          double rotation_dist = custom.get("r", double(-1.0));
          double minTagInterval = custom.get("i", double(-1.0));
          double maxTagInterval = custom.get("x", double(-1.0));
          double timeout = custom.get("t", double(-1.0));
          double FollowRefInterval = custom.get("f", double(-1.0));
          // Validate an update parameters
          if(rotation_dist < 0.0) {
            war("No/Wrong 'r', using rotation_dist from task arguments");
          } else {
            m_formation_rotation_step_rad = rotation_dist;
          } if(minTagInterval < 0.0) {
            war("No/Wrong 'i', using minTagInterval from task arguments");
          } else {
            m_minTagInterval = minTagInterval;
            m_last_tag_timer.setTop(m_minTagInterval);
          } if(maxTagInterval < 0.0) {
            war("No/Wrong 'x', using maxTagInterval from task arguments");
          } else {
            m_maxTagInterval = maxTagInterval;
          } if(timeout < 0.0) {
            war("No/Wrong 't', using timeout from task arguments");
          } else {
            m_timeout = timeout;
          } if(FollowRefInterval < 0.0) {
            war("No/Wrong 'f', using FollowRefInterval from task arguments");
          } else {
            m_FollowRefInterval = FollowRefInterval;
            m_ref_send_timer.setTop(m_FollowRefInterval);
          }

          debug("r=%f, i=%f, x=%f, t=%f, f=%f", rotation_dist, minTagInterval, maxTagInterval, timeout, FollowRefInterval);
        }

        bool rotationChecker(double &x, double &y, double r, unsigned p, double &angle) {
          std::string query = "select X(s), Y(s), 2 * asin (0.5 * distance(s, e) / r), IIF(geometryType(inter) == 'LINESTRING', 1, 0) from ("
              "select r, startPoint(inter) as s, endPoint(inter) as e, inter from ("
                "select linemerge(difference(exteriorRing(BuildArea(MakeCircle(" + std::to_string(x) + "," + std::to_string(y) + ", " + std::to_string(r) + ", 32632))), unigeom)) as inter, " + std::to_string(r) + " as r from ("
                  "select Gunion(geometry) as unigeom,* from innavigable where ROWID IN (SELECT ROWID FROM SpatialIndex "
                  "WHERE f_table_name = 'innavigable' and "
                  "search_frame = BuildCircleMbr(" + std::to_string(x) + "," + std::to_string(y) + ", " + std::to_string(r) + ",32632))"
                ")"
              ")"
            ")";

          sqlite3_stmt* db_handle = nullptr;
          
          if (sqlite3_prepare_v2(m_con->db, query.c_str(), query.length(), &db_handle, 0) != SQLITE_OK) {
            if (db_handle) {
              sqlite3_finalize(db_handle);
            }
            return false; // False, could not execute sql statement
          }

          // Execute
          sqlite3_step(db_handle);
          x = sqlite3_column_double(db_handle, 0);
          y = sqlite3_column_double(db_handle, 1);
          angle = sqlite3_column_double(db_handle, 2);
          int valid = sqlite3_column_int(db_handle, 3);
          // Teardown
          if (db_handle) {
            sqlite3_finalize(db_handle);
          } else {
            return false; // False, something wrong with sqlite finalizing
          }
          //std::cout << "Valid: " << valid << std::endl;
          if(!valid) {
            return false;
          }
          if(angle < 2*M_PI/p) {
            return true; // Rotation possible from
          }
          return false; // Angle too big, rotation not possible
        }

        void rotate_point(double cx,double cy,double angle, double &px, double &py)
        {
          float s = sin(angle);
          float c = cos(angle);

          // translate point back to origin:
          px -= cx;
          py -= cy;

          // rotate point
          float xnew = px * c - py * s;
          float ynew = px * s + py * c;

          // translate point back:
          px = xnew + cx;
          py = ynew + cy;
        }

        bool formationCollisionHandler(double &x0, double &y0, double &radius, unsigned participants, double &angle) {
          if(participants > 1) {
            if(m_con->distanceToLayerWithinUTM(x0, y0, radius)) { 
              // Possibility of obstacle in formation
              double x=x0, y=y0;
              if(rotationChecker(x, y, radius, participants, angle)) {
                // Rotation possible
                spew("Rotation used as mitigation for formation collision");
                double px=x, py=y;

                double newAng = ((2*M_PI)/participants - angle)/2;

                rotate_point(x0, y0, newAng, px,py);
                if(pointCheck->run(px,py)) {
                  inf("Rotate (%f, %f) %f around (%f, %f), got (%f, %f)", x, y, newAng, x0, y0, px, py);
                } else {
                  px=x, py=y;
                  rotate_point(x0, y0, -newAng, px,py);
                  inf("Rotate (%f, %f) %f around (%f, %f), got (%f, %f)", x, y, -newAng, x0, y0, px, py);
                }

                double dy2 = 0;//y0 - y0;
                double dx2 = radius;//x0+radius - x0;
                double dy1 = py - y0;
                double dx1 = px - x0;
                angle = -atan2(dx1*dy2-dx2*dy1, dx1*dx2+dy1*dy2);
                return true;
              } else {
                double distanceToLand = m_con->distanceToLayerUTM(x0, y0, m_last_of_msg.maxradius);
                double minDistToLand = m_last_of_msg.minradius - m_last_of_msg.minradius*std::cos(M_PI/participants);
                if(distanceToLand > minDistToLand) {
                  // Reducing radius to land to keep formation in place
                  radius = distanceToLand; // Plus something because of rotation
                  spew("Reducing radius as mitigation for formation collision. Distance to obstacle: %f", distanceToLand);
                  return true; // TODO: When implemented, should be true;
                }/* else if(distanceToLand - m_accepted_formation_move > radius) {
                  // Moving formation from land to keep formation
                  m_con->findClosestSafePointUTM()

                  return true;
                }*/ else {
                  radius = m_last_of_msg.minradius;
                  // No collision mitigation strategy possible, falling back to closest safe spot
                  spew("No collision mitigation strategy possible/needed, falling back to closest safe spot");
                }
              }
            }
          }
          return false;
        }

        std::vector<std::pair<double, double>> generateTrackingFormation(double x0, double y0, double radius, unsigned numVertices, double rotAngle)
        {
          std::vector<std::pair<double, double>> retval;
          double angle = 2 * M_PI / numVertices; // Calculate the angle between each vertex
          if(numVertices%2) { // Odd number of searchers, regular polygon can be used
          //spew("Odd");
            for (unsigned i = 0; i < numVertices; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              retval.push_back(std::pair<double, double>(x, y));
            }
          } else { // Even number of searchers, must skew half of the regular polygon
           //spew("Even");
            for (unsigned i = 0; i < numVertices/2; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              //spew("Ri: %d, x: %f, y: %f", i,x,y);
              retval.push_back(std::pair<double, double>(x, y));
            }
            rotAngle -= angle/2;
            for (unsigned i = numVertices/2; i < numVertices; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              //spew("Si: %d, x: %f, y: %f", i,x,y);
              retval.push_back(std::pair<double, double>(x, y));
            }
          }

          return retval;
        }
         
        std::vector<participant_t> formationAllocator(const std::vector<std::pair<double, double>> pos) {
        if(m_args.formationAllocator == 1) {
          return formationAllocatorMinTot(pos);
        } else {
          return formationAllocatorMinMax(pos);
        }
        }

        /// @brief This function tries all allocation combinations, and selects the one minimizing the total length traveled
        /// Alternative approach TODO: minimize the longest distance traveled to reduce the time before formation is ready
        /// @param pos 
        std::vector<participant_t> formationAllocatorMinTot(const std::vector<std::pair<double, double>> pos) {
          std::vector<participant_t> retVal;

          if(pos.size() == 2){
            // Find positions in lat/lon
            double lon[2] = {m_last_rs_msg.lon, m_last_rs_msg.lon};
            double lat[2] = {m_last_rs_msg.lat, m_last_rs_msg.lat};
            for(unsigned i = 0; i<2;i++) {
              DUNE::Coordinates::WGS84::displace(pos[i].second, pos[i].first, &lat[i], &lon[i]);
              inf("%f, %f", lat[i], lon[i]);
            }
            // Calculate distances betwen all vehicles and all formation positions
            double distMatrix[2][2];
            {
              unsigned i=0;
              for (auto &participant : m_participants) {
                for(unsigned j = 0; j<2;j++) {
                  distMatrix[i][j] = DUNE::Coordinates::WGS84::distance((double)std::get<0>(participant.second), std::get<1>(participant.second), 0.0, lat[j], lon[j], 0.0);
                }
                i++;
              }
            }
            // Find distance sum of each solution and select best combination
            participant_t firstVehicle  = m_participants.begin();
            participant_t secondVehicle  = std::next(m_participants.begin());
            double minDist = distMatrix[0][0] + distMatrix[1][1];
            retVal.push_back(firstVehicle);
            retVal.push_back(secondVehicle);

            if( (distMatrix[0][1] + distMatrix[1][0]) < minDist) {
              retVal[0] = secondVehicle;
              retVal[1] = firstVehicle;
            }

          } else if(pos.size() == 3) {
            // Find positions in lat/lon
            double lon[3] = {m_last_rs_msg.lon, m_last_rs_msg.lon, m_last_rs_msg.lon};
            double lat[3] = {m_last_rs_msg.lat, m_last_rs_msg.lat, m_last_rs_msg.lat};
            for(unsigned i = 0; i<3;i++) {
              DUNE::Coordinates::WGS84::displace(pos[i].second, pos[i].first, &lat[i], &lon[i]);
              //inf("%f, %f", Math::Angles::degrees(lat[i]), Math::Angles::degrees(lon[i]));
            }

            // Calculate distances betwen all vehicles and all formation positions
            double distMatrix[3][3];
            {
              unsigned i=0;
              for (auto &participant : m_participants) {
                for(unsigned j = 0; j<3;j++) {
                  distMatrix[i][j] = DUNE::Coordinates::WGS84::distance((double)std::get<0>(participant.second), std::get<1>(participant.second), 0.0, lat[j], lon[j], 0.0);
                  //war("Dist %d, %f for (%d, %d)", participant.first, distMatrix[i][j], i, j);
                }
                i++;
              }
            }
            // Find distance sum of each solution and select best combination
            auto firstVehicle  = m_participants.begin();
            auto secondVehicle  = std::next(firstVehicle);
            auto thirdVehicle  = std::next(secondVehicle) ;

            
            retVal.push_back(firstVehicle);
            retVal.push_back(secondVehicle);
            retVal.push_back(thirdVehicle);


              //std::get<2>(firstVehicle->second).lon = lon[0];
              //std::get<2>(firstVehicle->second).lat = lat[0];
              //std::get<2>(secondVehicle->second).lon = lon[1];
              //std::get<2>(secondVehicle->second).lat = lat[1];
              //std::get<2>(thirdVehicle->second).lon = lon[2];
              //std::get<2>(thirdVehicle->second).lat = lat[2];

            double minDist = distMatrix[0][0] + distMatrix[1][1] + distMatrix[2][2];

            double tempsum = distMatrix[0][0] + distMatrix[1][2] + distMatrix[2][1];
            //inf("Tempsum 1: %f + %f + %f = %f", distMatrix[0][0], distMatrix[1][1], distMatrix[2][2], minDist);
            //for (auto &participant : retVal) {
            //    inf("1. %d", participant->first);
            //  }
            //inf("Tempsum 2: %f + %f + %f = %f", distMatrix[0][0], distMatrix[1][2], distMatrix[2][1], tempsum);
            if( tempsum < minDist) {
              minDist = tempsum;
              //participant->first);
              retVal[0] = firstVehicle;
              retVal[1] = thirdVehicle;
              retVal[2] = secondVehicle;
              //std::get<2>(firstVehicle->second).lon = lon[0];
              //std::get<2>(firstVehicle->second).lat = lat[0];
              //std::get<2>(thirdVehicle->second).lon = lon[1];
              //std::get<2>(thirdVehicle->second).lat = lat[1];
              //std::get<2>(secondVehicle->second).lon = lon[2];
              //std::get<2>(secondVehicle->second).lat = lat[2];
              //for (auto &participant : retVal) {
              //  inf("2- %d", participant->first);
             // }

            }
            tempsum = distMatrix[0][1] + distMatrix[1][0] + distMatrix[2][2];
            //inf("Tempsum 3: %f + %f + %f = %f", distMatrix[0][1], distMatrix[1][0], distMatrix[2][2], tempsum);
            if( tempsum < minDist) {
              minDist = tempsum;
              //participant->first);
              retVal[0] = secondVehicle;
              retVal[1] = firstVehicle;
              retVal[2] = thirdVehicle;
              //std::get<2>(secondVehicle->second).lon = lon[0];
              //std::get<2>(secondVehicle->second).lat = lat[0];
              //std::get<2>(firstVehicle->second).lon = lon[1];
              //std::get<2>(firstVehicle->second).lat = lat[1];
              //std::get<2>(thirdVehicle->second).lon = lon[2];
              //std::get<2>(thirdVehicle->second).lat = lat[2];
              //            for (auto &participant : retVal) {
              //  inf("3- %d", participant->first);
             // }

            }
            tempsum = distMatrix[0][1] + distMatrix[1][2] + distMatrix[2][0];
            //inf("Tempsum 4: %f + %f + %f = %f", distMatrix[0][1], distMatrix[1][2], distMatrix[2][0], tempsum);
            if( tempsum < minDist) {
              minDist = tempsum;
              //participant->first);
              retVal[0] = thirdVehicle;
              retVal[1] = firstVehicle;
              retVal[2] = secondVehicle;
              //std::get<2>(thirdVehicle->second).lon = lon[0];
              //std::get<2>(thirdVehicle->second).lat = lat[0];
              //std::get<2>(firstVehicle->second).lon = lon[1];
              //std::get<2>(firstVehicle->second).lat = lat[1];
              //std::get<2>(secondVehicle->second).lon = lon[2];
              //std::get<2>(secondVehicle->second).lat = lat[2];
              //            for (auto &participant : retVal) {
              //  inf("4- %d", participant->first);
             // }

            }
            tempsum = distMatrix[0][2] + distMatrix[1][1] + distMatrix[2][0];
            //inf("Tempsum 5: %f + %f + %f = %f", distMatrix[0][2], distMatrix[1][1], distMatrix[2][0], tempsum);
            if( tempsum < minDist) {
              minDist = tempsum;
              //participant->first);
              retVal[0] = thirdVehicle;
              retVal[1] = secondVehicle;
              retVal[2] = firstVehicle;

              //std::get<2>(thirdVehicle->second).lon = lon[0];
              //std::get<2>(thirdVehicle->second).lat = lat[0];
              //std::get<2>(secondVehicle->second).lon = lon[1];
              //std::get<2>(secondVehicle->second).lat = lat[1];
              //std::get<2>(firstVehicle->second).lon = lon[2];
              //std::get<2>(firstVehicle->second).lat = lat[2];
              //            for (auto &participant : retVal) {
              //  inf("5- %d", participant->first);
              //}

            }
            tempsum = distMatrix[0][2] + distMatrix[1][0] + distMatrix[2][1];
            //inf("Tempsum 6: %f + %f + %f = %f", distMatrix[0][2], distMatrix[1][0], distMatrix[2][1], tempsum);
            if( tempsum < minDist) {
              minDist = tempsum;
              //participant->first);
              retVal[0] = secondVehicle;
              retVal[1] = thirdVehicle;
              retVal[2] = firstVehicle;
              //std::get<2>(secondVehicle->second).lon = lon[0];
              //std::get<2>(secondVehicle->second).lat = lat[0];
              //std::get<2>(thirdVehicle->second).lon = lon[1];
              //std::get<2>(thirdVehicle->second).lat = lat[1];
              //std::get<2>(firstVehicle->second).lon = lon[2];
              //std::get<2>(firstVehicle->second).lat = lat[2];
              //            for (auto &participant : retVal) {
              //  inf("6- %d", participant->first);
              //}

            }

            inf("Triple");
          } else {
            debug("No optimizing allocator available, returning in order");
            for (participant_t participant = m_participants.begin();participant != m_participants.end();participant++) {
              retVal.push_back(participant);
            }
          }
          return retVal;
        }

        /// @brief This function tries all allocation combinations, and selects the one minimizing the maximum vehicle distance traveled
        std::vector<participant_t> formationAllocatorMinMax(const std::vector<std::pair<double, double>> pos) {
          std::vector<participant_t> retVal;

          if(pos.size() == 2){
            // Find positions in lat/lon
            double lon[2] = {m_last_rs_msg.lon, m_last_rs_msg.lon};
            double lat[2] = {m_last_rs_msg.lat, m_last_rs_msg.lat};
            for(unsigned i = 0; i<2;i++) {
              DUNE::Coordinates::WGS84::displace(pos[i].second, pos[i].first, &lat[i], &lon[i]);
              inf("%f, %f", lat[i], lon[i]);
            }
            // Calculate distances betwen all vehicles and all formation positions
            double distMatrix[2][2];
            {
              unsigned i=0;
              for (auto &participant : m_participants) {
                for(unsigned j = 0; j<2;j++) {
                  distMatrix[i][j] = DUNE::Coordinates::WGS84::distance((double)std::get<0>(participant.second), std::get<1>(participant.second), 0.0, lat[j], lon[j], 0.0);
                }
                i++;
              }
            }
            // Find distance sum of each solution and select best combination
            participant_t firstVehicle  = m_participants.begin();
            participant_t secondVehicle  = std::next(m_participants.begin());
            double minMax = std::max(distMatrix[0][0], distMatrix[1][1]);
            retVal.push_back(firstVehicle);
            retVal.push_back(secondVehicle);

            if( std::max(distMatrix[0][1],distMatrix[1][0]) < minMax) {
              retVal[0] = secondVehicle;
              retVal[1] = firstVehicle;
            }

          } else if(pos.size() == 3) {
            // Find positions in lat/lon
            double lon[3] = {m_last_rs_msg.lon, m_last_rs_msg.lon, m_last_rs_msg.lon};
            double lat[3] = {m_last_rs_msg.lat, m_last_rs_msg.lat, m_last_rs_msg.lat};
            for(unsigned i = 0; i<3;i++) {
              DUNE::Coordinates::WGS84::displace(pos[i].second, pos[i].first, &lat[i], &lon[i]);
              //inf("%f, %f", Math::Angles::degrees(lat[i]), Math::Angles::degrees(lon[i]));
            }

            // Calculate distances betwen all vehicles and all formation positions
            double distMatrix[3][3];
            {
              unsigned i=0;
              for (auto &participant : m_participants) {
                for(unsigned j = 0; j<3;j++) {
                  distMatrix[i][j] = DUNE::Coordinates::WGS84::distance((double)std::get<0>(participant.second), std::get<1>(participant.second), 0.0, lat[j], lon[j], 0.0);
                  //war("Dist %d, %f for (%d, %d)", participant.first, distMatrix[i][j], i, j);
                }
                i++;
              }
            }
            // Find distance sum of each solution and select best combination
            auto firstVehicle  = m_participants.begin();
            auto secondVehicle  = std::next(firstVehicle);
            auto thirdVehicle  = std::next(secondVehicle) ;

            
            retVal.push_back(firstVehicle);
            retVal.push_back(secondVehicle);
            retVal.push_back(thirdVehicle);

            double minMax = std::max({distMatrix[0][0],distMatrix[1][1],distMatrix[2][2]});

            double tempmax = std::max({distMatrix[0][0], distMatrix[1][2], distMatrix[2][1]});
            if( tempmax < minMax) {
              minMax = tempmax;
              retVal[0] = firstVehicle;
              retVal[1] = thirdVehicle;
              retVal[2] = secondVehicle;

            }
            tempmax = std::max({distMatrix[0][1], distMatrix[1][0], distMatrix[2][2]});
            if( tempmax < minMax) {
              minMax = tempmax;
              retVal[0] = secondVehicle;
              retVal[1] = firstVehicle;
              retVal[2] = thirdVehicle;

            }
            tempmax = std::max({distMatrix[0][1], distMatrix[1][2], distMatrix[2][0]});
            if( tempmax < minMax) {
              minMax = tempmax;
              retVal[0] = thirdVehicle;
              retVal[1] = firstVehicle;
              retVal[2] = secondVehicle;
            }
            tempmax = std::max({distMatrix[0][2], distMatrix[1][1], distMatrix[2][0]});
            if( tempmax < minMax) {
              minMax = tempmax;
              retVal[0] = thirdVehicle;
              retVal[1] = secondVehicle;
              retVal[2] = firstVehicle;
            }
            tempmax = std::max({distMatrix[0][2], distMatrix[1][0], distMatrix[2][1]});
            if( tempmax < minMax) {
              minMax = tempmax;
              retVal[0] = secondVehicle;
              retVal[1] = thirdVehicle;
              retVal[2] = firstVehicle;
            }
            inf("Triple minmax");
          } else {
            debug("No optimizing allocator available, returning in order");
            for (participant_t participant = m_participants.begin();participant != m_participants.end();participant++) {
              retVal.push_back(participant);
            }
          }
          return retVal;
        }


        void updateReference() {
          static bool m_collision = false;
        std::pair<double,double> utmpoint;
        m_con->transformSRID(Math::Angles::degrees(m_last_rs_msg.lon), Math::Angles::degrees(m_last_rs_msg.lat), 4326, utmpoint.first, utmpoint.second, 32632);


          std::vector<std::pair<double, double>> pos;

          static double angle = 0.0;
          double prevangle = angle;
          m_formation_radius = m_last_of_msg.maxradius;
          if(m_useCollisionMitigation && formationCollisionHandler(utmpoint.first, utmpoint.second, m_formation_radius, m_participants.size(), angle)) {
            //if(angle < 0)
            //  angle = 2*M_PI-angle;
            pos = generateTrackingFormation(0,0, m_formation_radius, m_participants.size(), angle); 
            inf("Participants, colrotstate %ld: %f", m_participants.size(), angle);
            if(m_collision==false || m_allocation.empty() || std::abs(prevangle - angle) > M_PI/m_participants.size()) {
              m_allocation = formationAllocator(pos);
            }

              m_collision=true;
              m_formation_rotate_rad = angle;
            
          } else {
            pos = generateTrackingFormation(0,0, m_formation_radius, m_participants.size(), m_formation_rotate_rad);
            inf("Participants, rotstate %ld: %f", m_participants.size(), m_formation_rotate_rad);
            if(m_collision==true || m_allocation.empty()) {
              m_allocation = formationAllocator(pos);
            }
            m_collision=false;
          }


          
          for(auto &p : pos) {
            inf("%f, %f", p.first, p.second);
          }

          // TODO: If angle delta too big, recalculate allocation
          //if(m_allocation.empty()) {
          //  m_allocation = formationAllocator(pos);
          //}
          
          double lat,lon;
          unsigned i=0;
          for (auto &participant : m_allocation) {
            //inf("Vehicle: %d", participant->first);

            lon = m_last_rs_msg.lon;
            lat = m_last_rs_msg.lat;

            DUNE::Coordinates::WGS84::displace(pos[i].second, pos[i].first, &lat, &lon);
            //std::get<2>(participant->second).setDestination(participant->first);
            //std::get<2>(participant->second).speed.set(m_dsp);
            //std::get<2>(participant->second).flags = IMC::Reference::FlagsBits::FLAG_LOCATION | IMC::Reference::FlagsBits::FLAG_SPEED;

            std::get<2>(participant->second).lon = lon;
            std::get<2>(participant->second).lat = lat;
            
            inf("Vehicle %d (%f, %f)", participant->first, std::get<2>(participant->second).lon, std::get<2>(participant->second).lat);
            i++;
          
            dispatch(std::get<2>(participant->second));
          }
        }

        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);


/*/
            std::pair<double,double> m_last_utm_pos_end;
            m_con->transformSRID(Math::Angles::degrees(0.17626705), Math::Angles::degrees(1.10530141), 4326, m_last_utm_pos_end.first, m_last_utm_pos_end.second, 32632);

            if(m_con->findClosestSafePointUTM(m_last_utm_pos_end.first, m_last_utm_pos_end.second)) {
              //inf("Original Pos: %f, %f", desired_path.end_lat, desired_path.end_lon);
              inf("Safe Pos: %f, %f", m_last_utm_pos_end.first, m_last_utm_pos_end.second);
              spew("End Pointinlayer: %d", pointCheck->run(m_last_utm_pos_end.first, m_last_utm_pos_end.second));
            } else {
              err("findClosestSafePoint failed, probably DB error.");
              return;
            }*/
/*/////////////////////////////////////////
double angle, x,y;
x=554942.21,y=7022601.52;
if(rotationChecker(x,y, 60, 3, angle)) {
  inf("Example 1: angle: %f, start: (%f, %f)", angle, x,y);
} else {
  war("Example 1: angle: %f, start: (%f, %f)", angle, x,y);
}
x=556168.40,y=7025252.57;
if(rotationChecker(x,y, 60, 3, angle)) {
  inf("Example 2: angle: %f, start: (%f, %f)", angle, x,y);
} else {
  war("Example 2: angle: %f, start: (%f, %f)", angle, x,y);
}
x=553715.55,y=7023172.75;
if(rotationChecker(x,y, 60, 3, angle)) {
  inf("Example 3: angle: %f, start: (%f, %f)", angle, x,y);
} else {
  war("Example 3: angle: %f, start: (%f, %f)", angle, x,y);
}
x=556077.53,y=7025288.27;
if(rotationChecker(x,y, 60, 3, angle)) {
  inf("Example 4: angle: %f, start: (%f, %f)", angle, x,y);
} else {
  war("Example 4: angle: %f, start: (%f, %f)", angle, x,y);
}

/////////////////////////////////////////*/
            if (isActive())
            {
              if(m_ref_send_timer.overflow()) {
                if(!m_last_tag_timer.overflow())
                {
                  if(m_formation_started) {
                    updateReference(); 
                  }
      
                } else {
                  // Keep searchers alive during timeout
                  war("Position Estimate Too Old, re-sending previous message.");
                  for (auto &participant : m_allocation) {
                    dispatch(std::get<2>(participant->second));
                  }
                  // Alternative: Stop search on timeout
                  //requestDeactivation();
                }
                m_ref_send_timer.reset();
              }
              
            }
          }
        }
      };
    }
  }
}
DUNE_TASK
