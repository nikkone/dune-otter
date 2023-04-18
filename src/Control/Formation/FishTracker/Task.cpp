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
      };

      struct Task : public DUNE::Tasks::Task
      {
        Arguments m_args;
        //! Timer.
        Time::Counter<float> m_last_tag_timer;
        //! Timer.
        Time::Counter<float> m_ref_send_timer;
        //!
        IMC::RemoteSensorInfo m_last_rs_msg;

        IMC::otterFormation m_last_of_msg;
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
        double m_formation_radius;
        //! vehicleid - Lat(rad), Lon(rad), LastReference, time since last position update
        std::map<uint16_t, std::tuple<fp64_t, fp64_t, IMC::Reference, DUNE::Time::Delta>> m_participants; 

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

          bind<IMC::Abort>(this);
          bind<IMC::Announce>(this);
          bind<IMC::otterFormation>(this);
          bind<IMC::RemoteSensorInfo>(this);
          //bind<IMC::TBRFishTag>(this);
        }
        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if(paramChanged(m_args.fishtag_min_interval))
            m_last_tag_timer.setTop(m_args.fishtag_min_interval);
          if(paramChanged(m_args.ref_send_interval))
            m_ref_send_timer.setTop(m_args.ref_send_interval);
          if(paramChanged(m_args.formation_rotation_step))
            m_formation_rotation_step_rad = m_args.formation_rotation_step;
          if(paramChanged(m_args.fishtag_min_interval))
            m_minTagInterval =  m_args.fishtag_min_interval;
          if(paramChanged(m_args.ref_send_interval))
            m_FollowRefInterval = m_args.ref_send_interval;
        }
        
        void
        onResourceAcquisition(void)
        {
        }

        void
        onResourceRelease(void)
        {
        }

        void
        onResourceInitialization(void)
        {

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
            if(msg->getSource() != getSystemId()) {
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
        }

        void consume(const IMC::otterFormation* msg) {
          inf("Got otterFormation following %s", msg->target.c_str());
          
          switch(msg->msg_type) {
            case IMC::otterFormation::MessageTypeEnum::T_start:
              m_last_of_msg = *msg;
              m_formation_radius = msg->minradius;
              if(!isActive()) {

                // Add vehicles
                std::vector<std::string> parts;
                String::split(msg->participants, ",", parts);
                m_participants.clear();
                for (const auto &participant_name : parts) {
                  spew("Vehicle added: %d", resolveSystemName(participant_name));
                  m_participants[resolveSystemName(participant_name)] = std::tuple<fp64_t, fp64_t, IMC::Reference, DUNE::Time::Delta>{0.0,0.0, IMC::Reference(),DUNE::Time::Delta()};
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
              parseCustomParameters(msg->custom);
              m_formation_radius = msg->minradius;
              //TODO;
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

        std::vector<std::pair<double, double>> generateTrackingFormation(double x0, double y0, double radius, unsigned numVertices, double rotAngle)
        {
          std::vector<std::pair<double, double>> retval;
          double angle = 2 * M_PI / numVertices; // Calculate the angle between each vertex
          if(numVertices%2) { // Odd number of searchers, regular polygon can be used
          spew("Odd");
            for (unsigned i = 0; i < numVertices; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              retval.push_back(std::pair<double, double>(x, y));
            }
          } else { // Even number of searchers, must skew half of the regular polygon
           spew("Even");
            for (unsigned i = 0; i < numVertices/2; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              spew("Ri: %d, x: %f, y: %f", i,x,y);
              retval.push_back(std::pair<double, double>(x, y));
            }
            rotAngle -= angle/2;
            for (unsigned i = numVertices/2; i < numVertices; i++)
            {
              double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
              double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
              //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
              spew("Si: %d, x: %f, y: %f", i,x,y);
              retval.push_back(std::pair<double, double>(x, y));
            }
          }

          return retval;
        }
        // TODO
        void formationAllocator() {

        }

        void updateReference() {
          DUNE::IMC::DesiredSpeed m_dsp;
          m_dsp.value = m_last_of_msg.maxspeed;
          m_dsp.speed_units = m_last_of_msg.speed_units;
          std::vector<std::pair<double, double>> pos = generateTrackingFormation(0,0, m_formation_radius, m_participants.size(), m_formation_rotate_rad);
          inf("Participants, rotstate %ld: %f", m_participants.size(), m_formation_rotate_rad);
          double lat,lon;
          unsigned i=0;
          for (auto &participant : m_participants) {
            lon = m_last_rs_msg.lon;
            lat = m_last_rs_msg.lat;

            DUNE::Coordinates::WGS84::displace(pos[i].first, pos[i].second, &lat, &lon);
            std::get<2>(participant.second).setDestination(participant.first);
            std::get<2>(participant.second).lon = lon;
            std::get<2>(participant.second).lat = lat;
            std::get<2>(participant.second).speed.set(m_dsp);
            std::get<2>(participant.second).flags = IMC::Reference::FlagsBits::FLAG_LOCATION | IMC::Reference::FlagsBits::FLAG_SPEED;

            spew("%d - %f, %f", i, DUNE::Math::Angles::degrees(std::get<2>(participant.second).lon), DUNE::Math::Angles::degrees(std::get<2>(participant.second).lat));
            dispatch(std::get<2>(participant.second));

            inf("Participant: %d, Location: %f, %f", participant.first, std::get<0>(participant.second), std::get<1>(participant.second));
            i++;
          }
        }

        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);
            if (isActive())
            {
              if(m_ref_send_timer.overflow()) {
                if(!m_last_tag_timer.overflow())
                {
                  if(m_formation_started) {
                    spew("Ref Update");
                    updateReference(); 
                  }
      
                } else {
                  // Also threashold to keep standing still
                  // Check if over threashold, then requestDeactivation
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
