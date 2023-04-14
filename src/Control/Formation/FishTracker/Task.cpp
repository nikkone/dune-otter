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
        bool rotate_formation;
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
        //!
        std::vector<uint16_t> participant_ids;
        std::vector<IMC::Reference> participant_refs;

        double m_formation_rotate_radians;
        bool m_rotate_formation;
        double m_formation_rotation_step;

        Task(const std::string &name, Tasks::Context &ctx) : 
        DUNE::Tasks::Task(name, ctx),
        m_formation_rotate_radians(0.0),
        m_rotate_formation(true),
        m_formation_rotation_step(M_PI/2) // 90deg
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

          param("Reference Sending Interval", m_args.rotate_formation)
              .defaultValue("False")
              .description("Period between sync messages");

          param("Reference Sending Interval", m_args.formation_rotation_step)
              .units(Units::Radian)
              .defaultValue("0.0")
              .minimumValue("0.0")
              .maximumValue("6.284") // ~2PI
              .description("Period between sync messages");

          bind<IMC::RemoteSensorInfo>(this);
          bind<IMC::Abort>(this);
          //bind<IMC::TBRFishTag>(this);
        }
        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.fishtag_min_interval))
            m_last_tag_timer.setTop(m_args.fishtag_min_interval);
          if (paramChanged(m_args.ref_send_interval))
            m_ref_send_timer.setTop(m_args.ref_send_interval);
          if (paramChanged(m_args.rotate_formation))
            m_rotate_formation = m_args.rotate_formation;
          if (paramChanged(m_args.formation_rotation_step))
            m_formation_rotation_step = m_args.formation_rotation_step;
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
          // DIRTY TEMP SOLUTION
          participant_ids.push_back(getSystemId());
          participant_ids.push_back(0x2811);
          participant_ids.push_back(0x2812);
          participant_refs.push_back(IMC::Reference());
          participant_refs.push_back(IMC::Reference());
          participant_refs.push_back(IMC::Reference());
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
          man.timeout = 10;

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
          for (const auto participant_id : participant_ids)
          {
            dispatchToVehicle(participant_id, &startPlan);
          }
          m_last_tag_timer.reset();
          m_ref_send_timer.reset();
          m_formation_rotate_radians = 0;
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
          for (const auto participant_id : participant_ids)
          {
            dispatchToVehicle(participant_id, &stopPlan);
          }
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        }

        void consume(const IMC::RemoteSensorInfo *msg)
        {
          // TODO: Filter
          m_last_rs_msg = *msg;
          m_last_tag_timer.reset();
          if(!isActive()) {
            requestActivation();
          }
          if(m_rotate_formation) {
            m_formation_rotate_radians += m_formation_rotation_step;
          }
        }

        void consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;
          
          war(DTR("Abort detected. Disabling FishSearch control."));
          requestDeactivation();
        }

        std::vector<std::pair<double, double>> generateRegularPolygonVertices(double x0, double y0, double radius, int numVertices, int rotAngle)
        {
          std::vector<std::pair<double, double>> retval;
          double angle = 2 * M_PI / numVertices; // Calculate the angle between each vertex
          for (int i = 0; i < numVertices; i++)
          {
            double x = x0 + radius * std::cos(i * angle + rotAngle); // Calculate the x-coordinate of the vertex
            double y = y0 + radius * std::sin(i * angle + rotAngle); // Calculate the y-coordinate of the vertex
            //std::cout << "(" << x << "," << y << ")" << std::endl; // Output the vertex coordinates
            retval.push_back(std::pair<double, double>(x, y));
          }
          return retval;
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

        void updateReference() {
          DUNE::IMC::DesiredSpeed m_dsp;
          m_dsp.value = 1.0;
          m_dsp.speed_units = IMC::SUNITS_METERS_PS;
          std::vector<std::pair<double, double>> pos = generateTrackingFormation(0,0, 60, participant_refs.size(), m_formation_rotate_radians);
          inf("Participants: %ld", participant_refs.size());
          double lat,lon;
          for (unsigned i = 0;i<participant_refs.size();i++)
          {
            lon = m_last_rs_msg.lon;
            lat = m_last_rs_msg.lat;

            DUNE::Coordinates::WGS84::displace(pos[i].first, pos[i].second, &lat, &lon);
            participant_refs[i].setDestination(participant_ids[i]);
            participant_refs[i].lon = lon;
            participant_refs[i].lat = lat;
            spew("%d - %f, %f", i, DUNE::Math::Angles::degrees(participant_refs[i].lon), DUNE::Math::Angles::degrees(participant_refs[i].lat));
            participant_refs[i].speed.set(m_dsp);
            participant_refs[i].flags = IMC::Reference::FlagsBits::FLAG_LOCATION | IMC::Reference::FlagsBits::FLAG_SPEED;
            dispatch(participant_refs[i]);
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
                  spew("Ref Update");
                  updateReference();       
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
