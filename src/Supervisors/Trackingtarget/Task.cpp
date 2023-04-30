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

namespace Supervisors
{
  //! This task automatically starts the formation tracking for given estimators
  //! @author Nikolai Lauvås
  namespace Trackingtarget
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Sync Period;
      double timeout;
      //! Vector with the names of all estimators to consider folowing
      std::vector<std::string> monitoredEstimators;

      //! Minimum Speed.
      fp32_t minspeed;
      //! Maximum Speed.
      fp32_t maxspeed;
      //! Minimum Radius.
      fp32_t minradius;
      //! Maximum Radius.
      fp32_t maxradius;
      //! Formation Participants.
      std::string participants;
      //! Custom settings for formation.
      std::string custom;

      bool enableToggle;

      bool keepFormation;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
        //! Timer that is reset when a RemoteSensorInfo for the following entity is received. Top is m_minTagInterval
        Time::Counter<float> m_last_tag_timer;
        //!
        IMC::RemoteSensorInfo m_last_rs_msg;

        std::set<std::string> m_monitoredEstimators;

        IMC::otterFormation m_of_msg;

        bool m_formation_started;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_formation_started(false)
      {
        param("Formation Timeout", m_args.timeout)
            .units(Units::Second)
            .defaultValue("600.0")
            .minimumValue("0.0")
            .description("Period after which the tracker considers a fish tag lost");

        param("SetActive", m_args.monitoredEstimators)
        .defaultValue("MultiReceiverEKF36,Fish_position_est_1")
        .description("Activate sending.");

        param("Minimum Speed", m_args.minspeed)
        .units(Units::MeterPerSecond)
        .defaultValue("0.5")
        .description("Minimum Speed");

        param("Maximum Speed", m_args.maxspeed)
        .units(Units::MeterPerSecond)
        .defaultValue("1")
        .description("Maximum Speed.");

        param("Minimum Radius", m_args.minradius)
        .units(Units::Meter)
        .defaultValue("30")
        .description("Minimum Radius");

        param("Maximum Radius", m_args.maxradius)
        .units(Units::Meter)
        .defaultValue("60")
        .description("Maximum Radius.");

        param("Participants", m_args.participants)
        .defaultValue("ntnu-otter-02,ntnu-otter-01,ntnu-otter-03")
        .description("Participants.");

        param("Custom Arguments", m_args.custom)
        .defaultValue("r=0.0;i=30.0;x=90.0;t=60.0;f=5.0;")
        .description("Custom Arguments");


        param("Enable toggle", m_args.enableToggle)
        .defaultValue("true")
        .description("Activate sending.");

        param("Timeout Formation Keep", m_args.keepFormation)
        .defaultValue("true")
        .description("Keep Formation After Timeout, else formation is stopped");


        // Initialize entity state.
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        bind<IMC::RemoteSensorInfo>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
          if(paramChanged(m_args.timeout)) 
            m_last_tag_timer.setTop(m_args.timeout);

      if(paramChanged(m_args.minspeed))
        m_of_msg.minspeed = m_args.minspeed; 
      if(paramChanged(m_args.maxspeed))
        m_of_msg.maxspeed = m_args.maxspeed; 
      if(paramChanged(m_args.minradius))
        m_of_msg.minradius = m_args.minradius; 
      if(paramChanged(m_args.maxradius))
        m_of_msg.maxradius = m_args.maxradius; 
      if(paramChanged(m_args.participants))
        m_of_msg.participants = m_args.participants; 
      if(paramChanged(m_args.custom))
        m_of_msg.custom = m_args.custom; 



          if(paramChanged(m_args.monitoredEstimators)) {
            m_monitoredEstimators.clear();
            for(auto &estimate : m_args.monitoredEstimators) {
              m_monitoredEstimators.insert(estimate);
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

      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_of_msg.speed_units = IMC::SUNITS_METERS_PS;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

      }

      void sendFormationStart() {
        m_of_msg.msg_type = IMC::otterFormation::MessageTypeEnum::T_start;
        m_formation_started = true;
        dispatch(m_of_msg);
      }

      void sendFormationStop() {
        m_of_msg.msg_type = IMC::otterFormation::MessageTypeEnum::T_stop;
        m_formation_started = false;
        dispatch(m_of_msg);
      }

      void consume(const IMC::RemoteSensorInfo *msg)
      {
        spew("Tag found");
        if(m_args.enableToggle) {
          spew("Enabled");
          if(m_monitoredEstimators.find(msg->id) != m_monitoredEstimators.end()) {
              spew("Monitored tag found");
              if(m_args.keepFormation && m_formation_started && m_last_tag_timer.overflow()) {
                sendFormationStop();
              }
              m_last_rs_msg = *msg;
              m_last_tag_timer.reset();
              if(!m_formation_started) {
                m_of_msg.target = msg->id;
                sendFormationStart();
              }
          }
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
          if(m_formation_started && !m_args.keepFormation && m_last_tag_timer.overflow()) {
            sendFormationStop();
          }
        }
      }
    };
  }
}

DUNE_TASK
