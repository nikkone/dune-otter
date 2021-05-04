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

namespace Tuning
{
  //! This task demonstrates how OMPL is used in DUNE along with ENCGIS
  //! @author Nikolai Lauvås
  namespace Autopilot
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! 
      double desiredHeading;
      //! 
      double desiredMPS;
      //! 
      bool setHeading;
      //!
      bool setMPS;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      IMC::DesiredHeading m_heading;
      IMC::DesiredSpeed m_speed;
      IMC::ControlLoops m_control;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("DesiredHeading", m_args.desiredHeading)
        .defaultValue("0.0")
        .description("Value set");

        param("DesiredMPS", m_args.desiredMPS)
        .defaultValue("0.0")
        .description("Value set");

        param("SetHeading", m_args.setHeading)
        .defaultValue("false")
        .description("Activate sending");

        param("SetMPS", m_args.setMPS)
        .defaultValue("false")
        .description("Activate sending");
          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        m_speed.speed_units = IMC::SUNITS_METERS_PS;
        bind<IMC::Abort>(this);
      }

        void
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          // This works as redundancy, in case everything else fails
          reset();
          debug("disabling");
        }

      void reset() {
          m_control.enable = DUNE::IMC::ControlLoops::CL_DISABLE;
          m_control.scope_ref +=1;
          m_control.mask = IMC::CL_YAW;
          dispatch(m_control);
          m_control.enable = DUNE::IMC::ControlLoops::CL_DISABLE;
          m_control.scope_ref +=1;
          m_control.mask = IMC::CL_SPEED;
          dispatch(m_control);
      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {

        if(m_args.setHeading) {
          m_control.enable = DUNE::IMC::ControlLoops::CL_ENABLE;
          m_control.mask = IMC::CL_YAW;
          m_control.scope_ref +=1;
          m_heading.value = m_args.desiredHeading;
          dispatch(m_heading);
          dispatch(m_control);
        } else {
          m_control.enable = DUNE::IMC::ControlLoops::CL_DISABLE;
          m_control.scope_ref +=1;
          m_control.mask = IMC::CL_YAW;
          dispatch(m_control);
        }

        if(m_args.setMPS) {
          m_control.enable = DUNE::IMC::ControlLoops::CL_ENABLE;
          m_control.mask = IMC::CL_SPEED;
          m_control.scope_ref +=1;
          m_speed.value = m_args.desiredMPS;
          dispatch(m_speed);
          dispatch(m_control);
        } else {
          m_speed.value = 0;
          dispatch(m_speed);
          m_control.enable = DUNE::IMC::ControlLoops::CL_DISABLE;
          m_control.scope_ref +=1;
          m_control.mask = IMC::CL_SPEED;
          dispatch(m_control);
        }
        if(!m_args.setMPS && !m_args.setHeading) {
          inf("reactivate path");
          m_control.enable = DUNE::IMC::ControlLoops::CL_ENABLE;
          m_control.mask = IMC::CL_PATH;
          m_control.scope_ref +=1;
          dispatch(m_control);
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

      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

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
