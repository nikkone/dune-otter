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

namespace MotionPlanners
{
  //! Simple task to test PlanSpec sending.
  //! @author Nikolai Lauvås
  namespace SpecSender
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Defines the bounds of the area the path planner operates on.
      std::vector<double> planningBounds;
      //! Defines the start and end point to use while developing
      std::vector<double> startAndEnd;

      fp32_t speed;
    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments.
      Arguments m_args;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        param("Planning Bounds", m_args.planningBounds)
        .size(4)
        .defaultValue("10.369549, 63.407093, 10.426469, 63.463678")
        .description("Define the area searched for a solution (minLat, minLon, maxLat, maxLon)");

        param("Start and Goal", m_args.startAndEnd)
        .size(4)
        .defaultValue("10.38627, 63.44540, 10.38903, 63.41434")
        .description("A starting point and end point to use while developing");   

        param("Speed", m_args.speed)
        .defaultValue("1.0")
        .description("");   

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
      task(void)
      {

          IMC::PlanProbSpec spec;
          spec.setDestination(getSystemId());
          spec.vehicle = 0x2810;
          spec.problem_type = IMC::PlanProbSpec::TypeEnum::TBR_fpath;
          spec.start_lat = m_args.startAndEnd[1];
          spec.start_lon = m_args.startAndEnd[0];
          spec.end_lat = m_args.startAndEnd[3];
          spec.end_lon = m_args.startAndEnd[2];
          spec.speed = m_args.speed;
          spec.speed_units = IMC::SUNITS_METERS_PS;
          spec.custom = "TEST";


          IMC::PolygonVertex south_west;
          south_west.lat = m_args.planningBounds[1];
          south_west.lon = m_args.planningBounds[0];
          IMC::PolygonVertex north_east;
          north_east.lat = m_args.planningBounds[3];
          north_east.lon = m_args.planningBounds[2];
          spec.area.push_back(south_west);
          spec.area.push_back(north_east);

          dispatch(spec);
          waitForMessages(0.1);
      }
    };
  }
}

DUNE_TASK
