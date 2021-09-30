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
// written agreement between you and the Department of Engineering          *
// Cybernetics at the Norwegian University of Science and Technology        *
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
  //! 
  //! @author Nikolai Lauvås
  namespace FishTracking
  {
    using DUNE_NAMESPACES;

    //! Enumeration tracking states
    enum state_t {C_INACTIVE, C_TRANSIT, C_SEARCHING, C_ASSEMBLING_TRACKING_FORMATION, C_TRACKING_SINGLE_VEHICLE, C_TRACKING_FORMATION};
    struct Arguments
    {
      //! Toggle
      float toggled_time;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        
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
        void runFSM(state_t &state) {
            switch(state) {
                case C_TRANSIT:
                    // Find plan, perform plan
                    // If no plan found, retry x times, then go to INACTIVE while giving an error message
                    // If TagDetection, then
                    // If goal reached, go to C_SEARCHING
                    break;
                case C_SEARCHING:
                    // Get seraching assignment execute x times
                    // If Tag detection on any vessel, go to C_ASSEMBLING_TRACKING_FORMATION
                    // If search finished, either loiter, station keeping or return to base.
                    break;
                case C_ASSEMBLING_TRACKING_FORMATION:
                    // If tag detection on this vessel, go to C_TRACKING_SINGLE_VEHICLE
                    // If Tag detection on other vessel and not on this, get assigned location and enter C_TRANSIT
                    break;
                case C_TRACKING_SINGLE_VEHICLE:
                    break;
                case C_TRACKING_FORMATION:
                    break;
                case C_INACTIVE:
                    break;
                default:
                    err("FSM error, state should not be reached");
            }
        }
      //! Main loop.
      void
      onMain(void)
      {

      }
    };
  }
}

DUNE_TASK
