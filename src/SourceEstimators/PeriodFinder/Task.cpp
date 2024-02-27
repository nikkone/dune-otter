//***************************************************************************
// Copyright 2013-2024 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
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
#include <FishTagEstimators/DUNETagBuffer.hpp>
#include <memory>
#include <FishTagEstimators/PeriodFinder.hpp>
#define LOGFTOILE 1
namespace SourceEstimators
{
  //! Task that finds the transmission period of an IMC::TBRFishTag based on the known series of intervals.
  //! Note: Currently only supports single transmitter
  //! @author Nikolai Lauvås
  namespace PeriodFinder
  {
    using DUNE_NAMESPACES;
    const std::string intervals = "67,84,66,80,58,49,53,80,57,36,52,41,55,89,88,55,77,69,41,85,37,79,53,44,89,64,88,90,45,66,65,43,59,82,46,44,32,84,34,46,87,73,74,66,31,56,47,83,35,68,67,31,86,73,30,47,47,49,74,61,71,49,71,41,41,83,34,49,84,75,46,30,80,55,49,65,36,89,88,53,79,47,37,35,31,68,70,87,40,54,79,49,70,35,62,35,83,67,89,34";
    struct Arguments
    {
      //! Time to wait in while. In practice, this controls how regular the filter timing is
      float message_wait_time;
      //! Period between when filter is run
      float filter_timestep;
      //! Reset toggle for buffers and estimators
      bool reset_toggle;
      //! How long should a buffer be considered active before being deactivated
      uint16_t timestampTimeout;
      //! Factor to multiply tag data with to get depth in meters
      float depthConversion;
      //! Wait this long untill updating position filter (To avoid processing only the two first when three are available)
      float communicationWaitSec;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Datastructure to hold task arguments/parameters
      Arguments m_args;
      //! Container that stores the tag transmissions that have been received
      FishTagEstimators::DUNETagBuffers_t tagBuffers;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;
      //! Used to find bugs
      std::vector<std::unique_ptr<FishTagEstimators::PeriodFinder>> m_periodFinders; 
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Filter Timestep", m_args.filter_timestep)
        .description("The timestep of the filter")
        .units(Units::Second)
        .defaultValue("7.0");

        param("Reset Toggle", m_args.reset_toggle)
        .description("When changed, resets buffers and estimators.")
        .defaultValue("false");

        param("Timestamp Timeout [s]", m_args.timestampTimeout)
        .description("Maximum time [s] to keep an estimator alive without any detections received.")
        .units(Units::Second)
        .defaultValue("500");

        param("Message Wait Time", m_args.message_wait_time)
        .description("The time to wait for new messages in the while loop between checking timer.")
        .units(Units::Second)
        .defaultValue("0.01");
        bind<IMC::TBRFishTag>(this);

      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.filter_timestep))
          m_filter_timer.setTop(m_args.filter_timestep);
        if(paramChanged(m_args.reset_toggle)) {
          clearDUNETagBuffers_t(&tagBuffers);
        }
      }


      void
      onResourceAcquisition(void)
      {
      }
      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Each unique transmitter ID gets its own buffer, which in turn stores it in separate buffers according to receiver serials.
      void
      consume(const IMC::TBRFishTag* msg)
      {
        // Action taken on first reception of a transmitter ID: Add estimators, configure and initialize logfile
        if(tagBuffers.find(msg->trans_id) == tagBuffers.end()) {
          // New transmitter found, create buffer
          tagBuffers[msg->trans_id] = new FishTagEstimators::DUNETagBuffer(msg->trans_id, 5, m_args.depthConversion);
          // Set NED frame used on specific tag to location of first tag location
          double ref[] = {msg->lat, msg->lon, 0.0};
          tagBuffers[msg->trans_id]->setReferenceCoordinateRad(ref);
          tagBuffers[msg->trans_id]->setTimestampTimeoutLimit(m_args.timestampTimeout);
          spew("Created buffer for receiver %u", msg->serial_no);
        }
        uint64_t prevTimestamp_ms = tagBuffers[msg->trans_id]->getLatestTimestamp();
        // Add to buffer containing all tags
        if(!tagBuffers[msg->trans_id]->addTagDetection(msg)) {
            err("Failed to add tag to buffer");
            return;
        }

        if(tagBuffers[msg->trans_id]->getNoOfMostRecentdetections() > 1) {
          return;
        }

        uint16_t currentInterval = (uint16_t)std::round((float)(tagBuffers[msg->trans_id]->getLatestTimestamp() - prevTimestamp_ms)/1000);

        if(m_periodFinders.size() < 1000) {
        std::unique_ptr<FishTagEstimators::PeriodFinder> finder1 = std::make_unique<FishTagEstimators::PeriodFinder>(intervals, 30, 90, 4);
        m_periodFinders.push_back(std::move(finder1));
        }
        int i = 0;
        int sucesses = 0;
        static int totalSucesses = 0;
        int misses = 0;
        static int totalMisses = 0;
        for(auto& it : m_periodFinders) {
          uint16_t expected = it->getExpectedInterval();
          it->addInterval(currentInterval);

          switch(it->checkValidity()) {
            case FishTagEstimators::PeriodFinder::IntervalValidity::Invalid:
                war("PeriodEst #%d: Expected: %d, Got: %d, Next: %d. Invalid", i, expected, currentInterval, it->getExpectedInterval());
                ++misses;
                break;
            case FishTagEstimators::PeriodFinder::IntervalValidity::Valid:
                //inf("PeriodEst #%d: Expected: %d, Got: %d, Next: %d", i, expected, currentInterval, it->getExpectedInterval());
                ++sucesses;
                break;
            case FishTagEstimators::PeriodFinder::IntervalValidity::LowestEstimate:
                war("PeriodEst #%d: Expected: %d, Got: %d, Guesstimate: %d", i, expected, currentInterval, it->getExpectedInterval());
                ++misses;
                break;
            default:
                err("Unknown validity state");
                break;
          }
          i++;
        }
        totalSucesses += sucesses;
        totalMisses += misses;

        inf("Sucesses: %d, totalSucesses: %d, Misses: %d, totalMisses: %d", sucesses, totalSucesses, misses, totalMisses);
      }

      void
      onResourceRelease(void) {
        clearDUNETagBuffers_t(&tagBuffers);
      }

      void
      onResourceInitialization(void)
      {
        m_filter_timer.setTop(m_args.filter_timestep);
      }


      void
      onMain(void)
      {
        while(!stopping()) {
          if(m_filter_timer.overflow()) {
            m_filter_timer.reset();
          }
          waitForMessages(m_args.message_wait_time);
        }
      }
    };
  }
}

DUNE_TASK