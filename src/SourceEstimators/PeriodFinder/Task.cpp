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
#include <boost/circular_buffer.hpp>

#define LOGFTOILE 1
namespace SourceEstimators
{
  //! Task that runs source position estimation algorithms for IMC::TBRFishTag
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
      //!
      FishTagEstimators::DUNETagBuffers_t tagBuffers;

      std::string m_startupTimestamp;

      boost::circular_buffer<uint8_t> m_intervals;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;

      size_t m_previous_pos;

      bool m_interval_valid;

      uint16_t m_expected_interval;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx), 
        m_intervals(5),
        m_previous_pos(0),
        m_interval_valid(false),
        m_expected_interval(0)
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
        uint64_t  prevTimestamp = tagBuffers[msg->trans_id]->getLatestTimestamp(); //ms
        // Action taken for all receptions: Add to buffer and run measurment update on estimators.
        if(tagBuffers[msg->trans_id]->addTagDetection(msg)) {
          //inf("Recent Detections for tagID%u: %u, timestamp: %lu", msg->trans_id, tagBuffers[msg->trans_id]->getNoOfMostRecentdetections(), tagBuffers[msg->trans_id]->getLatestTimestamp());
          uint16_t interval = (uint16_t)std::round((float)(tagBuffers[msg->trans_id]->getLatestTimestamp() - prevTimestamp)/1000);
          if(interval > 1) {
            if(interval <= 90 && interval >= 30) {

             if((interval != m_expected_interval) && m_interval_valid) { // Unexpected, check if it's next
                int nextNumber = 0;
                int sum = 0;
                size_t temp_pos = m_previous_pos;
                for (int i = 0; i < 7; ++i) {
                    nextNumber = findNextNumber(intervals, temp_pos);
                    sum += nextNumber;
                    m_intervals.push_back(nextNumber);
                    war("Lost: %d", nextNumber);
                    //std::cout << "Lost: " << nextNumber << std::endl;
                    if(sum >= interval) {
                      break;
                    }
                }
                if(sum==interval) {
                  m_previous_pos = temp_pos;
                  m_expected_interval = findNextNumber(intervals, temp_pos);
                  //inf("Expected next %d", m_expected_interval);
                } else {
                  war("Could not find match. Sum: %u, Interval: %u", sum, interval);
                  m_intervals.clear();
                }
              return;
             }

              //inf("New interval: %d", interval);
              m_intervals.push_back(interval);
              std::string currentIntervals;
              for(auto inter : m_intervals) {
                currentIntervals += std::to_string(inter) + ",";// + inter;//std::string(inter);
              }
              currentIntervals.pop_back();

              size_t pos = 0;
              size_t prev_pos = 0;
              bool found = false;
              int nextInterval = 0;
              while ((pos = intervals.find(currentIntervals, pos)) != std::string::npos) {
                  spew("Found at position: %lu", pos);
                  
                  prev_pos = (pos + currentIntervals.length())%intervals.length();
                  pos += currentIntervals.length();
                  found=true;
              }
              if(!found) {
                pos = 0;
                prev_pos = 0;
                std::string intervalsR = shiftString(intervals);
                while ((pos = intervalsR.find(currentIntervals, pos)) != std::string::npos) {
                    spew("Found at position: %lu", pos);
                    
                    prev_pos = (pos + currentIntervals.length())%intervals.length();
                    pos += currentIntervals.length();
                    found=true;
                }
                // TODO: set m_previous_pos
                nextInterval = findNextNumber(intervalsR, prev_pos);
              } else {
                m_previous_pos = prev_pos;
                nextInterval = findNextNumber(intervals, prev_pos);
              }
              if(!found) {
                err("Could not find substring in intervals: %s", currentIntervals.c_str());
                
              }

              if(m_interval_valid) {
                if(interval == m_expected_interval) {
                  inf("Got: %d, Expected: %d, Next %d", interval, m_expected_interval, nextInterval);
                } else {
                  err("Got: %d, Expected: %d, Next %d", interval, m_expected_interval, nextInterval);
                }
              } else {
                inf("Next %d", nextInterval);
                m_interval_valid = true; // TODO: Make invalid if multiple possible
              }
              m_expected_interval = nextInterval;
            } else {
              if(m_interval_valid) {
                int nextNumber = 0;
                int sum = 0;
                size_t temp_pos = m_previous_pos;
                for (int i = 0; i < 7; ++i) {
                    nextNumber = findNextNumber(intervals, temp_pos);
                    sum += nextNumber;
                    m_intervals.push_back(nextNumber);
                    war("Lost: %d", nextNumber);
                    //std::cout << "Lost: " << nextNumber << std::endl;
                    if(sum >= interval) {
                      break;
                    }
                }
                if(sum==interval) {
                  m_previous_pos = temp_pos;
                  m_expected_interval = findNextNumber(intervals, temp_pos);
                  //inf("Expected next %d", m_expected_interval);
                } else {
                  war("Could not find match. Sum: %u, Interval: %u", sum, interval);
                  m_intervals.clear();
                }
              }
            }
          }
        }
      }
/* ChatGPT
Write cpp code takes a std::string with comma separated numbers shifts it according to the closest comma to center so that what comes after this point is now at the beginning, and what comes before this point is now at the end
*/
      std::string shiftString(const std::string& input) {
          // Find the closest comma to the center
          size_t mid = input.size() / 2;
          size_t closestCommaIndex = input.find_last_of(',', mid);

          // If no comma is found before the midpoint, consider the midpoint itself
          if (closestCommaIndex == std::string::npos)
              closestCommaIndex = mid;

          // Create a new string with the portion after the closest comma followed by the portion before it
          std::string shiftedString = input.substr(closestCommaIndex + 1) + "," + 
                                      input.substr(0, closestCommaIndex);

          return shiftedString;
      }

/* ChatGPT
In cpp, I have a large std::string that contains comma separated numbers. I want to create a function that is able to get a position in the large string and find the next number. If the end is reached, then the next number from the start should be returned
*/
      int findNextNumber(const std::string& input, size_t& position) {
          size_t startPos = position;
          size_t length = input.length();
          
          // Skip any non-digit characters
          while (startPos < length && !isdigit(input[startPos])) {
              startPos++;
          }
          if (startPos >= length) {
            startPos = 0;
            while (startPos < length && !isdigit(input[startPos])) {
                startPos++;
            }
          }

          
          size_t endPos = startPos;
          // Find the end of the number
          while (endPos < length && isdigit(input[endPos])) {
              endPos++;
          }
          
          // If the end of the string is reached, wrap around
          if (endPos > length) {
              endPos = 0;
              while (endPos < position && isdigit(input[endPos])) {
                  endPos++;
              }
          }
          
          // Extract the number
          int result =0;
          try{
            result = std::stoi(input.substr(startPos, endPos - startPos));
          } catch(...) {
            err("Not int: %s", input.substr(startPos, endPos - startPos).c_str());
          }
          
          // Update position
          position = endPos;
          
          return result;
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