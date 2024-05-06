//***************************************************************************
// Copyright 2013-2022 Norwegian University of Science and Technology (NTNU)*
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
// Vendor headers
#include <FishTagEstimators/DUNETagBuffer.hpp>

namespace Simulators
{
  //! If only two vehicles receive fishTags, this task creates a simulated third one.
  //! @author Nikolai Lauvås
  namespace FishTagAdditional
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Receiver ID
      uint32_t serial_no;
      //! Receiver memory address
      uint16_t recv_mem_addr;
      //! SNR values under this will not be transmitted/detected
      int SNR_detection_limit;
      //! Should the Speed of Sound in water be updated from measurement
      bool update_c_sound;
      //! Entity providing the Speed of Sound in water
      std::string entity_c_sound;
      //! IMC address of sources to consider for synthetic tag detections
      std::vector<uint16_t> participants;
      //! After receiving last detectionof tag, wait this long to delete the buffer.
      uint32_t bufferTimeout;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double a_fit;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double B_fit;
      //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
      double k_fit;
      //! Initial Speed of Sound in water
      float init_c_sound;
      //! Enable/disable sending of synthetic transmissions
      bool enabledState;
    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments
      Arguments m_args;
      //! Buffer for storing recent tag detections
      FishTagEstimators::DUNETagBuffers_t tagBuffers;
      //! FishTag that is sent
      IMC::TBRFishTag m_tag_msg;
      //! Timer for time after reception. On timeout, a synthetic measurment is created and sent
      Time::Counter<float> m_timeout_timer;
      //! Data structure for holding the most recent positions of each participant. Key is IMCsource ID, value is position pair is (Lat,Lon) in radians.
      std::map<uint16_t, std::pair<fp32_t, fp32_t>> m_positions;
      //! Data structure that keeps track of all encountered tracks.
      std::map<uint32_t, Time::Counter<float>> m_tag_timer;
      //! First is serialNo, second is IMCsrc
      std::map<uint32_t, uint16_t> serialNoToIMCsrc;
      //! Data structure for holding the most recent piositions for the fish. Key is recv_id, value is position pair is (Lat,Lon) in radians.
      std::map<uint32_t, std::pair<double, double>> m_fishPositions;
      //! Speed of sound provider entity label.
      int m_c_sound_eid;
      //! Speed of sound in water given as mps/1000
      double m_soundSpeed_kmps;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        param("Receiver Serial", m_args.serial_no)
        .defaultValue("10101010");

        param("Receiver Memory Location", m_args.recv_mem_addr)
        .defaultValue("1001");

        param("Participants", m_args.participants)
        .description("IMC source id of devices to monitor")
        .defaultValue("10256,10257,10258");
// SNR
        param("Minimum SNR", m_args.SNR_detection_limit)
        .description("Mean value of disturbance")
        .defaultValue("10.0");

        param("SNR linear a", m_args.a_fit)
        .description("Using a linear model ax+b=SNR, this is the 'a' coefficient")
        .defaultValue("0.015309");

        param("SNR linear b", m_args.B_fit)
        .description("Using a linear model ax+b=SNR, this is the 'b' coefficient ")
        .defaultValue("49.807");

        param("SNR logarithmic K", m_args.k_fit)
        .description("Using a linear model ax+b=SNR, this is the 'b' coefficient ")
        .defaultValue("4.9147");

        param("Timestamp Timeout [s]", m_args.bufferTimeout)
        .description("Maximum time [s] to keep an estimator alive without any detections received.")
        .units(Units::Second)
        .defaultValue("15");

        param("Initial Speed Of Sound", m_args.init_c_sound)
        .units(Units::MeterPerSecond)
        .description("The ID of the tracked fish tag")
        .defaultValue("1485.0");

        param("Use Speed Of Sound Measurement", m_args.update_c_sound)
        .description("If speed of sound is provided through IMC messages.")
        .defaultValue("false");

         param("Speed Of Sound - Entity", m_args.entity_c_sound)
        .description("The entity delivering the Speed of Sound in water")
        .defaultValue("CTD");

         param("SendEnabled", m_args.enabledState)
        .description("Enable/Disable sending")
        .defaultValue("true");

        bind<IMC::Announce>(this);
        bind<IMC::EstimatedState>(this);
        bind<IMC::RemoteSensorInfo>(this);
        bind<IMC::SoundSpeed>(this);
        bind<IMC::TBRFishTag>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.participants)) {
          for(auto participant : m_args.participants) {
            if(m_positions.find(participant) == m_positions.end()) {
              m_positions[participant] = std::pair<fp32_t, fp32_t>(0.0,0.0);
            }
          }
        }
        if(paramChanged(m_args.init_c_sound)) {
          m_soundSpeed_kmps = m_args.init_c_sound/1000;
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
        try
          {
            m_c_sound_eid = resolveEntity(m_args.entity_c_sound);
          }
        catch (...)
          {
            if(m_args.update_c_sound) {
              err("Could not find entity: %s", m_args.entity_c_sound.c_str());
            }
            m_c_sound_eid = 0;
          }
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
      onResourceRelease(void) {
        clearDUNETagBuffers_t(&tagBuffers);
      }

      // Generated with Claude 3 OPUS AI
      uint32_t extractNumbers(const std::string& str) {
          uint32_t numbers = 0;
          bool foundNumbers = false;
          
          for (char c : str) {
              if (std::isdigit(c)) {
                  foundNumbers = true;
                  numbers = numbers * 10 + (c - '0');
              } else if (foundNumbers) {
                  break;
              }
          }
          
          return numbers;
      }

      void
      consume(const IMC::Announce* msg)
      {
        if(m_positions.find(msg->getSource()) != m_positions.end()) {
          m_positions[msg->getSource()].first = msg->lat;
          m_positions[msg->getSource()].second = msg->lon;
        }
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        if(m_positions.find(msg->getSource()) != m_positions.end()) {
          m_positions[msg->getSource()].first = msg->lat;
          m_positions[msg->getSource()].second = msg->lon;
        }
      }

      void
      consume(const IMC::RemoteSensorInfo* msg)
      {
        if(m_positions.find(msg->getSource()) != m_positions.end()) {
          // Decode Estimator name
          // Add to map over most recent position estimates
          uint32_t tagID;
          if((tagID = extractNumbers(msg->id)) != 0) {
            m_fishPositions[tagID] = std::pair<double, double>(msg->lat, msg->lon);
            spew("Added tag to pos buffer: %u", tagID);
          } else {
            err("Could not decode : %s", (msg->id).c_str());
          }
        }
      }

      void
      consume(const IMC::SoundSpeed* msg)
      {
        if(msg->getSourceEntity() == m_c_sound_eid) {
          if(m_args.update_c_sound) {
            m_soundSpeed_kmps = msg->value/1000;
          }
        }
      }

      void
      consume(const IMC::TBRFishTag* msg)
      {
        // Filter for only procesing tags from participating vehicles
        if(m_positions.find(msg->getSource()) != m_positions.end()) {
          spew("Received tag ID%u from src: %u", msg->trans_id, msg->getSource());
          // Action taken on first reception of a transmitter ID
          if(tagBuffers.find(msg->trans_id) == tagBuffers.end()) {
          // New transmitter found, create buffer
          tagBuffers[msg->trans_id] = new FishTagEstimators::DUNETagBuffer(msg->trans_id, m_positions.size(), 0.2); // Depth conversion is dontcare here
          // Set NED frame used on specific tag to location of first tag location
          double ref[] = {msg->lat, msg->lon, 0.0};
          tagBuffers[msg->trans_id]->setReferenceCoordinateRad(ref);
          tagBuffers[msg->trans_id]->setTimestampTimeoutLimit(m_args.bufferTimeout);
          spew("Created buffer for receiver %u", msg->serial_no);
          }
        // Action taken for all receptions: Add to buffer.
        if(tagBuffers[msg->trans_id]->addTagDetection(msg)) {
          inf("Recent Detections: %u, timestamp: %lu, timestep: %lu", tagBuffers[msg->trans_id]->getNoOfMostRecentdetections(), tagBuffers[msg->trans_id]->getLatestTimestamp(), tagBuffers[msg->trans_id]->getLatestTimeStep());
          spew("Receivers in buffer: %lu", tagBuffers[msg->trans_id]->size());
          spew("Detection from receiver %u added to buffer storing tag ID %u.", msg->serial_no, msg->trans_id);
        } else {
          err("Could not add tag to buffer");
        }
        serialNoToIMCsrc[msg->serial_no] = msg->getSource();
        }
      }

      bool transmittSyntheticDetection(const FishTagEstimators::DUNETagBuffer* buffer) {
        if(serialNoToIMCsrc.size() == m_positions.size()) {
          for(auto participant : serialNoToIMCsrc) {
            // Check if a participant has a buffer or not
            if((buffer->tagBuffer).find(participant.first) == (buffer->tagBuffer).end()) {
              war("Dispatching missing detection for : %u", participant.second);
              try{
// Prepare TBRFishTag message
              auto referenceDetection = ((buffer->tagBuffer).cbegin()->second)->cbegin();
              m_tag_msg.serial_no = m_args.serial_no; // Set to the mission value
              m_tag_msg.trans_protocol = referenceDetection->trans_protocol;
              m_tag_msg.trans_id = referenceDetection->trans_id;
              m_tag_msg.trans_data = referenceDetection->trans_data;
              m_tag_msg.trans_freq = referenceDetection->trans_freq;
              m_tag_msg.recv_mem_addr = m_args.recv_mem_addr;
              m_tag_msg.lat = m_positions.at(participant.second).first;
              m_tag_msg.lon = m_positions.at(participant.second).second;
// Calculate distance
              double refdist = DUNE::Coordinates::WGS84::distance(referenceDetection->lat, referenceDetection->lon, 0.2,
                            m_fishPositions[m_tag_msg.trans_id].first, m_fishPositions[m_tag_msg.trans_id].second, 0.2); // Does not consider depth
              double dist = DUNE::Coordinates::WGS84::distance(m_tag_msg.lat, m_tag_msg.lon, 0.2,
                            m_fishPositions[m_tag_msg.trans_id].first, m_fishPositions[m_tag_msg.trans_id].second, 0.2); // Does not consider depth
// Location and timestamp dependent simulated timestamp
              double timeDiff_ms = (refdist-dist)/m_soundSpeed_kmps; // In milliseconds
              long int referenceReceiverToA_ms = (long int)referenceDetection->unix_timestamp*1000 + (long int)referenceDetection->millis;
              long int synthTOA_ms = referenceReceiverToA_ms - timeDiff_ms;
              m_tag_msg.unix_timestamp = synthTOA_ms/1000;
              m_tag_msg.millis = synthTOA_ms%1000;
// Location and model dependent SNR
              m_tag_msg.snr = m_args.B_fit - m_args.k_fit*std::log(dist) - m_args.a_fit*dist;
              if(m_tag_msg.snr > m_args.SNR_detection_limit) {
                if(m_args.enabledState) {
                  dispatch(m_tag_msg);
                }
              }
              return true; // Limits it to only one
              } catch(...) {
                err("Something went wrong with calculating synth tag");
                return false;
              }
            }
          }
          war("All received, doing nothing.");
        } else {
          war("serialNoToIMCsrc incomplete, doing nothing.");
        }
        return false;
      }

      void
      task(void)
      {   
        // Check for timed out buffers
        std::vector<uint32_t> buffersToDelete;
        for(const auto buffer : tagBuffers) {
          if(!(buffer.second)->checkTimeout(DUNE::Time::Clock::getSinceEpoch())) {
            spew("Tag %d timed out.", buffer.first);
            buffersToDelete.push_back(buffer.first);
          }
        }

        // Deleting done in separate loop to not mess up iteration through map.
        for(auto &id : buffersToDelete) {
          auto it = tagBuffers.find(id);
          if(it != tagBuffers.end()) {
            delete it->second;
            tagBuffers.erase(id);
          }
        }

        // Remaining timestamps has not timed out
        for(const auto buffer : tagBuffers) {
          if((buffer.second)->size() == 2) {
            transmittSyntheticDetection(buffer.second);
          }
        }
      } // End of task function
    };
  }
}

DUNE_TASK
