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


#include <chrono>
namespace Simulators
{
  //! TODO: Add reading positions from file to have moving tag
  //! TODO: Add movement models such as random walk, Levy flight etc.
  //!
  //! Simulates a fish tag by creating dispatching TBRFishTag messages with correct signal travel time between a given vehicle and the sensor.
  //! The vehicle is taken as the position of the tag, which can be received by the receiver at given distances.
  //! @author Nikolai Lauvås
  namespace FishTag
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Activation depth.
      double receiver_depth;
      //! Tag depth.
      double tag_depth;
      //! Initial receiver position (degrees)
      std::vector<double> receiver_position;
      //! Initial tag position (degrees)
      std::vector<double> tag_position;
      //! PRNG type.
      std::string time_prng_type;
      //! PRNG seed.
      int time_prng_seed;
      //! Mean temperature value.
      float time_mean_value;
      //! Standard deviation of temperature measurements.
      double time_std_dev;
      //! PRNG type
      std::string depth_prng_type;
      //! PRNG seed.
      int depth_prng_seed;
      //! Mean temperature value.
      float depth_mean_value;
      //! Standard deviation of temperature measurements.
      double depth_std_dev;
      //! PRNG type
      std::string position_prng_type;
      //! PRNG seed.
      int position_prng_seed;
      //! Mean temperature value.
      float position_mean_value;
      //! Standard deviation of temperature measurements.
      double position_std_dev;
      //! Fixed Time offset
      int time_offset_s;
      //! Fixed Time offset
      int time_offset_ms;
      //! Receiver ID
      uint32_t serial_no;
      //! TransmitterID
      uint32_t trans_id;
      //! Transmitter Data
      uint16_t trans_data;
      //! TransmittedFreq
      uint8_t trans_freq;
      //! Receiver memory address
      uint16_t recv_mem_addr;
      //! If GPS fix should be set as received location
      bool use_gps;
      //! Should RemoteSensorInfo be sent as well?
      bool sendRemoteSensorInfo;
      //! SNR values under this will not be transmitted/detected
      int SNR_detection_limit;
      //! Using a linear model ax+b=SNR, this is the 'a' coefficient 
      double SNR_linear_a;
      //! Using a linear model ax+b=SNR, this is the 'b' coefficient 
      double SNR_linear_b;
      //! Source Address to use GPS information from.
      std::string GPS_src;
      //! Source Entity to use GPS information from.
      std::string GPS_src_ent;
    };
    struct Task: public DUNE::Tasks::Periodic
    {
      IMC::RemoteSensorInfo tagPosition;
      IMC::TBRFishTag tag_msg;
      //! PRNG handle
      Random::Generator* m_time_prng;
      Random::Generator* m_position_prng;
      Random::Generator* m_depth_prng;

      uint16_t m_GPS_src;
      //! Source Entity to use GPS information from.
      uint8_t m_GPS_src_ent;

      //! Task arguments.
      Arguments m_args;
      //! Current Lat and Lon of vehicle.
      fp64_t m_current_lat, m_current_lon;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_time_prng(NULL),
        m_position_prng(NULL),
        m_depth_prng(NULL)
      {
        param("Receiver Serial", m_args.serial_no)
        .defaultValue("47");

        param("Transmitter Data", m_args.trans_data)
        .defaultValue("5");

        param("Transmitter Frequency", m_args.trans_freq)
        .defaultValue("67");

        param("Transmitter ID", m_args.trans_id)
        .defaultValue("40");

        param("Receiver Memory Location", m_args.recv_mem_addr)
        .defaultValue("1001");

        param("Send RemoteSensorInfo", m_args.sendRemoteSensorInfo)
        .defaultValue("false");

// Time Disturbance
        param("Time Disturbance PRNG Type", m_args.time_prng_type)
        .defaultValue(Random::Factory::c_default);

        param("Time Disturbance PRNG Seed", m_args.time_prng_seed)
        .defaultValue("-1");

        param("Time Disturbance Mean value", m_args.time_mean_value)
        .description("Mean value of disturbance")
        .units(Units::Millisecond)
        .defaultValue("0.0");

        param("Time Disturbance Standard deviation", m_args.time_std_dev)
        .description("Standard deviation of produced temperature")
        .units(Units::Millisecond)
        .defaultValue("0.00");
        
        param("Time Offset S", m_args.time_offset_s)
        .description("Offset to subtract from timestamp. Can simulate constant processing/receiving time.")
        .units(Units::Second)
        .defaultValue("0");

        param("Time Offset MS", m_args.time_offset_ms)
        .description("Offset to subtract from timestamp. Can simulate constant processing/receiving time.")
        .units(Units::Millisecond)
        .defaultValue("0");
// Position
        param("Initial Receiver Position", m_args.receiver_position)
        .units(Units::Degree)
        .size(2)
        .defaultValue("0.0, 0.0")
        .description("Initial tag position lat lon");

        param("Use GPS Position Receiver", m_args.use_gps)
        .defaultValue("true");

        param("GPS Source Address", m_args.GPS_src)
        .defaultValue("-1");

        param("GPS Source Entity", m_args.GPS_src_ent)
        .defaultValue("-1");

        param("Initial Tag Position", m_args.tag_position)
        .units(Units::Degree)
        .size(2)
        .description("Initial tag position lat lon");

        param("Position PRNG Type", m_args.position_prng_type)
        .defaultValue(Random::Factory::c_default);

        param("Position PRNG Seed", m_args.position_prng_seed)
        .defaultValue("-1");

        param("Position Mean value", m_args.position_mean_value)
        .description("Mean value of disturbance")
        .units(Units::Second)
        .defaultValue("0.0");

        param("Position Standard deviation", m_args.position_std_dev)
        .description("Standard deviation of produced temperature")
        .units(Units::Second)
        .defaultValue("0.0");

// Depth
        param("Initial Receiver Depth", m_args.receiver_depth)
        .units(Units::Meter)
        .minimumValue("0.0")
        .maximumValue("1000.0")
        .defaultValue("0.20")
        .description("Initial receiver depth");

        param("Initial Tag Depth", m_args.tag_depth)
        .units(Units::Meter)
        .minimumValue("0.0")
        .maximumValue("1000.0")
        .defaultValue("0.20")
        .description("Initial tag depth");

        param("Tag Depth PRNG Type", m_args.depth_prng_type)
        .defaultValue(Random::Factory::c_default);

        param("Tag Depth PRNG Seed", m_args.depth_prng_seed)
        .defaultValue("-1");

        param("Tag Depth Mean value", m_args.depth_mean_value)
        .description("Mean value of disturbance")
        .units(Units::Second)
        .defaultValue("0.0");

        param("Tag Depth Standard deviation", m_args.depth_std_dev)
        .description("Standard deviation of produced temperature")
        .units(Units::Second)
        .defaultValue("0.0");
// SNR
        param("Minimum SNR", m_args.SNR_detection_limit)
        .description("Mean value of disturbance")
        .defaultValue("0.0");

        param("SNR linear a", m_args.SNR_linear_a)
        .description("Using a linear model ax+b=SNR, this is the 'a' coefficient")
        .defaultValue("-0.05");

        param("SNR linear b", m_args.SNR_linear_b)
        .description("Using a linear model ax+b=SNR, this is the 'b' coefficient ")
        .defaultValue("50");

        bind<IMC::GpsFix>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(m_args.GPS_src == "-1") {
          m_GPS_src = getSystemId();
        } else {
          m_GPS_src = resolveSystemName(m_args.GPS_src);
        }
        if(m_args.GPS_src_ent == "-1") {
          m_GPS_src_ent = getEntityId();
        } else {
          m_GPS_src_ent = resolveEntity(m_args.GPS_src_ent);
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
        m_time_prng = Random::Factory::create(m_args.time_prng_type,
                                         m_args.time_prng_seed);
        m_position_prng = Random::Factory::create(m_args.position_prng_type,
                                         m_args.position_prng_seed);
        m_depth_prng = Random::Factory::create(m_args.depth_prng_type,
                                         m_args.depth_prng_seed);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        //IMC::TBRFishTag::TransmitProtocolEnum trans_protocol = IMC::TBRFishTag::TBR_S256;
        tag_msg.trans_protocol = IMC::TBRFishTag::TBR_S256;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_time_prng);
        Memory::clear(m_position_prng);
        Memory::clear(m_depth_prng);
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->getSource() != m_GPS_src)
          return;
        if(m_args.GPS_src_ent != "-1") {
          if (msg->getSourceEntity() != m_GPS_src_ent)
            return;
        }  
        
        m_current_lat=msg->lat;
        m_current_lon=msg->lon;
      }
      //! Main loop.
      void
      task(void)
      {
        // Calculate
        fp64_t receiver_lat, receiver_lon;

        if(m_args.use_gps) {
          receiver_lat = m_current_lat;
          receiver_lon = m_current_lon;
        } else {
          receiver_lat = Math::Angles::radians(m_args.receiver_position[0]);
          receiver_lon = Math::Angles::radians(m_args.receiver_position[1]);
        }
        tag_msg.lat = Math::Angles::radians(m_args.tag_position[0]) + m_position_prng->gaussian(m_args.position_mean_value, m_args.position_std_dev);
        tag_msg.lon = Math::Angles::radians(m_args.tag_position[1]) + m_position_prng->gaussian(m_args.position_mean_value, m_args.position_std_dev);

        double tag_depth = m_args.tag_depth + m_position_prng->gaussian(m_args.depth_mean_value, m_args.depth_std_dev);

        double dist = DUNE::Coordinates::WGS84::distance(receiver_lat, receiver_lon, m_args.receiver_depth, tag_msg.lat, tag_msg.lon, tag_depth);
        double SNR = dist*m_args.SNR_linear_a + m_args.SNR_linear_b;
        spew("SNR: %lf, dist %f", SNR, dist);
        if(SNR > m_args.SNR_detection_limit) {
          double t=dist/1485; //1485=Speed of sound in water

          std::chrono::milliseconds newtime = std::chrono::seconds(std::time(nullptr))
                                            + std::chrono::milliseconds(static_cast<int>(std::round(t*1000)))
                                            + std::chrono::milliseconds(static_cast<int>(m_time_prng->gaussian(m_args.time_mean_value, m_args.time_std_dev)));

          int unix_timestamp = std::chrono::duration_cast<std::chrono::seconds>(newtime).count() + m_args.time_offset_s;
          int millis = newtime.count()-std::chrono::duration_cast<std::chrono::seconds>(newtime).count()*1000 + m_args.time_offset_ms;

          


          
          inf("Timestamp: %i - %i dist: %f - traveltime: %f", unix_timestamp,millis,dist, t);
          tag_msg.serial_no = m_args.serial_no;
          tag_msg.unix_timestamp = unix_timestamp;
          tag_msg.millis = millis;
          tag_msg.trans_id = m_args.trans_id;
          tag_msg.trans_data = tag_depth/0.392;
          tag_msg.snr = SNR;
          tag_msg.trans_freq = m_args.trans_freq;
          tag_msg.recv_mem_addr = m_args.recv_mem_addr;

          dispatch(tag_msg);
          if(m_args.sendRemoteSensorInfo) {
            tagPosition.lat = tag_msg.lat;
            tagPosition.lon = tag_msg.lon;
            tagPosition.alt = tag_msg.trans_data;
            tagPosition.id = std::to_string(m_args.serial_no) + " - " + std::to_string(m_args.trans_id);
            tagPosition.data = SNR;
            dispatch(tagPosition);
          }
          
        //inf("%lf",m_time_prng->gaussian(m_args.time_mean_value, m_args.time_std_dev));
        }
      }
    };
  }
}

DUNE_TASK
