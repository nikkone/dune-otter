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
  //! TODO: Add drift
  //! TODO: Add reading from file to have moving tag
  //!
  //! Insert explanation on task behaviour here.
  //! @author Nikolai Lauvås
  namespace FishTag
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Activation depth.
      double depth;
      //! Initial position (degrees)
      std::vector<double> position;
      //! PRNG type.
      std::string prng_type;
      //! PRNG seed.
      int prng_seed;
      //! Mean temperature value.
      float mean_value;
      //! Standard deviation of temperature measurements.
      double std_dev;
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
    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! PRNG handle
      Random::Generator* m_prng;
      //! Task arguments.
      Arguments m_args;
      //! Current Lat and Lon of vehicle.
      fp64_t m_current_lat, m_current_lon;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        param("Initial Position", m_args.position)
        .units(Units::Degree)
        .size(2)
        .description("Initial tag position lat lon");

        param("Initial Depth", m_args.depth)
        .units(Units::Meter)
        .minimumValue("0.0")
        .maximumValue("1000.0")
        .defaultValue("0.20")
        .description("Initial tag depth");

        param("Time Disturbance Standard deviation", m_args.std_dev)
        .description("Standard deviation of produced temperature")
        .units(Units::Second)
        .defaultValue("0.01");

        param("Time Disturbance PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        param("Time Disturbance PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        param("Time Disturbance Mean value", m_args.mean_value)
        .description("Mean value of disturbance")
        .units(Units::Second)
        .defaultValue("0.0");

        param("Time Offset", m_args.mean_value)
        .description("Offset to subtract from timestamp. Can simulate constant processing/receiving time.")
        .units(Units::Second)
        .defaultValue("0.0");

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


        param("Use GPS Position", m_args.use_gps)
        .defaultValue("false");



        bind<IMC::GpsFix>(this);
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
        m_prng = Random::Factory::create(m_args.prng_type,
                                         m_args.prng_seed);
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
        Memory::clear(m_prng);
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->getSource() != getSystemId())
          return;
        m_current_lat=msg->lat;
        m_current_lon=msg->lon;
      }
      //! Main loop.
      void
      task(void)
      {
        IMC::TBRFishTag::TransmitProtocolEnum trans_protocol = IMC::TBRFishTag::TBR_S256;
        // Calculate

        double dist = DUNE::Coordinates::WGS84::distance(Math::Angles::radians(m_args.position[0]), Math::Angles::radians(m_args.position[1]), m_args.depth, m_current_lat, m_current_lon, 0.0);
        
        double t=dist/1485; //1485=Speed of sound in water

        // newtime=unixtimestamp + estimated propagation time
        std::chrono::milliseconds newtime = std::chrono::seconds(std::time(nullptr)) + std::chrono::milliseconds(static_cast<int>(std::round(t*1000)));

        int unix_timestamp = std::chrono::duration_cast<std::chrono::seconds>(newtime).count();

        int millis = newtime.count()-std::chrono::duration_cast<std::chrono::seconds>(newtime).count()*1000;
        //std::cout << std::time(nullptr)<< std::endl;

        int SNR =50-dist*5/100;


        
        inf("Timestamp: %i - %i dist: %f - traveltime: %f", unix_timestamp,millis,dist, t);
        IMC::TBRFishTag tag_msg;
        tag_msg.serial_no = m_args.serial_no;
        tag_msg.unix_timestamp = unix_timestamp;
        tag_msg.millis = millis;
        tag_msg.trans_protocol = trans_protocol;
        tag_msg.trans_id = m_args.trans_id;
        tag_msg.trans_data = m_args.trans_data;
        tag_msg.snr = SNR;
        tag_msg.trans_freq = m_args.trans_freq;
        tag_msg.recv_mem_addr = m_args.recv_mem_addr;
        if(m_args.use_gps) {
          tag_msg.lat = m_current_lat;
          tag_msg.lon = m_current_lon;
        } else {
          tag_msg.lat = Math::Angles::radians(m_args.position[0]);
          tag_msg.lon = Math::Angles::radians(m_args.position[1]);
        }

        dispatch(tag_msg);
      }
    };
  }
}

DUNE_TASK
