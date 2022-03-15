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
#include <FishTagEstimators/EstimatorMap.hpp>
#include <FishTagEstimators/DUNETagBuffer.hpp>


#define LOGFTOILE 1
namespace SourceEstimators
{
  //! Task that runs source position estimation algorithms for IMC::TBRFishTag
  //! @author Nikolai Lauvås
  namespace FishTag
  {
    using DUNE_NAMESPACES;

    static const unsigned c_states = 3;

    struct Arguments
    {
      //! Time to wait in while. In practice, this controls how regular the filter timing is
      float message_wait_time;
      //! Period between when filter is run
      float filter_timestep;
      //! Initial Speed of Sound in water
      float init_c_sound;
      //! Location and prefix of logfiles
      std::string log_folder_and_prefix;
      //! Should the Speed of Sound in water be updated from measurement
      bool update_c_sound;
      //! Entity providing the Speed of Sound in water
      std::string entity_c_sound;
// Kalman Filter
      //! Extended Kalman filter - Qm
      std::vector<double> ekf_Qm;      
      //! Extended Kalman filter - P0
      std::vector<double> ekf_P0;
      //! Extended Kalman filter - x0
      std::vector<double> ekf_x0;
      //! Time of Arrival Covariance
      double rr_cov;
      //! Depth measurement Covariance
      double rz_cov;
      //! Maximum allowed time [ms] shift between receivers' messages
      double max_time_shift_ms;
// Single receiver estimator arguments
      uint32_t ss_serial_no;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Datastructure to hold task arguments/parameters
      Arguments m_args;
      //!
      FishTagEstimators::DUNETagBuffers_t tagBuffers;
      //!
      FishTagEstimators::EstimatorMap m_emap;
      std::vector<FishTagEstimators::EstimatorMap::estimatorTypeEnum_t> SingleReceiverEstimatorTypeToUse;
      
      //! Speed of sound provider entity label.
      int m_c_sound_eid;
      //! Current Speed of sound in water
      float m_c_sound;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Filter Timestep", m_args.filter_timestep)
        .description("The timestep of the filter")
        .units(Units::Second)
        .defaultValue("7.0");

        param("Initial Speed Of Sound", m_args.init_c_sound)
        .units(Units::MeterPerSecond)
        .description("The ID of the tracked fish tag")
        .defaultValue("1485.0");

        param("Use Speed Of Sound Measurement", m_args.update_c_sound)
        .description("If speed of sound is provided through IMC messages.")
        .defaultValue("false");

         param("Speed Of Sound - Entity", m_args.entity_c_sound)
        .units(Units::MeterPerSecond)
        .description("The entity delivering the Speed of Sound in water")
        .defaultValue("CTD");

        param("Message Wait Time", m_args.message_wait_time)
        .description("The time to wait for new messages in the while loop between checking timer.")
        .units(Units::Second)
        .defaultValue("0.01");

        param("Log Folder and Prefix", m_args.log_folder_and_prefix)
        .description("")
        .defaultValue("log/predict-"); 
// Kalman Filter Parameters
        param("x0", m_args.ekf_x0)
        .size(c_states)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("0.0, 0.0, 0.0");

        param("P0", m_args.ekf_P0)
        .size(c_states*c_states)
        .description("Initial P matrix value for the extended Kalman filter, first row")
        .defaultValue("1, 0, 0, 0, 1, 0, 0, 0, 1.0}");

        param("Qm", m_args.ekf_Qm)
        .size(c_states*c_states)
        .description("Process noise covariance matrix for the extended Kalman filter")
        .defaultValue("1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1");
        
        param("ToA Cov", m_args.rr_cov)
        .description("Time of Arrival Covariance")
        .defaultValue("0");

        param("Depth Cov", m_args.rz_cov)
        .description("Depth measurement Covariance")
        .defaultValue("0");

// Single Source parameters
        param("SS - Max time shift [ms]", m_args.max_time_shift_ms)
        .description("Maximum allowed time [ms] shift between receivers' messages")
        .defaultValue("500");

        param("SS - Receiver Serial number", m_args.ss_serial_no)
        .description("Receiver to use for Single receiver estimators. 0 takes value from first received message.")
        .defaultValue("0");

        bind<IMC::TBRFishTag>(this);
        bind<IMC::SoundSpeed>(this);

      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.filter_timestep))
          m_filter_timer.setTop(m_args.filter_timestep);
        if(paramChanged(m_args.init_c_sound)) {
          m_c_sound = m_args.init_c_sound;
          m_emap.setSoundSpeed(m_c_sound);
        }
      }
      void
      onResourceAcquisition(void)
      {
        SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverEKF);
        SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverUKF);
        SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverSRUKF);
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

      //! Each unique transmitter ID gets its own buffer, which in turn stores it in separate buffers according to receiver serials.
      void
      consume(const IMC::TBRFishTag* msg)
      {
        // Action taken on first reception of a transmitter ID: Add estimators, configure and initialize logfile
        if(tagBuffers.find(msg->trans_id) == tagBuffers.end()) {
          // New transmitter found, create buffer
          tagBuffers[msg->trans_id] = new FishTagEstimators::DUNETagBuffer(msg->trans_id, 5);
          // Set NED frame used on specific tag to location of first tag location
          double ref[] = {msg->lat, msg->lon, 0.0};
          tagBuffers[msg->trans_id]->setReferenceCoordinateRad(ref);
          spew("Created buffer for receiver %u", msg->serial_no);
          // Configure Estimators
          for(auto it = SingleReceiverEstimatorTypeToUse.begin();it !=SingleReceiverEstimatorTypeToUse.end();it++) {
            FishTagEstimators::Estimator* est = m_emap.addEstimator(msg->trans_id,*it);
            est->trans_id = msg->trans_id;
            est->setSoundSpeed(m_c_sound);
            est->setAllowedTimeShift(m_args.max_time_shift_ms);
            est->setTDOACovariance(m_args.rr_cov);
            est->setDepthCovariance(m_args.rz_cov);
            est->initialize(
              Eigen::Matrix3d::Identity(),
              Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_Qm.data()),
              Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_P0.data()),
              Eigen::Map<Eigen::Matrix<double, c_states, 1> >(m_args.ekf_x0.data())
            );
            if(m_args.ss_serial_no == 0) {
              est->setParameter("receiver", msg->serial_no);
            } else {
              est->setParameter("receiver", m_args.ss_serial_no);
            }
            
            est->setParameter("receiver_depth", -0.5);
            est->setParameter("max_jitter", 0.01);
            est->setParameter("max_updates_per_new_measurement", 1);
            est->setParameter("max_correction_attempts", 0);
            est->setParameter("interval_mode", 1);
            //est->setParameter("tag_period", 10.0);
            // Create/clear csv logfile for estimator with header
            std::ofstream logOutStream;
            logOutStream.open(m_args.log_folder_and_prefix + est->name + std::to_string(est->trans_id) + ".csv", std::ofstream::out | std::ofstream::trunc);
            if (logOutStream.good()) {
                logOutStream << "timestamp,N,E,D,Lat,Lon" << std::endl;
                logOutStream.close();
            }
          }
        }
        // Action taken for all receptions: Add to buffer and run measurment update on estimators.
        if(tagBuffers[msg->trans_id]->addTagDetection(msg)) {
          m_emap.updateAll(msg->trans_id, tagBuffers[msg->trans_id]);
          spew("Detection from receiver %u added to buffer storing tag ID %u.", msg->serial_no, msg->trans_id);
        }
      }

      void
      consume(const IMC::SoundSpeed* msg)
      {
        if(msg->getSourceEntity() == m_c_sound_eid) {
          if(m_args.update_c_sound) {
            m_c_sound = msg->value;
            m_emap.setSoundSpeed(m_c_sound);
            spew("Setting c_sound to: %f", msg->value);
          }
        }
      }

      void
      onResourceRelease(void) {
        for (FishTagEstimators::DUNETagBuffers_t::iterator it = tagBuffers.begin(); it != tagBuffers.end(); it++)
        {
          Memory::clear(it->second);
          spew("Cleared buffer for tag %u", it->first);
        }
        tagBuffers.clear();
      }

      void
      onResourceInitialization(void)
      {
        m_filter_timer.setTop(m_args.filter_timestep);
      }

      //! Function for logging to an external file and dispatching the result as IMC
      //! @param [in] result The result to be logged. This is a NED value
      //! @param [in] in_logfilename Filename of file written to
      //! @param [in] logname Name used for the ID in the dispatched IMC::RemoteSensorInfo
      void logResult(FishTagEstimators::Estimator* est, const std::string &in_logfilename, const std::string &logname) {
        double lati,longi;
        std::tuple<double, double, double>  estimate = est->getEstimate();
        double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
        double latLon[3] = {0,0,0};
        if(tagBuffers.find(est->trans_id) != tagBuffers.end()) {
          tagBuffers[est->trans_id]->fromNEDframe(result, latLon);
          lati=latLon[0], longi=latLon[1];

          // Send output to Neptus/DUNE log
          IMC::RemoteSensorInfo tagPosition;
          tagPosition.lat = lati;
          tagPosition.lon = longi;
          tagPosition.alt = -result[2];
          tagPosition.data = std::to_string(result[0]) + std::to_string(result[1]) + "," + std::to_string(result[2]);
          tagPosition.id = logname + std::to_string(est->trans_id);
          dispatch(tagPosition);

          // External Logfile
          #if LOGFTOILE
          std::ofstream logOutStream;
          logOutStream.open(in_logfilename, std::fstream::app);
          if (logOutStream.good()) {
            logOutStream.precision(15);
              logOutStream << Clock::getSinceEpochMsec() << "," << result[0] << "," << result[1] << "," << result[2] << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
              logOutStream.close();
          }   
          #endif
          spew("%s :New Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", logname.c_str(), result[0], result[1], result[2],DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
        } else {
          err("Transmitter ID: %u not in buffer for estimator %s.", est->trans_id, est->name.c_str());
        }
      }

      void
      onMain(void)
      {
        while(!stopping()) {
          if(m_filter_timer.overflow()) {
            m_filter_timer.reset();
            m_emap.predictAll();
            for (FishTagEstimators::EstimatorMap::EstimatorMap_t::iterator it = m_emap.estimatorMap.begin(); it != m_emap.estimatorMap.end(); it++)
            {
              if (it->second != NULL)
              {
                for(FishTagEstimators::EstimatorMap::EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
                  if(( *est)->isActive()) {
                    logResult((*est), m_args.log_folder_and_prefix + (*est)->name + std::to_string((*est)->trans_id) + ".csv", (*est)->name);
                  }
                }
              }
            }
          }
          waitForMessages(m_args.message_wait_time);
        }
      }
    };
  }
}

DUNE_TASK
