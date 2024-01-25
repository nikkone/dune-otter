//***************************************************************************
// Copyright 2013-2022 Norwegian University of Science and Technology (NTNU)*
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
 
/* TODO:
Better way to get estimator custom parameters to task arguments
Check depth estimates not seeming right
Check single-receiver estimators bool output, may be wrongly assigned
Update parameters to estimators online, such as covariance
Move more parameters to custom/common interface
Max/min tag period as estimator parameter
Fix large number period being set at first regular
Run some estimators only when x amounts after the first reception to allow for transmission from all connected transmitters.
*/
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

      //! Reset toggle for buffers and estimators
      bool reset_toggle;

      std::vector<std::string> singleEstimators;
      std::vector<std::string> multiEstimators;

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
      //!
      std::vector<std::string> ss_extra_param_name;
      //!
      std::vector<double> ss_extra_param_value;

      uint32_t timestampTimeout;
      //! Factor to multiply tag data with to get depth in meters
      float depthConversion;
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
      std::vector<FishTagEstimators::EstimatorMap::estimatorTypeEnum_t> MultiReceiverEstimatorTypeToUse;
      FishTagEstimators::tagBool_t newData;
      bool m_ss_valid;
      //! Speed of sound provider entity label.
      int m_c_sound_eid;
      //! Current Speed of sound in water
      float m_c_sound;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;

      std::string m_startupTimestamp;
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_ss_valid(false)
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
        .description("The entity delivering the Speed of Sound in water")
        .defaultValue("CTD");

        param("Message Wait Time", m_args.message_wait_time)
        .description("The time to wait for new messages in the while loop between checking timer.")
        .units(Units::Second)
        .defaultValue("0.01");

        param("Log Folder and Prefix", m_args.log_folder_and_prefix)
        .description("")
        .defaultValue("log/predict-"); 

        param("Reset Toggle", m_args.reset_toggle)
        .description("When changed, resets buffers and estimators.")
        .defaultValue("false");

// Kalman Filter Parameters
        param("x0", m_args.ekf_x0)
        .size(c_states)
        .description("Initial X value for the extended Kalman filter (Should not be 0,0,0, as this may give division by zero in estimators.)")
        .defaultValue("1.0, 1.0, 1.0");

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

        param("SS - Extra Parameters - Name", m_args.ss_extra_param_name)
        .description("Receiver to use for Single receiver estimators. 0 takes value from first received message.")
        .defaultValue("receiver_depth,max_jitter,max_updates_per_new_measurement,max_correction_attempts,interval_mode,tag_period");
        param("SS - Extra Parameters - Value", m_args.ss_extra_param_value)
        .description("Receiver to use for Single receiver estimators. 0 takes value from first received message.")
        .defaultValue("-0.5,0.01,1,0,1,7");

        param("Single Receiver Estimators", m_args.singleEstimators)
        .description("What single-receiver estimators to activate")
        .defaultValue("EKF, UKF, SRUKF");
        
        param("Multi Receiver Estimators", m_args.multiEstimators)
        .description("What multi-receiver estimators to activate")
        .defaultValue("XKF,EKF");

        param("Timestamp Timeout [s]", m_args.timestampTimeout)
        .description("Maximum time [s] to keep an estimator alive without any detections received.")
        .defaultValue("500");

        param("Depth Coefficient", m_args.depthConversion)
        .description("The coefficient used to convert the data field of a tag to depth in meters")
        .defaultValue("0.2");

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
        if(paramChanged(m_args.reset_toggle)) {
          m_emap.clear();
          clearDUNETagBuffers_t(&tagBuffers);
        }
        if(paramChanged(m_args.ss_extra_param_name) || paramChanged(m_args.ss_extra_param_value)) {
          if(m_args.ss_extra_param_name.size() == m_args.ss_extra_param_value.size()) {
            for(unsigned i = 0; i<m_args.ss_extra_param_name.size(); i++) {
              inf("%s = %lf", m_args.ss_extra_param_name[i].c_str(), m_args.ss_extra_param_value[i]);
              m_emap.setParameterAll(m_args.ss_extra_param_name[i].c_str(), m_args.ss_extra_param_value[i]);
            }
            m_ss_valid = true;
          } else {
            m_ss_valid = false;
          }
        }
        if(paramChanged(m_args.singleEstimators) || paramChanged(m_args.multiEstimators)) {
          m_emap.clear();
          SingleReceiverEstimatorTypeToUse.clear();
          MultiReceiverEstimatorTypeToUse.clear();
          clearDUNETagBuffers_t(&tagBuffers);
          addEstimators();
        }
      }

      void addEstimators() {
        if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "EKF") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverEKF);
        } if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "UKF") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverUKF);
        } if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "SRUKF") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverSRUKF);
        } if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "ASLV") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverASLV);
        } if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "ASLV2") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverASLV2);
        } if(std::find(m_args.singleEstimators.begin(), m_args.singleEstimators.end(), "XKF") != m_args.singleEstimators.end()) {
          SingleReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_SingleReceiverXKF);
        }
        
        if(std::find(m_args.multiEstimators.begin(), m_args.multiEstimators.end(), "XKF") != m_args.multiEstimators.end()) {
          MultiReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_MultipleReceiverXKF);
        }
        if(std::find(m_args.multiEstimators.begin(), m_args.multiEstimators.end(), "EKF") != m_args.multiEstimators.end()) {
          MultiReceiverEstimatorTypeToUse.push_back(FishTagEstimators::EstimatorMap::estimatorTypeEnum_t::estimatorType_MultipleReceiverEKF);
        }
      }

      void
      onResourceAcquisition(void)
      {
        addEstimators();
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
          tagBuffers[msg->trans_id] = new FishTagEstimators::DUNETagBuffer(msg->trans_id, 5, m_args.depthConversion);
          // Set NED frame used on specific tag to location of first tag location
          double ref[] = {msg->lat, msg->lon, 0.0};
          tagBuffers[msg->trans_id]->setReferenceCoordinateRad(ref);
          tagBuffers[msg->trans_id]->setTimestampTimeoutLimit(m_args.timestampTimeout);
          spew("Created buffer for receiver %u", msg->serial_no);
          // Configure Single Receiver Estimators
          for(auto it = SingleReceiverEstimatorTypeToUse.begin();it !=SingleReceiverEstimatorTypeToUse.end();it++) {
            FishTagEstimators::Estimator<double>* est = m_emap.addEstimator(msg->trans_id,*it);
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
            if(m_ss_valid) {
              for(unsigned i = 0; i<m_args.ss_extra_param_name.size(); i++) {
                est->setParameter(m_args.ss_extra_param_name[i].c_str(), m_args.ss_extra_param_value[i]);
              }
            }
            //est->setParameter("receiver_depth", -0.5);
            //est->setParameter("max_jitter", 0.01);
            //est->setParameter("max_updates_per_new_measurement", 1);
            //est->setParameter("max_correction_attempts", 0);
            //est->setParameter("interval_mode", 1);
            //est->setParameter("tag_period", 10.0);
            // Create/clear csv logfile for estimator with header
            std::ofstream logOutStream;
            logOutStream.open(m_args.log_folder_and_prefix + m_startupTimestamp + est->name + std::to_string(est->trans_id) + ".csv", std::ofstream::out | std::ofstream::trunc);
            if (logOutStream.good()) {
                logOutStream << "t0,timestamp,N,E,D,Lat,Lon" << std::endl;
                logOutStream.close();
            }
          }
        }

        // Action taken for all receptions: Add to buffer and run measurment update on estimators.
        size_t prev = tagBuffers[msg->trans_id]->size();
        if(tagBuffers[msg->trans_id]->addTagDetection(msg)) {
          // Add MultiReceiver Estimators when going from 2 to 3 receiving receivers
          if((prev == 2) && tagBuffers[msg->trans_id]->size() == 3) {
            for(auto it = MultiReceiverEstimatorTypeToUse.begin();it !=MultiReceiverEstimatorTypeToUse.end();it++) {
              FishTagEstimators::Estimator<double>* est = m_emap.addEstimator(msg->trans_id,*it);
              spew("Added MultiReceiver Estimator");
              // Configure a multiReceiverEstimator
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
              // Create/clear csv logfile for estimator with header
              std::ofstream logOutStream;
              logOutStream.open(m_args.log_folder_and_prefix + m_startupTimestamp + est->name + std::to_string(est->trans_id) + ".csv", std::ofstream::out | std::ofstream::trunc);
              if (logOutStream.good()) {
                  logOutStream << "t0,timestamp,N,E,D,Lat,Lon" << std::endl;
                  logOutStream.close();
              }
              // Add all receivers to UnprocessedData in current Estimator
              for (FishTagEstimators::tagBufferMap_t::const_iterator receiver = (tagBuffers[msg->trans_id])->tagBuffer.begin(); receiver != (tagBuffers[msg->trans_id])->tagBuffer.end(); receiver++) {
                est->updateUnprocessedData(receiver->first);
              }
            }
          }
          m_emap.updateUnprocessedDataAll(msg->serial_no);
          newData[msg->trans_id] = true;
          spew("Receivers in buffer: %lu", tagBuffers[msg->trans_id]->size());

/*
Legg til timer, legg til UnprocessedData i m_emap, muligens med tagID i stedet for receiver
Sjekk timer i onMain, kjør m_emap.updateAll(msg->trans_id, tagBuffers[msg->trans_id]); når unprocessedData
*/
          //if(std::time(nullptr) - tagBuffers[msg->trans_id]->getLatestTimestamp() > 2.0) {
          //  findFishPosition(m_tagDetection);
          //}
          //m_emap.updateAll(msg->trans_id, tagBuffers[msg->trans_id]);
          spew("Detection from receiver %u added to buffer storing tag ID %u.", msg->serial_no, msg->trans_id);
          spew("Latest timestamp: %ld", tagBuffers[msg->trans_id]->getLatestTimestamp());
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
        clearDUNETagBuffers_t(&tagBuffers);
      }

      void
      onResourceInitialization(void)
      {
        m_filter_timer.setTop(m_args.filter_timestep);
        m_startupTimestamp = std::to_string( Clock::getSinceEpochMsec() );
      }

      //! Function for logging to an external file and dispatching the result as IMC
      //! @param [in] result The result to be logged. This is a NED value
      //! @param [in] in_logfilename Filename of file written to
      //! @param [in] logname Name used for the ID in the dispatched IMC::RemoteSensorInfo
      void logResult(FishTagEstimators::Estimator<double>* est, const std::string &in_logfilename, const std::string &logname) {
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
              logOutStream << est->getLatestTimestamp()<< ","<< Clock::getSinceEpochMsec() << "," << result[0] << "," << result[1] << "," << result[2] << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi);
            for(auto it : est->getUsedPositions()) {
              logOutStream << ","<< it.first << ","<< it.second;
              //inf("Receiver Position (North,East): (%f,%f)", it.first, it.second);
            }
            logOutStream << std::endl;
            logOutStream.close();
              //spew("Logstream good.");
          }/* else {
            war("Logstream not good: %s", in_logfilename.c_str());
          }*/  
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

            std::vector<uint32_t> estimatorsToDelete;
            for (FishTagEstimators::EstimatorMap::EstimatorMap_t::iterator it = m_emap.estimatorMap.begin(); it != m_emap.estimatorMap.end(); it++)
            {
              if (it->second != NULL)
              {
                for(FishTagEstimators::EstimatorMap::EstimatorVector_t::iterator est = it->second->begin();est != it->second->end();est++) {
                  if(newData[it->first]) {// && (*est)->checkTime((double)tagBuffers[it->first]->getLatestTimestamp(), Clock::getSinceEpoch())) {
                    (*est)->update(tagBuffers[it->first]);
                    //inf("Updated %s %d %f", (*est)->name.c_str(), (*est)->trans_id, Clock::getSinceEpoch() - (double)tagBuffers[it->first]->getLatestTimestamp());
                  }
                  
                    //inf("Updated %s %d %f", (*est)->name.c_str(), (*est)->trans_id, Clock::getSinceEpoch() - (double)tagBuffers[it->first]->getLatestTimestamp());

                  if(( *est)->isActive()) {
                    logResult((*est), m_args.log_folder_and_prefix + m_startupTimestamp + (*est)->name + std::to_string((*est)->trans_id) + ".csv", (*est)->name);
                  }


                }
                newData[it->first] = false;


                //inf("Limit %d, current: %f last %d, delta %f", tagBuffers[it->first]->timestampTimeoutLimit, DUNE::Time::Clock::getSinceEpoch(), tagBuffers[it->first]->latestTimestamp, DUNE::Time::Clock::getSinceEpoch() - tagBuffers[it->first]->latestTimestamp);
                //if(!tagBuffers[it->first]->checkTimeout(DUNE::Time::Clock::getSinceEpoch())) {
                //  war("Tag %d timed out.", it->first);
                //  estimatorsToDelete.push_back(it->first);
                //}
              }
            }

            for(auto &id : estimatorsToDelete) {
               war("Removing tag: %d .", id);
              m_emap.removeEstimators(id);
              auto it = tagBuffers.find(id);
              if(it != tagBuffers.end()) {
                delete it->second;
                tagBuffers.erase(id);
              }
            }
            m_emap.predictAll();
          }
          waitForMessages(m_args.message_wait_time);
        }
      }
    };
  }
}

DUNE_TASK