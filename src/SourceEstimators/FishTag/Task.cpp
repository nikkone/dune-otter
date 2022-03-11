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
#include <FishTagEstimators/MultipleReceiverEKF.hpp>
#include <FishTagEstimators/SingleReceiverEKF.hpp>
#include <FishTagEstimators/SingleReceiverUKF.hpp>
#include <FishTagEstimators/SingleReceiverSRUKF.hpp>



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
      //! Tag ID
      unsigned tag_id;
      //! Time to wait in while. In practice, this controls how regular the filter timing is
      float message_wait_time;
      //! Period between when filter is run
      float filter_timestep;
      //! Initial Speed of Sound in water
      float init_c_sound;
// Location settings
      //! Reference coordinate position (degrees)
      std::vector<double> reference;
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

      std::string log_folder_and_prefix;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Datastructure to hold task arguments/parameters
      Arguments m_args;

      FishTagEstimators::DUNETagBuffers_t tagBuffers;

      FishTagEstimators::MultipleReceiverEKF m_ekf;
      FishTagEstimators::SingleReceiverEKF m_sekf;
      FishTagEstimators::SingleReceiverUKF m_sukf;
      FishTagEstimators::SingleReceiverSRUKF m_ssrukf;



      
      std::vector<FishTagEstimators::Estimator*> estimators;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Filter Timestep", m_args.filter_timestep)
        .description("The timestep of the filter")
        .units(Units::Second)
        .defaultValue("7.0");

        param("Tag ID", m_args.tag_id)
        .description("The ID of the tracked fish tag")
        .defaultValue("40");

        param("Initial Speed Of Sound", m_args.init_c_sound)
        .units(Units::MeterPerSecond)
        .description("The ID of the tracked fish tag")
        .defaultValue("1485.0");

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

        param("ToA Cov", m_args.rr_cov)
        .description("Time of Arrival Covariance")
        .defaultValue("0");

        param("Depth Cov", m_args.rz_cov)
        .description("Depth measurement Covariance")
        .defaultValue("0");

        param("Max time shift [ms]", m_args.max_time_shift_ms)
        .description("Maximum allowed time [ms] shift between receivers' messages")
        .defaultValue("500");

        param("Qm", m_args.ekf_Qm)
        .size(c_states*c_states)
        .description("Process noise covariance matrix for the extended Kalman filter")
        .defaultValue("1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1");
// Others

        param("Reference Coordinate", m_args.reference)
        .defaultValue("63.334, 10.084333")
        .units(Units::Degree)
        .size(2)
        .description("Origin of the reference coordinate system");

        bind<IMC::TBRFishTag>(this);
      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.filter_timestep))
          m_filter_timer.setTop(m_args.filter_timestep);
      }
      void
      onResourceAcquisition(void)
      {
        estimators.push_back(&m_sekf);
        estimators.push_back(&m_sukf);
        estimators.push_back(&m_ssrukf);


        //estimators.push_back(&m_ekf);

        for(std::vector<FishTagEstimators::Estimator*>::iterator it = estimators.begin();it != estimators.end();it++) {
          (*it)->trans_id = m_args.tag_id;
          (*it)->setSoundSpeed(m_args.init_c_sound);
          (*it)->setAllowedTimeShift(m_args.max_time_shift_ms);
          (*it)->setTDOACovariance(m_args.rr_cov);
          (*it)->setDepthCovariance(m_args.rz_cov);
          //(*it)->setReferenceCoordinate(m_args.reference.data());
          (*it)->initialize(
          Eigen::Matrix3d::Identity(),
          Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_Qm.data()),
          Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_P0.data()),
          Eigen::Map<Eigen::Matrix<double, c_states, 1> >(m_args.ekf_x0.data())
          );
          (*it)->setParameter("receiver", 1000052);
          (*it)->setParameter("receiver_depth", -2.0);
          (*it)->setParameter("tag_period", 10.0);
          (*it)->setParameter("max_jitter", 0.01);
          (*it)->setParameter("max_updates_per_new_measurement", 1);
          (*it)->setParameter("max_correction_attempts", 0);
          std::ofstream logOutStream;
          logOutStream.open(m_args.log_folder_and_prefix + (*it)->name + ".csv", std::ofstream::out | std::ofstream::trunc);
          if (logOutStream.good()) {
              logOutStream << "timestamp,N,E,D,Lat,Lon" << std::endl;
              logOutStream.close();
          }
        }
      }

      void
      consume(const IMC::TBRFishTag* msg)
      {

          if(tagBuffers.find(msg->trans_id) == tagBuffers.end()) {
            // New tag found, create buffer
            tagBuffers[msg->trans_id] = new FishTagEstimators::DUNETagBuffer(msg->trans_id, 5);
            tagBuffers[msg->trans_id]->setReferenceCoordinate(m_args.reference.data());
            spew("Created buffer for receiver %u", msg->serial_no);
          }
          if(tagBuffers[msg->trans_id]->addTagDetection(msg)) {
            for(std::vector<FishTagEstimators::Estimator*>::iterator it = estimators.begin();it != estimators.end();it++) {
              (*it)->update(tagBuffers[msg->trans_id]);
            }
            spew("Detection from receiver %u added to buffer storing tag ID %u.", msg->serial_no, msg->trans_id);
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
        //std::tuple<double, double, double>  estimate = est->getEstimate();
        //double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
        //spew("New Kalman Estimate: (N,E,D)= %.15f,%.15f,%.15f", result[0], result[1], result[2]);

        double lati,longi;
        std::tuple<double, double, double>  estimate = est->getEstimate();
        double result[3] = {std::get<0>(estimate), std::get<1>(estimate), std::get<2>(estimate)};
        double latLon[3];

        tagBuffers[est->trans_id]->fromNEDframe(result, latLon);
        //est->fromNEDframe(result, latLon);
        lati=latLon[0], longi=latLon[1];

        // Send output to Neptus/DUNE log
        IMC::RemoteSensorInfo tagPosition;
        tagPosition.lat = lati;
        tagPosition.lon = longi;
        tagPosition.alt = -result[2];
        tagPosition.data = std::to_string(result[0]) + std::to_string(result[1]) + "," + std::to_string(result[2]);
        //tagPosition.data << result[0] << "," << result[1] << "," << result[2];
        tagPosition.id = logname + std::to_string(m_args.tag_id);
        dispatch(tagPosition);

        // External Logfile
        #if LOGFTOILE
        std::ofstream logOutStream;
        logOutStream.open(in_logfilename, std::fstream::app);
        if (logOutStream.good()) {
          logOutStream.precision(15);
            logOutStream << Clock::getSinceEpochMsec() << "," << result[0] << "," << result[1] << "," << result[2] << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
            //logOutStream << *est;
            logOutStream.close();
        }   
        #endif
        spew("New Kalman Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", result[0], result[1], result[2],DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
      
      }

      void
      onMain(void)
      {
        while(!stopping()) {
          if(m_filter_timer.overflow()) {
            m_filter_timer.reset();
            for(std::vector<FishTagEstimators::Estimator*>::iterator it = estimators.begin();it != estimators.end();it++) {
              (*it)->predict();
              if(( *it)->isActive()) {
                logResult((*it), m_args.log_folder_and_prefix + (*it)->name + ".csv", (*it)->name);
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
