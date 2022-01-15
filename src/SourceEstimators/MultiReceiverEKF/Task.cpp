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

#define MultiReceiverEKFLog 1

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <unistd.h>
#include <Eigen/Core>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>


namespace SourceEstimators
{
  //! TODO: Implement altitude in TBR and use this. More todos in code.
  //! TODO: Implement use preassure/depth from tag. 
  //! Insert explanation on task behaviour here.
  //! @author Nikolai Lauvås
  namespace MultiReceiverEKF
  {
    using DUNE_NAMESPACES;

    static const unsigned c_buffer_size = 5;
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
      //! Extended Kalman filter - Rm
      std::vector<double> ekf_Rm; 
      //! Extended Kalman filter - P0
      std::vector<double> ekf_P0;
      //! Extended Kalman filter - x0
      std::vector<double> ekf_x0;

      //! Logfile folder and prefix
      std::string log_folder_and_prefix;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Buffer holding received tag detections.
      //boost::circular_buffer<IMC::TBRFishTag> *tagBuffer;
      typedef std::map<uint32_t, bool> unusedData_t;
      unusedData_t unusedData;
      typedef std::map<uint32_t, boost::circular_buffer<IMC::TBRFishTag>*> tagBuffer_t;
      tagBuffer_t tagBuffer;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;
      //! Current Speed of sound in water
      float m_c_speed;
      //! Speed of sound provider entity label.
      //int m_c_sound_eid;

      double m_max_tdoa_ms;

      //unsigned totalTagDetections;
      //std::string logfilename;
      //std::string aslvlogfilename;
      OFP::KalmanFilterDynamic<double, c_states> m_ekf;

      //! How far back into the buffer to attempt period matching.
      //unsigned int m_max_correction_attempts;
      //! Reference coordinate used to calculate NED frame
      double m_refCoord[3];

      std::string logfilename;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
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

        param("Log Folder and Prefix - 1", m_args.log_folder_and_prefix)
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
        .defaultValue("100, 0, 0, 0, 100, 0, 0, 0, 10}");

        param("Rm", m_args.ekf_Rm)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("1.5*3.8706, 0, 0, 2.1638e6");

        param("Qm", m_args.ekf_Qm)
        .size(c_states*c_states)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0");
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
        logfilename = m_args.log_folder_and_prefix + getEntityLabel() + ".log";

        m_refCoord[0] = Math::Angles::radians(m_args.reference[0]);
        m_refCoord[1] = Math::Angles::radians(m_args.reference[1]);
        m_refCoord[2] = 0.0;

        m_c_speed=m_args.init_c_sound;

        m_max_tdoa_ms = 500;

        if(paramChanged(m_args.filter_timestep))
          m_filter_timer.setTop(m_args.filter_timestep);
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
        std::ofstream logOutStream;
        logOutStream.open(logfilename, std::fstream::app);
        if (logOutStream.good()) {
            logOutStream << "N,E,D,Lat,Lon" << std::endl;
            logOutStream.close();
        }
      }

      void
      consume(const IMC::TBRFishTag* msg)
      {
        if(msg->trans_id == m_args.tag_id) {
          if(tagBuffer.find(msg->serial_no) == tagBuffer.end()) {
            // New receiver found, create buffer
            tagBuffer[msg->serial_no] = new boost::circular_buffer<IMC::TBRFishTag>(c_buffer_size);
            spew("Created buffer for receiver %u", msg->serial_no);
          }
          tagBuffer[msg->serial_no]->push_back(*msg);
          unusedData[msg->serial_no] = true;
          updateFilter2();
        }
        // Ignore other tags
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_ekf.A << Eigen::Matrix3d::Identity();
        m_ekf.Q = Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_Qm.data());
        m_ekf.PHat = Eigen::Map<Eigen::Matrix<double, c_states, c_states> >(m_args.ekf_P0.data());
        m_ekf.xHat = Eigen::Map<Eigen::Matrix<double, c_states, 1> >(m_args.ekf_x0.data());
        //m_ekf.R = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_Rm.data());
        m_filter_timer.setTop(m_args.filter_timestep);
        m_ekf.dt = m_args.filter_timestep;
        /*m_ekf.h = [](Eigen::Matrix<double, c_states, 1> x, Eigen::Matrix<double, Eigen::Dynamic, 1> u) {
          // Add range difference jacobian for all valid combinations
          // Add depth estimate


          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          (x-u.block(3,0,3,1)).squaredNorm();
          // Calculate estimated measurements
          Eigen::Matrix<double, Eigen::Dynamic,1> ykest;
          return ykest;
        };
        m_ekf.calculateJacobian = [](Eigen::Matrix<double, c_states, 1> x, Eigen::Matrix<double, Eigen::Dynamic, 1> u) {
          // Add range difference for all valid combinations
          // Add depth estimate
          // Calculate estimated measurements
          Eigen::Matrix<double, Eigen::Dynamic,c_states> C;//(u.rows(), c_states);
          C.resize(u.rows(), c_states);
          Eigen::Matrix<double, 3, 1> distance1 = x-u.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-u.block(3,0,3,1); // X_e-X_rx1
          double r1 = distance1.norm();// ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||
          std::cout << "r1: " << r1 << ", r2: " << r2 << std::endl;
          C.row(0) = (distance2/r2) - (distance1/r1);
          return Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic,c_states> >(C.data(), u.rows(), c_states);
        };*/
        m_ekf.active = true;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        //spew("Entering Release");
        for (tagBuffer_t::iterator it = tagBuffer.begin(); it != tagBuffer.end(); it++)
        {
          Memory::clear(it->second);
          spew("Cleared buffer for receiver %u", it->first);
        }
        tagBuffer.clear();
        //spew("Exiting Release");
      }

      void printBuffer(void) {
        for (tagBuffer_t::iterator it = tagBuffer.begin(); it != tagBuffer.end(); it++)
        {
          spew("Content of buffer for receiver %u", it->first);
          for(boost::circular_buffer<DUNE::IMC::TBRFishTag>::reverse_iterator i=(it->second)->rbegin(); i != (it->second)->rend();i++) {
            spew("%u", i->unix_timestamp);
          }
        }
      }

      //! Check if the TDOA indicates a time shift larger than accepted
      //! @param [in] TDOA Time Difference of Arrival 
      //! @return Boolean representing accepted/not accepted
      bool timeShiftCorrect(const long int TDOA)
      {
        if((std::abs(TDOA) <= m_max_tdoa_ms))
          return true;
        else
          return false;
      }

      //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
      //! @param [in] input Tag detection to take lat/lon [rad] from 
      //! @param [in] refCoord Reference coordinate in {lat[rad], lon [rad], elevation [m]} 
      //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
      void toNEDframe(const IMC::TBRFishTag &input, const double refCoord[3], std::tuple<double, double, double> &output)
      {
        //inf("%f, %f",Math::Angles::degrees(input.lat), Math::Angles::degrees(input.lon));
        WGS84::displacement(refCoord[0], refCoord[1], refCoord[2], input.lat, input.lon, 0.0, &(std::get<0>(output)), &(std::get<1>(output)), &(std::get<2>(output)));
      }

      //! Takes a NED frame position and transforms it to a WGS84 lat/lon/elevation position
      //! @param [in] input NED frame position to transform {North, East, Down} [meters] relative to the reference coordinate
      //! @param [in] refCoord Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
      //! @param [out] output Input position converted to WGS84 coordinates {lat[rad], lon [rad], elevation [m]} 
      void fromNEDframe(const double input[3], const double refCoord[3], double (&output)[3]) {
        output[0] = refCoord[0];
        output[1] = refCoord[1];
        output[2] = refCoord[2];
        WGS84::displace(input[0], input[1], input[2], &(output[0]), &(output[1]), &(output[2]));
      }

      //! Function for logging to an external file and dispatching the result as IMC
      //! @param [in] result The result to be logged. This is a NED value
      //! @param [in] in_logfilename Filename of file written to
      //! @param [in] logname Name used for the ID in the dispatched IMC::RemoteSensorInfo
      void logResult(const double result[3], std::string in_logfilename, std::string logname) {
        double lati,longi;
        //double result[3] = {m_ekf.xHat(0),m_ekf.xHat(1),m_ekf.xHat(2)};
        double latLon[3];
        fromNEDframe(result, m_refCoord, latLon);
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
        #if MultiReceiverEKFLog
        std::ofstream logOutStream;
        logOutStream.open(in_logfilename, std::fstream::app);
        if (logOutStream.good()) {
          logOutStream.precision(15);
            logOutStream << result[0] << "," << result[1] << "," << result[2] << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
            logOutStream.close();
        }   
        #endif
        spew("New Kalman Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", m_ekf.xHat(0), m_ekf.xHat(1), m_ekf.xHat(2),DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
      }

      void updateFilter2(void) {
        if(tagBuffer.size() <2)
          return;
        int com = boost::math::binomial_coefficient<double>(tagBuffer.size(), 2);
        inf("com: %i", com);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RDOA(com  ,1);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> NED(3,tagBuffer.size());

        //Eigen::Matrix<double, Eigen::Dynamic, 1> ykest(1,1);
        //Eigen::Matrix<double, Eigen::Dynamic, c_states> C(1,c_states);
        m_ekf.ykest.resize(1,1);
        m_ekf.C.resize(1,c_states);


// TODO: Fix that NED is calculated two times
        std::tuple<double, double, double> tempNED;
        unsigned combinations = 0;
        unsigned outer = 0;
        unsigned inner;
        for (tagBuffer_t::iterator outerreceiver = tagBuffer.begin(); outerreceiver != tagBuffer.end(); outerreceiver++) {
          toNEDframe(*outerreceiver->second->rbegin(), m_refCoord, tempNED);
          NED.col(outer) << std::get<0>(tempNED), std::get<1>(tempNED),std::get<2>(tempNED);
          inner = outer;
          for (tagBuffer_t::iterator receiver = std::next(outerreceiver); receiver != tagBuffer.end(); receiver++) {
            inner++;
            //inf("%u - %u", outerreceiver->first, receiver->first);
            if( unusedData[outerreceiver->first] || unusedData[receiver->first]) {
              long int tempTDOA_ms = ((long int)outerreceiver->second->rbegin()->unix_timestamp - receiver->second->rbegin()->unix_timestamp)*1000 + ((int)outerreceiver->second->rbegin()->millis - receiver->second->rbegin()->millis);
              if(timeShiftCorrect(tempTDOA_ms)) {
                toNEDframe(*receiver->second->rbegin(), m_refCoord, tempNED);
                NED.col(inner) << std::get<0>(tempNED), std::get<1>(tempNED),std::get<2>(tempNED);
                //inf("Usable TDOA: %li", tempTDOA_ms);
                RDOA.row(combinations) << m_c_speed*tempTDOA_ms/1000;
                if(m_ekf.active) {
                  Eigen::Matrix<double, c_states, 1> distance1 = m_ekf.xHat-NED.col(outer); // X_e-X_rx0
                  Eigen::Matrix<double, c_states, 1> distance2 = m_ekf.xHat-NED.col(inner); // X_e-X_rx1
                  double r1 = distance1.norm();//  ||X_e-X_rx0||
                  double r2 = distance2.norm();// ||X_e-X_rx1||

                  // Update ykest
                  //inf("Diff: %u, %u, %f", outer, inner, r2 - r1);

                  m_ekf.ykest.row(m_ekf.ykest.rows() -1) << r2 - r1;
                  //std::cout << "Row" << ykest.row(ykest.rows() -1) << std::endl;
                  m_ekf.ykest.conservativeResize(m_ekf.ykest.rows()+1,1);

                  // Update C and R
                  m_ekf.C.row(m_ekf.C.rows() -1) = (distance2/r2) - (distance1/r1);
                  m_ekf.C.conservativeResize(m_ekf.C.rows()+1,c_states);
                  inf("C");
                  std::cout << m_ekf.C << std::endl;
                }
              }
            }
            combinations++;
          }
          outer++;
        }


        if(combinations <3) {
          return; // Do not update filter
        }

        double depth = 2.5;

        m_ekf.ykest.row(m_ekf.ykest.rows()-1) << m_ekf.xHat(3,1);
        m_ekf.C.row(m_ekf.C.rows()-1) << 0,0,1;

        
        

        Eigen::Matrix<double, Eigen::Dynamic, 1> measurements(RDOA.cols()*RDOA.rows()+1,1);
        measurements << Eigen::Map<Eigen::VectorXd>(RDOA.data(), RDOA.cols()*RDOA.rows());
        measurements.row(RDOA.cols()*RDOA.rows()) << depth;

        m_ekf.R = 2*Eigen::MatrixXd::Identity(measurements.rows(), measurements.rows());
        m_ekf.R(measurements.rows() - 1, measurements.rows() - 1) = 0.2;
        m_ekf.update(measurements);

        inf("C");
        std::cout << m_ekf.C << std::endl;
        //inf("Measurements");
        //std::cout << measurements << std::endl;
        /*inf("NED");
        std::cout << Eigen::Map<Eigen::VectorXd>(NED.data(), NED.cols()*NED.rows()) << std::endl;
        inf("NED");
        std::cout << NED << std::endl;*/
      }
      //! Main loop.
      void
      onMain(void)
      {

        while (!stopping())
        {
          if(m_filter_timer.overflow()) {
            m_filter_timer.reset();
            //printBuffer();
            if(m_ekf.active) {
              m_ekf.predict(); // Filter time update
              double result[3] = {m_ekf.xHat(0),m_ekf.xHat(1),m_ekf.xHat(2)};
              logResult(result, logfilename, "OFPEKF");
            }
          }
          waitForMessages(m_args.message_wait_time);
        }
      }
    };
  }
}

DUNE_TASK
