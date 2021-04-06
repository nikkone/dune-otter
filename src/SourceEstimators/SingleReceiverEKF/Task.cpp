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

#define SingleReceiverEKFLog 1

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <boost/circular_buffer.hpp>
#include <OpenFilterPack/AlgebraicSolution.hpp>
#include <OpenFilterPack/ExtendedKalmanFilter.hpp>
namespace SourceEstimators
{
  //! TODO: Implement altitude in TBR and use this. More todos in code.
  //! TODO: Implement use preassure/depth from tag. 
  //! Insert explanation on task behaviour here.
  //! @author Nikolai Lauvås
  namespace SingleReceiverEKF
  {
    using DUNE_NAMESPACES;

    static const unsigned c_buffer_size = 5;

    struct Arguments
    {
      //! Fitting of SNR to range
      std::vector<double> ranging_snr_fit;
      //! Jitter upper limit (seconds)
      float max_jitter;
      //! Expected tag Period
      float tag_period;
      //! Period between when filter is run
      float filter_timestep;
      //! Tag ID
      unsigned tag_id;
      //! Time to wait in while. In practice, this controls how regular the filter timing is
      float message_wait_time;
      //! How deep the receiver is mounted in altitude
      float receiver_depth;

// Initial Parameters for calculating Speed of Sound
      //! Initial Speed of Sound in water
      float init_c_sound;
// Parameters for updating Speed of Sound

      //! Should the Speed of Sound in water be updated from measurement
      bool update_c_sound;
      //! Entity providing the Speed of Sound in water
      std::string entity_c_sound;

      //! How far back into the buffer to attempt period matching.
      int max_correction_attempts;
// Kalman Filter
      //! Extended Kalman filter - Qm
      std::vector<double> ekf_Qm;      
      //! Extended Kalman filter - Rm
      std::vector<double> ekf_Rm; 
      //! Extended Kalman filter - P0
      std::vector<double> ekf_P0;
      //! Extended Kalman filter - x0
      std::vector<double> ekf_x0;
// Location settings

      uint32_t receiver_serial;
      //! Reference coordinate position (degrees)
      std::vector<double> reference;
    };
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Buffer holding received tag detections.
      boost::circular_buffer<IMC::TBRFishTag> *tagBuffer;
      //! Timer responsible for running filter timestep
      Time::Counter<float> m_filter_timer;
      //! Current Speed of sound in water
      float m_c_speed;
      //! Speed of sound provider entity label.
      int m_c_sound_eid;

      OFP::ExtendedKalmanFilter<double, 3, 3> m_ekf;
      OFP::ExtendedKalmanFilter<double, 3, 1> m_ekf2;
      OFP::AlgebraicSolver<double, 3, 9, 5> m_aslv;
      Eigen::Matrix<double, 3, 1> pos_current;
      Eigen::Matrix<double, 3, 1> pos_previous;
      //! How far back into the buffer to attempt period matching.
      int m_max_correction_attempts;
      //! Reference coordinate used to calculate NED frame
      double m_refCoord[3];
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Ranging - SNR Fit", m_args.ranging_snr_fit)
        .size(2)
        .description("Linear fit of SNR to distance (a, b, forms r=ax+b)");

        param("Tag Max Jitter", m_args.max_jitter)
        .description("The maximum jitter allowed between measurements")
        .units(Units::Second)
        .defaultValue("0.01");

        param("Tag Transmitt Period", m_args.tag_period)
        .description("The expected period between tag registration")
        .units(Units::Second)
        .defaultValue("7.0");

        param("Receiver Serial Number", m_args.receiver_serial)
        .description("The serial number of the receiver to accept tag registrations from.")
        .defaultValue("634");

        param("Receiver Depth", m_args.receiver_depth)
        .description("The depth of the receiver providing sensor messages")
        .units(Units::Meter)
        .defaultValue("3.0");

        param("Message Wait Time", m_args.message_wait_time)
        .description("The time to wait for new messages in the while loop between checking timer.")
        .units(Units::Second)
        .defaultValue("0.01");

        param("Filter Timestep", m_args.filter_timestep)
        .description("The timestep of the filter")
        .units(Units::Second)
        .defaultValue("7.0");

        param("Max Correction Attempts", m_args.max_correction_attempts)
        .description("How far back into the buffer to attempt period matching. -1 gives max allowed in used buffer")
        .defaultValue("-1");

        param("Tag ID", m_args.tag_id)
        .description("The ID of the tracked fish tag")
        .defaultValue("40");
// Initial Parameters for calculating Speed of Sound
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


// Kalman Filter Parameters
        param("x0", m_args.ekf_x0)
        .size(3)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("0.0, 0.0, 0.0");

        param("P0", m_args.ekf_P0)
        .size(9)
        .description("Initial P matrix value for the extended Kalman filter, first row")
        .defaultValue("66458, -26820, 0, -26820, 12116, 0, 0, 0, 0");

        param("Rm", m_args.ekf_Rm)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("1.5*3.8706, 0, 0, 2.1638e6");

        param("Qm", m_args.ekf_Qm)
        .size(9)
        .description("Initial X value for the extended Kalman filter")
        .defaultValue("0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00001");
// Others
        param("Reference Coordinate", m_args.reference)
        .units(Units::Degree)
        .size(2)
        .description("Origin of the reference coordinate system");

        bind<IMC::TBRFishTag>(this);
        bind<IMC::SoundSpeed>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_refCoord[0] = Math::Angles::radians(m_args.reference[0]);
        m_refCoord[1] = Math::Angles::radians(m_args.reference[1]);
        m_refCoord[2] = 0.0;

        m_c_speed=m_args.init_c_sound;

        if(m_args.max_correction_attempts < 0)
          m_max_correction_attempts = c_buffer_size-2;
        else {
          m_max_correction_attempts = m_args.max_correction_attempts;
        }

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
        tagBuffer = new boost::circular_buffer<IMC::TBRFishTag>(c_buffer_size);
      }

      void
      consume(const IMC::TBRFishTag* msg)
      {
      #if SingleReceiverEKFLog  
      std::ofstream logOutStream;
      std::string filename = "log/tag-";
                  filename += getEntityLabel();
                  filename += ".log";
      logOutStream.open(filename, std::fstream::app);
      if (logOutStream.good()) {
        logOutStream.precision(15);
          logOutStream << DUNE::Math::Angles::degrees(msg->lat) << "," << DUNE::Math::Angles::degrees(msg->lon) << "," << msg->unix_timestamp << "," << msg->millis<< std::endl;
          logOutStream.close();
      #endif
      }   
        if (m_args.receiver_serial == msg->serial_no) {
          if(msg->trans_id == m_args.tag_id) {
            tagBuffer->push_back(*msg);
            updateFilter();
          }
          // Ignore other tags
        }
      }

      void
      consume(const IMC::SoundSpeed* msg)
      {
        if(msg->getSourceEntity() == m_c_sound_eid) {
          if(m_args.update_c_sound) {
            m_c_speed = msg->value;
            spew("Setting c_sound to: %f", msg->value);
          }
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // m_ekf
          m_ekf.A << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
          m_ekf.Q = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_Qm.data());
          m_ekf.R = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_Rm.data());
          m_ekf.PHat = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_P0.data());
          m_ekf.xHat = Eigen::Map<Eigen::Matrix<double, 3, 1> >(m_args.ekf_x0.data());
        //! Set timer for periodic part of filter
        m_filter_timer.setTop(m_args.filter_timestep);
        m_ekf.dt = m_args.filter_timestep;
        m_ekf.h = [this](Eigen::Matrix<double, 3, 1> x) {
          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = x-this->pos_previous;//z.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-this->pos_current;//z.block(3,0,3,1);  // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||
          
          // Calculate estimated measurements
          Eigen::Matrix<double, 3, 1> ykest;
          ykest(0) = r2 - r1; // h is eq (2.16) in masters
          ykest(1) = r2; // Eq (2.19) in masters
          ykest(2) = x(2); // Depth estimate
          return ykest;
        };

        m_ekf.calculateJacobian = [this](Eigen::Matrix<double, 3, 1> x) {

          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = x-this->pos_previous;//z.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-this->pos_current;//z.block(3,0,3,1);  // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||

          // Calculate Jacobian with RDOA, SNR and Depth
          Eigen::Matrix<double, 3, 3> C;       // Observation matrix
          C.row(0) = (distance2/r2) - (distance1/r1); // Eq (2.18)
          C.row(1) = (distance2/r2); // Exends the Jacobian with eq (2.20)
          Eigen::Matrix<double, 1, 3> Hdepth= {0.0,0.0,1.0};
          C.row(2) = Hdepth;  

          return C;
        };
        // m_ekf2
          m_ekf2.A << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
          m_ekf2.Q = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_Qm.data());
          m_ekf2.R << m_args.ekf_Rm[0];
          m_ekf2.PHat = Eigen::Map<Eigen::Matrix<double, 3, 3> >(m_args.ekf_P0.data());
          m_ekf2.xHat = Eigen::Map<Eigen::Matrix<double, 3, 1> >(m_args.ekf_x0.data());
        //! Set timer for periodic part of filter
        m_filter_timer.setTop(m_args.filter_timestep);
        m_ekf2.dt = m_args.filter_timestep;
        m_ekf2.h = [this](Eigen::Matrix<double, 3, 1> x) {
          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = x-this->pos_previous;//z.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-this->pos_current;//z.block(3,0,3,1);  // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||
          
          // Calculate estimated measurements
          Eigen::Matrix<double, 1, 1> ykest;
          ykest(0) = r2 - r1; // h is eq (2.16) in masters
          return ykest;
        };

        m_ekf2.calculateJacobian = [this](Eigen::Matrix<double, 3, 1> x) {

          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = x-this->pos_previous;//z.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-this->pos_current;//z.block(3,0,3,1);  // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||

          // Calculate Jacobian with RDOA, SNR and Depth
          Eigen::Matrix<double, 1, 3> C;       // Observation matrix
          C.row(0) = (distance2/r2) - (distance1/r1); // Eq (2.18)
          return C;
        };
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(tagBuffer);
      }

          //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
          //! @param [in] input Tag detection to take lat/lon [rad] from 
          //! @param [in] refCoord Reference coordinate in {lat[rad], lon [rad], elevation [m]} 
          //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
          void toNEDframe(const IMC::TBRFishTag &input, const double refCoord[3], double (&output)[3])
          {
            double input_d[3] = {input.lat, input.lon, 0.0};
            toNEDframe(input_d,refCoord, output);
          }

          //! Turns the latitude and longtitude of the input to a NED representation with refCoord as origin.
          //! @param [in] input Location in WGS84 {lat[rad], lon [rad], elevation [m]} 
          //! @param [in] refCoord Reference coordinate in WGS84 {lat[rad], lon [rad], elevation [m]} 
          //! @param [out] output NED frame representation of input in {North, East, Down} [meters] relative to the reference coordinate
          void toNEDframe(const double input[3], const double refCoord[3], double (&output)[3])
          {
            WGS84::displacement(refCoord[0], refCoord[1], refCoord[2], input[0], input[1], input[2], &(output[0]), &(output[1]), &(output[2]));
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

      //! 
      void updateFilter(void) {
        //inf("Update %ld", c_buffer_size - tagBuffer->size());
        double measurement_millis = tagBuffer->rbegin()->unix_timestamp + (double)tagBuffer->rbegin()->millis/1000;
        // TODO: Stop at m_args.max_correction_attempts
        for(boost::circular_buffer<DUNE::IMC::TBRFishTag>::reverse_iterator i=tagBuffer->rbegin()+1; i != tagBuffer->rend();i++) {
          //inf("%d - %d", tagBuffer->rbegin()->unix_timestamp, i->unix_timestamp);
          double td = measurement_millis - i->unix_timestamp - (double)i->millis/1000;
          double closestMultipleOfPeriod = m_args.tag_period*std::round(td/m_args.tag_period);
          double tdoa = td - closestMultipleOfPeriod;
          inf("delta %f %f", td, closestMultipleOfPeriod);

          if(abs(tdoa) < m_args.max_jitter || td > 60.0) {
              double P[2] = {m_args.ranging_snr_fit[0], m_args.ranging_snr_fit[1]}; // Found on page 54 of master, will wary from experiment to experiment
              double rangeSNR = (tagBuffer->rbegin()->snr - P[1])/P[0];

              double rdoa = m_c_speed*tdoa; // Range difference
              double depth = i->trans_data*0.392;

              double NED1[3];
              double NED2[3];

              toNEDframe(*tagBuffer->rbegin(), m_refCoord, NED1);
              toNEDframe(*i, m_refCoord, NED2);

              Eigen::Matrix<double, 9, 1> allMeasurements;
              allMeasurements << NED2[0] ,NED2[1] ,m_args.receiver_depth, NED1[0] ,NED1[1] ,m_args.receiver_depth, rdoa, rangeSNR, depth;
              if (m_aslv.addMeasurement(allMeasurements)) {
              
                double result[3] = {m_aslv.x(0), m_aslv.x(1), m_aslv.x(2)};
                if(!m_ekf.active) {
                  m_ekf.xHat << m_aslv.x(0), m_aslv.x(1), m_aslv.x(2);
                  m_ekf.active = true;
                }
                if(!m_ekf2.active) {
                  m_ekf2.xHat << m_aslv.x(0), m_aslv.x(1), m_aslv.x(2);
                  m_ekf2.active = true;
                }
                #if SingleReceiverEKFLog
                double latLon[3];
                fromNEDframe(result, m_refCoord, latLon);
                std::ofstream logOutStream;
                std::string filename = "log/aslv-";
                            filename += getEntityLabel();
                            filename += ".log";
                logOutStream.open(filename, std::fstream::app);
                if (logOutStream.good()) {
                  logOutStream.precision(15);
                  logOutStream << m_aslv.x(0) << "," << m_aslv.x(1) << "," << m_aslv.x(2) << "," << DUNE::Math::Angles::degrees(latLon[0]) << "," << DUNE::Math::Angles::degrees(latLon[1]) << std::endl;
                  logOutStream.close();
                }
                #endif
              }

              pos_current <<  NED1[0], NED1[1], m_args.receiver_depth;
              pos_previous << NED2[0], NED2[1], m_args.receiver_depth;
              m_ekf.update(allMeasurements.block(6,0,3,1));
              m_ekf2.update(allMeasurements.block(6,0,1,1));
              return;
          }
        }
        war("No good TDOA value found");
      }

      //! Main loop.
      void
      onMain(void)
      {

        while (!stopping())
        {
          if(m_filter_timer.overflow()) {
            m_filter_timer.reset();
            if(m_ekf.active) {
              m_ekf.predict(); // Filter time update
                  double lati,longi;
                  double result[3] = {m_ekf.xHat(0),m_ekf.xHat(1),m_ekf.xHat(2)};
                double latLon[3];
                fromNEDframe(result, m_refCoord, latLon);
                lati=latLon[0], longi=latLon[1];
                  // Send output to Neptus/DUNE log
                  IMC::RemoteSensorInfo tagPosition;
                  tagPosition.lat = lati;
                  tagPosition.lon = longi;
                  tagPosition.alt = -m_ekf.xHat(2);
                  tagPosition.data = std::to_string(m_ekf.xHat(0)) + std::to_string(m_ekf.xHat(1)) + "," + std::to_string(m_ekf.xHat(2));
                  tagPosition.id = "OFPEKF" + std::to_string(m_args.receiver_serial);
                  dispatch(tagPosition);
                  #if SingleReceiverEKFLog
                  std::string filename = "log/predict-";
                  filename += getEntityLabel();
                  filename += ".log";
                  std::ofstream logOutStream;
                  logOutStream.open(filename, std::fstream::app);
                  if (logOutStream.good()) {
                    logOutStream.precision(15);
                      logOutStream << m_ekf.xHat(0) << "," << m_ekf.xHat(1) << "," << m_ekf.xHat(2) << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
                      logOutStream.close();
                  }   
                  #endif
                  spew("New Kalman Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", m_ekf.xHat(0), m_ekf.xHat(1), m_ekf.xHat(2),DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
              
            }
            if(m_ekf2.active) {
              m_ekf2.predict(); // Filter time update
                  double lati,longi;
                  double result[3] = {m_ekf2.xHat(0),m_ekf2.xHat(1),m_ekf2.xHat(2)};
                double latLon[3];
                fromNEDframe(result, m_refCoord, latLon);
                lati=latLon[0], longi=latLon[1];
                  // Send output to Neptus/DUNE log
                  IMC::RemoteSensorInfo tagPosition;
                  tagPosition.lat = lati;
                  tagPosition.lon = longi;
                  tagPosition.alt = -m_ekf2.xHat(2);
                  tagPosition.data = std::to_string(m_ekf2.xHat(0)) + std::to_string(m_ekf2.xHat(1)) + "," + std::to_string(m_ekf2.xHat(2));
                  tagPosition.id = "OFPEKF2" + std::to_string(m_args.receiver_serial);
                  dispatch(tagPosition);
                  #if SingleReceiverEKFLog
                  std::string filename = "log/predict2-";
                  filename += getEntityLabel();
                  filename += ".log";
                  std::ofstream logOutStream;
                  logOutStream.open(filename, std::fstream::app);
                  if (logOutStream.good()) {
                    logOutStream.precision(15);
                      logOutStream << m_ekf2.xHat(0) << "," << m_ekf2.xHat(1) << "," << m_ekf2.xHat(2) << "," << DUNE::Math::Angles::degrees(lati) << "," << DUNE::Math::Angles::degrees(longi) << std::endl;
                      logOutStream.close();
                  }   
                  #endif
                  spew("New Kalman Estimate: (N,E,D,La,Lo)= %.15f,%.15f,%.15f,%.15f, %.15f", m_ekf2.xHat(0), m_ekf2.xHat(1), m_ekf2.xHat(2),DUNE::Math::Angles::degrees(lati),DUNE::Math::Angles::degrees(longi));
              
            }
          }
          waitForMessages(m_args.message_wait_time);
        }
      }
    };
  }
}

DUNE_TASK
