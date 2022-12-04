//***************************************************************************
// Copyright 2013-2022 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file was developed for use in DUNE: Unified Navigation Environment. *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <Eigen/Core>
#include <ctime> /* time_t, struct tm, time, mktime */

// CPP STD headers
#include <algorithm>  // std::count
#include <iterator>


#include "XKF.hpp"
// External Libraries
//#include <OpenFilterPack/AlgebraicSolution.hpp>
//#include <OpenFilterPack/ExtendedKalmanFilter.hpp>

  namespace SourceEstimators
  {
      namespace MultipleReceiverXKF
      {
        using DUNE_NAMESPACES;

        const uint8_t c_receivers = 3; // TODO: Make everything scalable according to this number so x receivers can be used

        //! %Task arguments.
        struct Arguments
        {
          //! Reference coordinate position (degrees)
          std::vector<double> reference;
          //! Initial position of the fish tag (NED) for EKF
          std::vector<double> position;
          //! Sensor serial numbers to use
          std::vector<uint32_t> receiver_serial;
          //! Fish Position Covariance
          double qq_cov;
          //! Time of Arrival Covariance
          double rr_cov;
          //! Depth measurement Covariance
          double rz_cov;
          //! Initial value for speed of sound in water [m/s]
          double speed_of_sound_in_water;
          //! Maximum allowed time [ms] shift between receivers' messages
          double max_time_shift_ms;

// Kalman Filter
          //! Extended Kalman filter - Qm
          std::vector<double> ekf_Qm;      
          //! Extended Kalman filter - Rm
          std::vector<double> ekf_Rm; 
          //! Extended Kalman filter - P0
          std::vector<double> ekf_P0;
          //! Extended Kalman filter - x0
          std::vector<double> ekf_x0;
        };

        struct Task: public DUNE::Tasks::Periodic
        {

          //! Task arguments
          Arguments m_args;
          //! Output files for logging
          std::ofstream m_o[c_receivers];
          //! Storage for most recent fish tag detection per receiver.
          IMC::TBRFishTag m_tagDetection[c_receivers];
          //! To keep track of registrations that are used.
          bool m_newDetection[c_receivers];
          //! Value to compare against when desciding if a pair of measurements are valid
          double m_max_rdoa;
          //! Reference coordinate used to calculate NED frame
          double m_refCoord[3];
          //! Position in NED used to initialize the estimator
          double m_initPosition[3];
          //! Position estimate filter
          //OFP::ExtendedKalmanFilter<double, 3, 3, 0> m_ekf;
          // Class with eXogenous Kalman Filter
          XKF m_xkf;
          //! Most recent timestamp
          uint32_t newTimestamp;
          //! Constructor.
          //! @param[in] name task name.
          //! @param[in] ctx context.
          Task(const std::string& name, Tasks::Context& ctx):
            DUNE::Tasks::Periodic(name, ctx)
          {
            param("Receiver Serial Numbers", m_args.receiver_serial)
            .description("Receiver Serial Numbers")
            .size(c_receivers)
            .defaultValue("632, 634, 631");

            param("Max time shift [ms]", m_args.max_time_shift_ms)
            .description("Maximum allowed time [ms] shift between receivers' messages")
            .defaultValue("500");

            param("Speed of sound in water [m/s]", m_args.speed_of_sound_in_water)
            .description("Speed of sound in water [m/s]")
            .defaultValue("1485");

            param("FishPos Cov", m_args.qq_cov)
            .description("Fish Position Covariance")
            .defaultValue("0");

            param("ToA Cov", m_args.rr_cov)
            .description("Time of Arrival Covariance")
            .defaultValue("0");

            param("Depth Cov", m_args.rz_cov)
            .description("Depth measurement Covariance")
            .defaultValue("0");

            param("Reference Coordinate", m_args.reference)
            .units(Units::Degree)
            .size(2)
            .description("Origin of the reference coordinate system");

            param("Initial Position", m_args.position)
            .size(3)
            .description("Initial Fish tag position in NED from reference coordinate frame");
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
            // Setup processing of IMC messages
            // Initialize messages.
            bind<IMC::TBRFishTag>(this);
          }

          //! Update internal state with new parameter values.
          void
          onUpdateParameters(void)
          {
            m_max_rdoa = m_args.max_time_shift_ms/1000*m_args.speed_of_sound_in_water;

            m_xkf.setDiagonalCovarianceR(m_args.rr_cov);
            m_xkf.setDiagonalCovarianceQ(m_args.qq_cov);
            m_xkf.setVarianceRZ(m_args.rz_cov);

            m_xkf.setTimestep(1/getFrequency());

            m_refCoord[0] = Math::Angles::radians(m_args.reference[0]);
            m_refCoord[1] = Math::Angles::radians(m_args.reference[1]);
            m_refCoord[2] = 0.0;

            m_initPosition[0] = m_args.position[0];
            m_initPosition[1] = m_args.position[1];
            m_initPosition[2] = m_args.position[2];

            debug(DTR("Reference Cooridnate: %f %f"), m_args.reference[0], m_args.reference[1]);
            debug(DTR("Fish Position: %f %f %f"), m_args.position[0], m_args.position[1], m_args.position[2]);

            for (unsigned i = 0; i<c_receivers;i++) {
              m_o[i].open("/home/nikolai/ExperimentDataSet/New/E" + std::to_string(i) + "3output.txt", std::ios::out | std::ios::trunc);
              if(m_o[i].is_open())
              {
                debug(DTR("Stage %d open!"), i);
              } 
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
          }

          //! Initialize resources.
          void
          onResourceInitialization(void)
          {
            if(!m_xkf.isInitialized())
            {
              Eigen::Matrix<double, 3,1> xInit;
              xInit << m_initPosition[0], m_initPosition[1], m_initPosition[2];
              m_xkf.initialize(xInit);
            }
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          }

          //! Release resources.
          void onResourceRelease(void)
          {
          }

          void consume(const IMC::TBRFishTag* msg) {

            // Iterate through receivers
            for(unsigned i=0;i<c_receivers;i++) {
              if (m_args.receiver_serial[i] == msg->serial_no)
              {
                if(msg->unix_timestamp - newTimestamp > 2.0) {
                  // TODO: Check if more than one unused, then run update of filter
                  newTimestamp = msg->getTimeStamp();
                  //inf("New Timestamp: %d", newTimestamp);
                }
                debug(DTR("Message from R%d arrived"), i);
                m_tagDetection[i] = *msg;
                m_newDetection[i] = true;
                m_xkf.setActive(true);
                return;
              }
            }
            debug(DTR("consume: Message from non registered carrier with ID %d"), msg->getSource());
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

          //! Function responsible for loggin results from the three stages of the estimator
          void logFishPosition()
          {
            double result[3][3] = {{m_xkf.stage1.xHat(0,0), m_xkf.stage1.xHat(1,0), m_xkf.stage1.xHat(2,0)},
                                   {m_xkf.stage2.xHat(0,0), m_xkf.stage2.xHat(1,0), m_xkf.stage2.xHat(2,0)},
                                   {m_xkf.stage3.xHat(0,0), m_xkf.stage3.xHat(1,0), m_xkf.stage3.xHat(2,0)}
                                   };
            double latlon[3];
            //Log timestamps and estimates for stage 1-3
            for (unsigned i = 0; i<3;i++) {
              m_o[i].precision(15);
              
              m_o[i] << m_tagDetection[0].unix_timestamp << m_tagDetection[0].millis << "," << m_tagDetection[1].unix_timestamp << m_tagDetection[1].millis << "," << m_tagDetection[2].unix_timestamp << m_tagDetection[2].millis << ","; 
              fromNEDframe(result[i], m_refCoord, latlon);
              m_o[i] << DUNE::Math::Angles::degrees(latlon[0]) << "," << DUNE::Math::Angles::degrees(latlon[1]) << "," << latlon[2] << "," << m_xkf.stage1.xHat(0,0) << "," << m_xkf.stage1.xHat(1,0) << "," << m_xkf.stage1.xHat(2,0);
            }

            // Log Stage 1: LS Estimate
            m_o[0] << "," << m_xkf.stage1.m_dr << "," << Clock::getSinceEpochMsec() << std::endl;

            // Log Stage 2: LTV KF Estimate
            m_o[1] << "," << m_xkf.stage2.innov.norm() << "," << m_xkf.stage2.PHat.norm() << "," << m_xkf.stage2.PHat.trace() << "," << Clock::getSinceEpochMsec() << std::endl;

            // Log Stage 3: Linearized KF Estimate
            m_o[2] << "," << m_xkf.stage3.innov.norm() << "," << m_xkf.stage3.PHat.norm() << "," << m_xkf.stage3.PHat.trace() << "," << Clock::getSinceEpochMsec() << std::endl;
            
            // Send output to Neptus/DUNE log
            IMC::RemoteSensorInfo tagPosition;
            tagPosition.lat = latlon[0];
            tagPosition.lon = latlon[1];
            tagPosition.alt = -latlon[2];
            tagPosition.data = std::to_string(m_tagDetection[0].unix_timestamp) + std::to_string(m_tagDetection[0].millis) + "," + std::to_string(m_tagDetection[1].unix_timestamp) + std::to_string(m_tagDetection[1].millis) + "," + std::to_string(m_tagDetection[2].unix_timestamp) + std::to_string(m_tagDetection[2].millis);
            // + "," + m_xkf.stage3.innov.norm_p(2) + "," << m_xkf.stage3.PHat.norm_p(2) + "," << m_xkf.stage3.PHat.trace();
            tagPosition.id = "MultipleReceiverXKF";
            dispatch(tagPosition);
          }

          //! Check if the RDOA indicates a time shift larger than accepted
          //! @param [in] RDOA Range Difference of Arrival 
          //! @return Boolean representing accepted/not accepted
          bool timeShiftCorrect(const double RDOA)
          {
            if((std::abs(RDOA) <= m_max_rdoa))
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
            WGS84::displacement(refCoord[0], refCoord[1], refCoord[2], input.lat, input.lon, 0.0, &(std::get<0>(output)), &(std::get<1>(output)), &(std::get<2>(output)));
          }

          void findFishPosition(const IMC::TBRFishTag tagData[c_receivers]) {
            spew("Running filter update");
/* TODO:
Fiks NED ved combo 1-2 etc.
Fiks send hvilke rader av NED som korresponderer til hvilke RDOA
DONE: Fiks kjør update etter tidligst x sec etter første mottatte av sist timestamp slik at den ikke kjører eks. 0-2 fordi 1 ikke har rukket å komme.
Done Fiks separer predict og update kjøring. 
*/

            // Step 2: Compile NED positions and RDOA measurments.
            static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, c_receivers,1> RDOA2;
            //static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, c_receivers,3> RDOA3;
            static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, c_receivers,3> NED2;
            RDOA2.resize(0,1);
            //RDOA3.resize(0,3);
            NED2.resize(0,3);
            std::tuple<double, double, double> tempNED;

            //std::vector<double> RDOA; // In meters
            unsigned used_counter = 0;
            //std::vector<std::pair<uint8_t, uint8_t>> RDOAcombinations;
            //For loop to create unique order invariant permutations (Each receiver is combined with another only once.)
            std::vector<bool> used(c_receivers, false); // Initializes all to false TODO: Use std::transform to initialize to !(m_newDetection)
            for(uint8_t outerreceiver = 0;outerreceiver<c_receivers;++outerreceiver) {
              for(uint8_t receiver = outerreceiver+1;receiver<c_receivers;++receiver) {
                if( !(used[outerreceiver] && used[receiver]) && (m_newDetection[outerreceiver] || m_newDetection[receiver]) ) {
                  double tempRDOA = m_args.speed_of_sound_in_water*((static_cast<double>(tagData[receiver].unix_timestamp) - tagData[outerreceiver].unix_timestamp) * 1000.0 + (static_cast<double>(tagData[receiver].millis) - tagData[outerreceiver].millis))/1000.0;
                  if(timeShiftCorrect(tempRDOA)) {
                    //inf("%d, %d", outerreceiver, receiver);
                    //RDOA.push_back(tempRDOA);
                    //RDOAcombinations .push_back(std::pair<uint8_t, uint8_t>(outerreceiver, receiver));
                    RDOA2.resize(used_counter+1,1); // +1 because used_counter starts from 0.
                    RDOA2.row(used_counter) << tempRDOA;
                    //RDOA3.resize(used_counter+1,3); // +1 because used_counter starts from 0.
                    //RDOA3.row(used_counter) << tempRDOA, outerreceiver, receiver;
                    used[outerreceiver] = true;
                    used[receiver] = true;
                    used_counter++;
                  }
                }
              }
            }

            if(RDOA2.rows() < 1) {
              inf("No valid RDOA2 combinations");
              return;
            }
            /*if(RDOA.empty()) {
              inf("No valid combinations");
              return;
            }*/
            //spew("RDOA made");
            NED2.resize(used_counter+1,3);
            //spew("NED2 resize");
            // Create NED representation of used detections, and remove them from the new detections
            std::vector<std::tuple<double, double, double>> NED(c_receivers);
            std::vector<double> depth; 

            // TODO: Problem: When only receiver 1 and 2 used, tries to write to row 1 and 2 of a matrix with index 0-1
            for(std::vector<bool>::iterator it = used.begin(); it != used.end(); it++) {
              if(*it) { // Enter if receiver is used
                //inf("%lu", it - used.begin());
                toNEDframe(tagData[it - used.begin()], m_refCoord, NED[it - used.begin()]);

                toNEDframe(tagData[it - used.begin()], m_refCoord, tempNED);
                NED2.row(it - used.begin()) << std::get<0>(tempNED), std::get<1>(tempNED),std::get<2>(tempNED);

                m_newDetection[it - used.begin()] = false;
                depth.push_back(tagData[it - used.begin()].trans_data*0.392); //0.392 from S256 data spec?
              }
            }
            //spew("NED2 made");

            // FOR DEBUGGING
            //for(std::vector<bool>::iterator it = used.begin(); it != used.end(); it++) {
            //  if(*it) { // Enter if receiver is used
            //    debug("%f, %f, %f", std::get<0>(NED[it - used.begin()]), std::get<1>(NED[it - used.begin()]),std::get<2>(NED[it - used.begin()]));
            //  }
            //} 
            //for(std::vector<std::pair<uint8_t, uint8_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
            //  inf("RDOAcombinations %d - %d", it->first, it->second);
            //}
          //inf("NED2 POST");
          //std::cout << "NED2" << std::endl << NED2 << std::endl;
          //inf("RDOA2");
          //std::cout << "RDOA2" << std::endl  << RDOA2 << std::endl;

          // Step 3: Perform position estimate update
          m_xkf.update(NED2.transpose(), RDOA2, depth[0]);
          //spew("Position filter updated");

            logFishPosition();
            //inf("End findFishPos");
          }

          //! Main loop.
          void
          task(void)
          {   
              //inf("%ld, %d, %ld", std::time(nullptr), newTimestamp, std::time(nullptr) - newTimestamp);
              if(std::time(nullptr) - newTimestamp > 2.0) {
                findFishPosition(m_tagDetection);
              }
              // Step 1: Predict
              m_xkf.predict();
              //inf("other side");
          } // End of task function
        };
      }
    }

DUNE_TASK
