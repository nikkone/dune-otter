#include "SingleReceiverEKF.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverEKF::getEstimate() {
    return {ekf.xHat(0),ekf.xHat(1),ekf.xHat(2)};
  }

  void SingleReceiverEKF::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    ekf.A =     A_inn;
    ekf.Q =     Q_inn;
    ekf.PHat = P0_inn;
    ekf.xHat = x0_inn;
    
    ekf.C.resize(2,c_states);
    ekf.ykest.resize(2,1);
    ekf.yk.resize(2,1);
    ekf.R = Eigen::Matrix<double, 2, 2>::Identity();
    ekf.R << rr_cov,0,0,rz_cov;

/*
    ekf.C.resize(1,c_states);
    ekf.ykest.resize(1,1);
    ekf.yk.resize(1,1);
    ekf.R = Eigen::Matrix<double, 1, 1>::Identity();
    ekf.R << rr_cov;
*/
    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    name = "SingleReceiverEKF";
  }
    
  void SingleReceiverEKF::setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn) {
    ekf.xHat = x0_inn;
  }

  bool SingleReceiverEKF::update(TagBuffer *tagBuffer) {
    SingleReceiverBase::update(tagBuffer);
    if(tag_period <= 0) {
      return false;
    }
    if(tagBuffer->tagBuffer.find(receiver) ==tagBuffer->tagBuffer.end()) {
      std::cout << "Did not find receiver: " << receiver << std::endl;
      return false;
    }
    
    const tagBuffer_t *receiverBuffer = tagBuffer->tagBuffer.find(receiver)->second;
        unsigned int m_max_correction_attempts;
    if(max_correction_attempts == 0)
      m_max_correction_attempts = receiverBuffer->max_size()-2;
    else {
      m_max_correction_attempts = max_correction_attempts;
    }
    //inf("Update %ld", c_buffer_size - receiverBuffer->size());
    // Create unix timestamp in milliseconds for the most recent measurement
    
    //tagBuffer_t::const_reverse_iterator it = tagBuffer.find(receiver)->second->rbegin();
    double measurement_millis = receiverBuffer->rbegin()->unix_timestamp + (double)receiverBuffer->rbegin()->millis/1000;

    unsigned int updates = 0;
    unsigned int attempt = 0;
    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    //std::cout << "Steg0"<< std::endl;
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {
      attempt++;
      //inf("%d - %d", receiverBuffer->rbegin()->unix_timestamp, i->unix_timestamp);

      // Time difference of arrival without correcting for period
      double td = measurement_millis - i->unix_timestamp - (double)i->millis/1000;

      // Calculate closest multiple of period between the new measurement and the buffered detection
      double closestMultipleOfPeriod = tag_period*std::round(td/tag_period);

      // Period corrigated time difference of arrival
      double tdoa = td - closestMultipleOfPeriod;

      //inf("delta %f %f", td, closestMultipleOfPeriod);

      // Check if buffered detection satisfies conditions for use in estimator
      if(abs(tdoa) < max_jitter || td > 60.0) {
        //std::cout << "Steg1"<< std::endl;
        // Linear fit of SNR to range
        //double P[2] = {m_args.ranging_snr_fit[0], m_args.ranging_snr_fit[1]};
        //double rangeSNR = (receiverBuffer->rbegin()->snr - P[1])/P[0];
        double rangeSNR = 50; //(DELETE)
        // Range difference of arrival calculation
        double rdoa = c_speed*tdoa;

        // Depth reading from the current tag
        double depth;
        if(depthConversion > 0.01) {
          depth = i->trans_data*depthConversion;
        } else {
          depth = receiver_depth;
        }
        // Compile all mesurements for convenience
        Eigen::Matrix<double, 9, 1> allMeasurements;
        
        allMeasurements << i->N, i->E ,receiver_depth, receiverBuffer->rbegin()->N,receiverBuffer->rbegin()->E ,receiver_depth, rdoa, rangeSNR, depth;


        // Update the algebraic solver
        if (aslv.addMeasurement(allMeasurements)) {
        //std::cout << "Steg2"<< std::endl;
          // Initialize kalman filters with position found with the algebraic solver
          if(!isActive()) {
            setPositionEstimate(aslv.x);
            activateEstimator();
          }
          
          // Log results from algebraic solver
          //double result[3] = {aslv.x(0), aslv.x(1), aslv.x(2)};
          //logResult(result, aslvlogfilename, "OFPASLV");
        }

        // Update kalman filters with current measurement and inputs
        if(isActive()) {
          //std::cout << "Steg3"<< std::endl;
          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = ekf.xHat-allMeasurements.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = ekf.xHat-allMeasurements.block(3,0,3,1); // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||

          if(depthConversion>0.1) { //Use Depth

          // Calculate Jacobian with RDOA and depth
          ekf.C.row(0) = (distance2/r2) - (distance1/r1); // Eq (2.18)
          ekf.C.row(1) << 0, 0, 1;
          
          // Calculate estimated measurements
          ekf.ykest(0) = r2 - r1; // h is eq (2.16) in masters
          ekf.ykest(1) = ekf.xHat(2,0);

          Eigen::Matrix<double, 2, 1> measurements;
          measurements << rdoa ,depth;
          ekf.update(measurements);
          } else {
          // Calculate Jacobian with RDOA only
          ekf.C.row(0) = (distance2/r2) - (distance1/r1); // Eq (2.18)
          
          // Calculate estimated measurements
          ekf.ykest(0) = r2 - r1; // h is eq (2.16) in masters

          Eigen::Matrix<double, 1, 1> measurements;
          measurements << rdoa;
          ekf.update(measurements);
          }

          

          updates++;
          // Stop the loop after using the new measurement a given number of times.
          if(updates >= max_updates_per_new_measurement) {
            return true; 
          }
    ////std::cout << "r1" << std::endl << r1 << std::endl;
    ////std::cout << "r2" << std::endl << r2 << std::endl;
    ////std::cout << "ykest" << std::endl << ekf.ykest << std::endl;
    ////std::cout << "r1" << std::endl << receiverBuffer->rbegin()->recv_mem_addr << std::endl;
    ////std::cout << "r2" << std::endl << i->recv_mem_addr << std::endl; 
        }
        // Stop after a number of predetermined attempts
        if(attempt >= m_max_correction_attempts) {
          //war("Stopped because maximum correction attempts reached. Attempts: %u, Updates %d", attempt, updates);
          return false;
        }
      }
    }
    return false;
  }
  
  void SingleReceiverEKF::predict() {
    ekf.predict();
  }
  void SingleReceiverEKF::print(std::ostream& os) const {
    os << "A" << std::endl << ekf.A << std::endl;
    os << "C" << std::endl << ekf.C << std::endl;
    os << "PHat" << std::endl << ekf.PHat << std::endl;
    os << "xHat" << std::endl << ekf.xHat << std::endl;
    os << "R" << std::endl << ekf.R << std::endl;
    os << "Q" << std::endl << ekf.Q << std::endl;
    os << "K" << std::endl << ekf.K << std::endl;
    os << "ykest" << std::endl << ekf.ykest << std::endl;
    os << "yk" << std::endl << ekf.yk << std::endl;
    os << "innov" << std::endl << ekf.innov << std::endl;
  }
}