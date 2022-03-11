#include "SingleReceiverUKF.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverUKF::getEstimate() {
    return {ukf.xHat(0),ukf.xHat(1),ukf.xHat(2)};
  }
  typedef enum 
  { 
    param_receiver,
    param_receiver_depth,
    param_tag_period,
    param_max_jitter,
    param_max_updates_per_new_measurement,
    param_max_correction_attempts
  } t_param;

  void SingleReceiverUKF::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    name = "SingleReceiverUKF";
       ukf.h = [this](Eigen::Matrix<double, 3, 1> x) {
          // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
          Eigen::Matrix<double, 3, 1> distance1 = x-this->pos_previous;//z.block(0,0,3,1); // X_e-X_rx0
          Eigen::Matrix<double, 3, 1> distance2 = x-this->pos_current;//z.block(3,0,3,1);  // X_e-X_rx1
          double r1 = distance1.norm();//  ||X_e-X_rx0||
          double r2 = distance2.norm();// ||X_e-X_rx1||
          
          // Calculate estimated measurements
          Eigen::Matrix<double, 2, 1> ykest;
          ykest(0) = r2 - r1; // h is eq (2.16) in masters
          ykest(1) = x(2); // Depth estimate
          //std::cout << this->pos_current << std::endl;
          return ykest;
        };

        ukf.f = [A_inn](Eigen::Matrix<double, 3, 1> x) {

          return A_inn*x;
        };

        ukf.Q = Q_inn;
        ukf.R = Eigen::Matrix<double, 2, 2>::Identity();
        ukf.R << rr_cov,0,0,rz_cov;
        ukf.PHat = P0_inn;
        ukf.xHat = x0_inn;
        //ukf.dt = m_args.filter_timestep;

    registerParameter("receiver");
    registerParameter("receiver_depth");
    registerParameter("tag_period");
    registerParameter("max_jitter");
    registerParameter("max_updates_per_new_measurement");
    registerParameter("max_correction_attempts");

    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }
    
  void SingleReceiverUKF::setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn) {
    ukf.xHat = x0_inn;
  }

  void SingleReceiverUKF::parseParameter(unsigned parameterID, double value) {
    switch(parameterID) {
      case param_receiver:
        receiver = value;
        break;
      case param_receiver_depth:
        receiver_depth = value;
        break;
      case param_tag_period:
        tag_period = value;
        break;
      case param_max_jitter:
        max_jitter = value;
        break;
      case param_max_updates_per_new_measurement:
        max_updates_per_new_measurement = value;
        break;
      case param_max_correction_attempts:
        max_correction_attempts = value;
        break;
      default:
        break;
    }
  }

  void SingleReceiverUKF::update(TagBuffer *tagBuffer) {
    if(tagBuffer->tagBuffer.find(receiver) !=tagBuffer->tagBuffer.end()) {

      
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
          double depth = i->trans_data*0.392;

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
             pos_previous = allMeasurements.block(0,0,3,1); // X_e-X_rx0
             pos_current = allMeasurements.block(3,0,3,1); // X_e-X_rx1

            Eigen::Matrix<double, 2, 1> measurements;
            measurements << rdoa ,depth;
            ukf.update(measurements);

            updates++;
            // Stop the loop after using the new measurement a given number of times.
            if(updates >= max_updates_per_new_measurement) {
              return; 
            }
            
          }
          // Stop after a number of predetermined attempts
          if(attempt >= m_max_correction_attempts) {
            //war("Stopped because maximum correction attempts reached. Attempts: %u, Updates %d", attempt, updates);
            return;
          }
        }
      }
      //war("No good TDOA value found");
    }
  }
  
  void SingleReceiverUKF::predict() {
    ukf.predict();
  }
  void SingleReceiverUKF::print(std::ostream& os) const {
    //os << "A" << std::endl << ukf.A << std::endl;
    //os << "C" << std::endl << ukf.C << std::endl;
    os << "PHat" << std::endl << ukf.PHat << std::endl;
    os << "xHat" << std::endl << ukf.xHat << std::endl;
    os << "R" << std::endl << ukf.R << std::endl;
    os << "Q" << std::endl << ukf.Q << std::endl;
    //os << "K" << std::endl << ukf.K << std::endl;
    //os << "ykest" << std::endl << ukf.ykest << std::endl;
    //os << "yk" << std::endl << ukf.yk << std::endl;
    //os << "innov" << std::endl << ukf.innov << std::endl;
  }
}