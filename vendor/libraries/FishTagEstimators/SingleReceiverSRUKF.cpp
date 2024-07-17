#include "SingleReceiverSRUKF.hpp"
namespace FishTagEstimators
{
  std::tuple<double, double, double> SingleReceiverSRUKF::getEstimate() const {
    return {srukf.xHat(0),srukf.xHat(1),srukf.xHat(2)};
  }

  void SingleReceiverSRUKF::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    A        = A_inn;
    srukf.xHat = x0_inn;
    srukf.h = [this](Eigen::Matrix<double, 3, 1> x) {
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

    srukf.f = [this](Eigen::Matrix<double, c_states, 1> x) {
      return Eigen::Matrix<double, c_states, 1>(A*x);
    };

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity();
    R << rr_cov,0,0,rz_cov;
    srukf.setMeasurmentCovariance(R);
    srukf.setInitialCovariance(P0_inn);
    srukf.setProcessCovariance(Q_inn);

    SingleReceiverBase::initialize(A_inn, Q_inn, P0_inn, x0_inn);
    registerParameter("unscented_alpha", param_unscented_alpha);
    registerParameter("unscented_beta", param_unscented_beta);
    registerParameter("unscented_kappa", param_unscented_kappa);
    name = "SingleReceiverSRUKF";
  }
    
  void SingleReceiverSRUKF::setPositionEstimate(const Eigen::Matrix<double, c_states, 1> &x0_inn) {
    srukf.xHat = x0_inn;
  }

  void SingleReceiverSRUKF::parseParameter(unsigned parameterID, double value) {
    switch(parameterID) {
      case param_unscented_alpha:
        srukf.setAlpha(value);
        break;
      case param_unscented_beta:
        srukf.setBeta(value);
        break;
      case param_unscented_kappa:
        srukf.setKappa(value);
        break;
      default:
        SingleReceiverBase::parseParameter(parameterID, value);
        break;
    }
  }

  bool SingleReceiverSRUKF::update(TagBuffer *tagBuffer) {
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
    // Create unix timestamp in milliseconds for the most recent measurement
    long int measurement_ms = (long int)receiverBuffer->rbegin()->unix_timestamp*1000 + receiverBuffer->rbegin()->millis;

    unsigned int updates = 0;
    unsigned int attempt = 0;
    // Check the buffer of older tag detections from the second newest to the oldest.
    // Only combine if a multiple of the period is found within a given threashold/jitter.
    //std::cout << "Steg0"<< std::endl;
    for(tagBuffer_t::const_reverse_iterator i=receiverBuffer->rbegin()+1; i != receiverBuffer->rend();i++) {
      attempt++;

      // Time difference of arrival without correcting for period
      long int rawTDOA_ms = measurement_ms - (long int)i->unix_timestamp*1000 - i->millis;

      long int tempTDOA_ms;
      if(interval_mode == interval_mode_fixed_unknown) {
        tempTDOA_ms = rawTDOA_ms - std::round((double)rawTDOA_ms/1000)*1000;
      } else { // Fixed period corrigated time difference of arrival
      long int closestMultipleOfPeriod_ms = tag_period*1000*std::round(rawTDOA_ms/(tag_period*1000));
        tempTDOA_ms = rawTDOA_ms - closestMultipleOfPeriod_ms;
      }

      // Check if buffered detection satisfies conditions for use in estimator
      if(std::abs(tempTDOA_ms) < max_jitter*1000 && std::abs(tempTDOA_ms) > 0) {
        //std::cout << "Steg1"<< std::endl;
        // Linear fit of SNR to range
        //double P[2] = {m_args.ranging_snr_fit[0], m_args.ranging_snr_fit[1]};
        //double rangeSNR = (receiverBuffer->rbegin()->snr - P[1])/P[0];
        double rangeSNR = 50; //(DELETE)
        // Range difference of arrival calculation
        double rdoa = c_speed*tempTDOA_ms/1000;

        // Depth reading from the current tag
        double depth;
        if(tagBuffer->getDepthConversionCoefficient() > 0.01) {
          depth = i->trans_data*tagBuffer->getDepthConversionCoefficient();
        } else {
          depth = receiver_depth;
        }
        // Compile all mesurements for convenience
        Eigen::Matrix<double, 9, 1> allMeasurements;
        
        allMeasurements << i->N, i->E ,receiver_depth, receiverBuffer->rbegin()->N,receiverBuffer->rbegin()->E ,receiver_depth, rdoa, rangeSNR, depth;

        if(useAslv) {
          if(!isActive()) {
          // Update the algebraic solver
            if (aslv.addMeasurement(allMeasurements)) {
              // Initialize kalman filters with position found with the algebraic solver
              setPositionEstimate(aslv.x);
              activateEstimator();
            }
          }
        } else {
          activateEstimator();
        }

        // Update kalman filters with current measurement and inputs
        if(isActive()) {
          pos_previous = allMeasurements.block(0,0,3,1); // X_e-X_rx0
          pos_current = allMeasurements.block(3,0,3,1); // X_e-X_rx1

          Eigen::Matrix<double, 2, 1> measurements;
          measurements << rdoa ,depth;
          srukf.update(measurements);

          updates++;
          // Stop the loop after using the new measurement a given number of times.
          if(updates >= max_updates_per_new_measurement) {
             return true; 
          }       
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
  
  void SingleReceiverSRUKF::predict() {
    if(!isActive()) {
      srukf.predict();
    }
  }
  void SingleReceiverSRUKF::print(std::ostream& os) const {
    //os << "A" << std::endl << srukf.A << std::endl;
    //os << "C" << std::endl << srukf.C << std::endl;
    //os << "PHat" << std::endl << srukf.PHat << std::endl;
    os << "xHat" << std::endl << srukf.xHat << std::endl;
    //os << "R" << std::endl << srukf.R << std::endl;
    //os << "Q" << std::endl << srukf.Q << std::endl;
    //os << "K" << std::endl << srukf.K << std::endl;
    //os << "ykest" << std::endl << srukf.ykest << std::endl;
    //os << "yk" << std::endl << srukf.yk << std::endl;
    //os << "innov" << std::endl << srukf.innov << std::endl;
  }
}