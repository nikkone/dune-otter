#ifndef OFP_KALMANFILTERDYNAMIC
#define OFP_KALMANFILTERDYNAMIC
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
namespace OFP
{
  template <class T, int states>
  class KalmanFilterDynamic
  {
    public:
      //! Boolean to be set when filter is initalized. Answers "Is the filter active/initialized/running?"
      bool active;
      //! Filter Timestep.
      T dt;
      //! Number of states.
      const int nx = states;
      //! Number of outputs.
      int measurements;
      //! Number of inputs.
      int inputs;
      //! State transition matrix.
      Eigen::Matrix<T, states, states> A;
      //! Input matrix.
      Eigen::Matrix<T, states, Eigen::Dynamic> B;
      //! Observation matrix.
      Eigen::Matrix<T, Eigen::Dynamic, states> C;
      //! Unknown input matrix.
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> D;
      //! Kalman gain.
      Eigen::Matrix<T, states, Eigen::Dynamic> K;
      //! State Covariance estimate matric.
      Eigen::Matrix<T, states, states> PHat;
      //! Process noise covariance matrix.
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Q;
      //! Measurement noise covariance matrix.
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
      //! Time varying innovation vector.
      Eigen::Matrix<T, Eigen::Dynamic, 1> innov;
      //! Measurement vector.
      Eigen::Matrix<T, Eigen::Dynamic, 1> yk;
      //! Estimate of the Eigen::Dynamic.
      Eigen::Matrix<T, Eigen::Dynamic, 1> ykest;
      //! State estimate vector.
      Eigen::Matrix<T, states, 1> xHat;

      //! Filter constructor.
      KalmanFilterDynamic() : active(false){
        //std::cout << std::endl<< std::endl<< "KAlman initializws" << std::endl << std::endl;
      }

      //! Initializer for the filter
      //! @param [in] A_inn State transition matrix
      //! @param [in] Q_inn Measurement covariance matrix
      //! @param [in] P0_inn Initial covarinace matrix
      //! @param [in] x0_inn Initial state
      void initialize(const Eigen::Matrix<T, states, states> &A_inn,
                      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Q_inn,
                      const Eigen::Matrix<T, states, states> &P0_inn,
                      const Eigen::Matrix<T, states, 1> &x0_inn) {
        A = A_inn;
        Q = Q_inn;
        PHat = P0_inn;
        xHat = x0_inn;
        D = Eigen::Matrix<T, states, states>::Identity();
      }


/*      Eigen::Matrix<T, Eigen::Dynamic, 1>
      calculateInnovation(Eigen::Matrix<T, Eigen::Dynamic, 1> yk_in, Eigen::Matrix<T, Eigen::Dynamic, 1> u) {
        return yk_in - C*xHat - D*u;
        //return Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero();
      }*/

      Eigen::Matrix<T, Eigen::Dynamic, 1>
      estimateMeasurement(const Eigen::Matrix<T, Eigen::Dynamic, 1> &u) {
        ykest = C*xHat - D*u;
        return ykest;
      }

      //! Measurment update for the filter with inputs. (corrector)
      //! @param[in] yk_in Measurment vector.
      //! @param[in] u Input vector.
      //! @return True if active and end reached, false if filter not active.

      bool update(const Eigen::Matrix<T, Eigen::Dynamic, 1> &yk_in) {
        if(active) {
        //std::cout << "yk_in" << std::endl << yk_in << std::endl;
        //std::cout << "ykest" << std::endl << ykest << std::endl;
          // Compute Kalman gain
          //std::cout << "C*PHat*C.transpose()" << std::endl << C*PHat*C.transpose() << std::endl;
          //std::cout << "R" << std::endl << R << std::endl;
          K = PHat * C.transpose() * (C*PHat*C.transpose() + R).inverse();

          //Update estimate with measurement
          yk=yk_in;
          innov = yk_in - ykest;
          //innov = yk_in - C*xHat - D*u;
          //std::cout << "innov" << std::endl << innov << std::endl;
          //std::cout << "K" << std::endl << K << std::endl;
          xHat = xHat + K*innov;//calculateInnovation(yk_in, u);

          // Compute error covariance for updated estimate
          //PHat = (Eigen::Matrix<T, states, states>::Identity() - K*C)*PHat;  // Simplified, but only valid for optimal K
          PHat = (Eigen::Matrix<T, states, states>::Identity() - K*C)*PHat*
                 (Eigen::Matrix<T, states, states>::Identity() - K*C).transpose()
                 + K*R*K.transpose();

          return true;
        }
        return false;
      }

// UPDATE TO INCLUDE B
      //! Time update for the filter.
      //! @return True if active and end reached, false if filter not active.
      bool predict() {
        if(active) {
          //Predict Step / Project ahead
          xHat = A*xHat;
          PHat = A*PHat*A.transpose() + D*Q*D.transpose();
          return true;
        }
        return false;
      }
  };
}
#endif //OFP_KALMANFILTERDYNAMIC