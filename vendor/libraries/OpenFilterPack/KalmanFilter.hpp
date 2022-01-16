#ifndef OFP_KALMANFILTER
#define OFP_KALMANFILTER
#include <Eigen/Core>
#include <Eigen/Dense>
namespace OFP
{
  template <class T, int states, int measurements, int inputs>
  class KalmanFilter
  {
    public:
      //! Boolean to be set when filter is initalized. Answers "Is the filter active/initialized/running?"
      bool active;
      //! Filter Timestep.
      T dt;
      //! Number of states.
      const int nx = states;
      //! Number of outputs.
      const int ny = measurements;
      //! Number of inputs.
      const int nu = inputs;
      //! State transition matrix.
      Eigen::Matrix<T, states, states> A;
      //! Input matrix.
      Eigen::Matrix<T, states, inputs> B;
      //! Observation matrix.
      Eigen::Matrix<T, measurements, states> C;
      //! Unknown input matrix.
      Eigen::Matrix<T, measurements, inputs> D;
      //! State Covariance estimate matric.
      Eigen::Matrix<T, states, measurements> K;
      //! State Covariance estimate matric.
      Eigen::Matrix<T, states, states> PHat;
      //! Process noise covariance matrix.
      Eigen::Matrix<T, states, states> Q;
      //! Measurement noise covariance matrix.
      Eigen::Matrix<T, measurements, measurements> R;
      //! Time varying innovation vector.
      Eigen::Matrix<T, measurements, 1> innov;
      //! Measurement vector.
      //Eigen::Matrix<T, measurements, 1> yk;
      //! Estimate of the measurements.
      Eigen::Matrix<T, measurements, 1> ykest;
      //! State estimate vector.
      Eigen::Matrix<T, states, 1> xHat;

      //! Filter constructor.
      KalmanFilter() {
        active = false;
      }
/*
      template <int F = inputs>
      typename std::enable_if<F == 0, Eigen::Matrix<T, measurements, 1>>::type
      calculateInnovation(Eigen::Matrix<T, measurements, 1> yk_in) {
        return yk_in - C*xHat;
      }

      template <int F = inputs>
      typename std::enable_if<F != 0, Eigen::Matrix<T, measurements, 1>>::type
      calculateInnovation(Eigen::Matrix<T, measurements, 1> yk_in, Eigen::Matrix<T, inputs, 1> u) {
        return yk_in - C*xHat - D*u;
        //return Eigen::Matrix<T, measurements, 1>::Zero();
      }
*/
      //! Measurment update for the filter without inputs. (corrector)
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
      template <int F = inputs>
      typename std::enable_if<F == 0, bool>::type
      update(const Eigen::Matrix<T, measurements, 1> &yk_in) {
        if(active) {
          // Compute Kalman gain
          K = PHat * C.transpose() * (C*PHat*C.transpose() + R).inverse();

          //Update estimate with measurement
          innov = yk_in - ykest;
          //innov = yk_in - C*xHat;

          xHat = xHat + K*innov;//calculateInnovation(yk_in);

          // Compute error covariance for updated estimate
          PHat = (Eigen::Matrix<T, states, states>::Identity() - K*C)*PHat;
          return true;
        }
        return false;
      }

      //! Measurment update for the filter with inputs. (corrector)
      //! @param[in] yk_in Measurment vector.
      //! @param[in] u Input vector.
      //! @return True if active and end reached, false if filter not active.
      template <int F = inputs>
      typename std::enable_if<F != 0, bool>::type
      update(const Eigen::Matrix<T, measurements, 1> &yk_in, const Eigen::Matrix<T, inputs, 1> &u) {
        if(active) {
          // Compute Kalman gain
          K = PHat * C.transpose() * (C*PHat*C.transpose() + R).inverse();

          //Update estimate with measurement
          innov = yk_in - ykest;
          //innov = yk_in - C*xHat - D*u;
          xHat = xHat + K*innov;//calculateInnovation(yk_in, u);

          // Compute error covariance for updated estimate
          PHat = (Eigen::Matrix<T, states, states>::Identity() - K*C)*PHat;
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
          PHat = A*PHat*A.transpose() + Q;
          return true;
        }
        return false;
      }
  };
}
#endif //OFP_KALMANFILTER