#ifndef DUNE_SOURCEESTIMATORSEIGEN_KF
#define DUNE_SOURCEESTIMATORSEIGEN_KF
#include <Eigen/Core>
#include <Eigen/Dense>
  namespace SourceEstimators
  {
      namespace SingleSourceEigen
      {
        template <class T, int states, int measurements>
        class KalmanFilter
        {
          public:
          bool active;
          T dt; // Timestep

          const int nx = states;                    // Num states
          const int ny = measurements;              // Num outputs
          //const int nu = measurements;              // Num inputs
          Eigen::Matrix<T, states, states> A;       // State transition matrix
          Eigen::Matrix<T, states, states> B;       // Input matrix
          Eigen::Matrix<T, states, states> C;       // Observation matrix
          Eigen::Matrix<T, states, states> D;       // unknown input matrix
          Eigen::Matrix<T, states, states> K;       // State Covariance estimate matric
          Eigen::Matrix<T, states, states> PHat;    // State Covariance estimate matric
          Eigen::Matrix<T, states, states> Q;       // Process noise covariance matrix
          Eigen::Matrix<T, states, states> R;       // Measurement noise covariance matrix
          Eigen::Matrix<T, states, 1> innov;        // Time varying innovation vector
          Eigen::Matrix<T, states, 1> yk;           // Measurement vector
          Eigen::Matrix<T, states, 1> ykest;        // Estimate of the measurements
          Eigen::Matrix<T, states, 1> xHat;         // State estimate vector
          KalmanFilter() {
          }

          void update(Eigen::Matrix<T, states, 1> zk) {

            // Compute Kalman gain
            K = PHat * C.transpose() * (C*PHat*C.transpose() + R).inverse();

            //Update estimate with measurement
            innov = zk - ykest;
            xHat = xHat + K*innov;

            // Compute error covariance for updated estimate
            PHat = (Eigen::Matrix<T, states, states>::Identity() - K*C)*PHat;

          }

          void predict() {
	        if(active) {
                //Predict Step / Project ahead
                xHat = A*xHat;
                PHat = A*PHat*A.transpose() + Q;
            }
          }
        };
      }
  }
#endif //DUNE_SOURCEESTIMATORSEIGEN_KF