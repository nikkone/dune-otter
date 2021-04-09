#ifndef OFP_SquareRootUnscentedKalmanFilter
#define OFP_SquareRootUnscentedKalmanFilter
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <iostream> // For debugging, remove after

namespace OFP
{
  // Based on Van der Merwe an WAN - 2001 - The square-root unscented Kalman Filter for state and parameter-estimation
  template <class T, int states, int measurements>
  class SquareRootUnscentedKalmanFilter
  {
    public:
      //! Boolean to be set when filter is initalized. Answers "Is the filter active/initialized/running?"
      bool active;
      //! Filter timestep
      T dt;
      //! Number of states
      const int nx = states;
      //! Number of measurements
      const int ny = measurements;
      //! The size of sigma points for use in code
      const int size_sigmaPoints = 2*states+1;
      //! Kalman Gain
      Eigen::Matrix<T, states, measurements> K;
      //! Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, states, states> S;
      //! Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, states, states> Q_sqrt;
      //! Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, measurements, measurements> R_sqrt;
      //! State estimate vector
      Eigen::Matrix<T, states, 1> xHat;
      //! Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints;
      //! Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_f;
      //! Sigma points for the Unscented transform.
      Eigen::Matrix<T, measurements, 2*states+1> sigmaPoints_h;
      //! Sigma point weights for the Unscented transform covariance.
      Eigen::Matrix<T, 1, 2*states+1> Wc;
      //! Sigma point weights for the Unscented transform mean.
      Eigen::Matrix<T, 1, 2*states+1> Wm;
      //! Holder for state transition function
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, states, 1> x)> f;
      //! Holder for measurment function
      std::function<Eigen::Matrix<T, measurements, 1>(Eigen::Matrix<T, states, 1> x)> h;

      //! Constructor that configures the variables needed for the unscented transform.
      //! @param[in] alpha_in.
      //! @param[in] beta_in.
      //! @param[in] kappa_in.
      SquareRootUnscentedKalmanFilter(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        gamma = std::sqrt(nx+lambda);
        computeWeights();
      }

      //! Function for changing the parameters used in the unscented transform.
      //! @param[in] alpha_in.
      //! @param[in] beta_in.
      //! @param[in] kappa_in.    
      void setUnscentedParameters(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        gamma = std::sqrt(nx+lambda);
        computeWeights();
      }

      void setInitialCovariance(Eigen::Matrix<T, states, states> P0) {
        S = P0.llt().matrixU(); //Upper triangular cholesky
      }

      void setMeasurmentCovariance(Eigen::Matrix<T, measurements, measurements> R_in) {
        R_sqrt = R_in.llt().matrixU(); //Upper triangular cholesky
      }

      void setProcessCovariance(Eigen::Matrix<T, states, states> Q_in) {
        Q_sqrt = Q_in.llt().matrixU(); //Upper triangular cholesky
      }

      //! Measurment update for the filter. This is dependent on the measurment function being set.
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
      bool update(Eigen::Matrix<T, measurements, 1> yk_inn) {
        if(active) {
          // Update sigma points to reflect the prediction
          sigmaPoints = generateSigmaPoints(xHat, S);
          // Propagate the sigma points through the measurment model. 
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_h.col(s) =  h(sigmaPoints.col(s));
          }
          
          Eigen::Matrix<T, measurements, 1> yk_est;
          // Unscented transform - Calculate the a priori estimate mean
          yk_est = sigmaPoints_h*Wm.transpose();
          
          //yk_est = (sigmaPoints_h*Wm.transpose()).colwise().sum();

          // Unscented transform - Calculate the a priori estimate Covariance
          Eigen::Matrix<T, measurements, 2*states+1> sigmaDelta = sigmaPoints_h.colwise() - yk_est;

          Eigen::Matrix<T, measurements, 2*states+measurements> QR;
          QR << (std::sqrt(Wc(0,1))*sigmaDelta.block(0,1,measurements, 2*states)), R_sqrt;

          Eigen::Matrix<T, measurements, measurements> Sy = QR.transpose().householderQr().matrixQR().topLeftCorner(measurements, measurements).template triangularView<Eigen::Upper>();
          Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(Sy, sigmaDelta.col(0), Wc(0,0));
          Sy.transposeInPlace();
          Eigen::Matrix<T, states, measurements> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

          K = (Pxy * Sy.inverse().transpose())*Sy.inverse();
          xHat = xHat + K*(yk_inn - yk_est);
          Eigen::Matrix<T, states, measurements> U = K*Sy;
         
          for(int i = 0; i < measurements; i++) {
            Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, U.col(i), T(-1.0));
          }
          return true;
        }
        return false;
      }

      //! Time update for the filter. This is dependent on the state propagation function being set.
      //! @return True if active and end reached, false if filter not active.
      bool predict() {
        if(active) {

          // Compute sigma points
          sigmaPoints = generateSigmaPoints(xHat, S);

          // Propagate the sigma points through the process model. 
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_f.col(s) =  f(sigmaPoints.col(s));
          }
          // Unscented transform - Calculate the a priori estimate mean
          xHat = (Wm*sigmaPoints_f.transpose()).colwise().sum();
          // Unscented transform - Calculate the a priori estimate Covariance 
          Eigen::Matrix<T, states, 2*states+1> sigmaDelta = sigmaPoints_f.colwise() - xHat;

          Eigen::Matrix<T, states, 3*states>  QR;
          QR << (std::sqrt(Wc(0,1))*sigmaDelta.rightCols(2*states)), Q_sqrt;
    
          S = QR.transpose().householderQr().matrixQR().topLeftCorner(states,states).template  triangularView<Eigen::Upper>();
          // Cholesky update
          Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, sigmaDelta.col(0), Wc(0,0));
          S.transposeInPlace();
          return true;
        }
        return false;
      }

/*
      //! Time update for the filter.
      //! @return True if active and end reached, false if filter not active.
      bool predict2() {
        if(active) {
          Eigen::Matrix<double, 3, 3> A;
          A << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;
          //Predict Step / Project ahead
          xHat = A*xHat;
          PHat = A*(S.transpose()*S)*A.transpose() + Q;
          S = PHat.llt().matrixU(); //Upper triangular cholesky
          return true;
        }
        return false;
      }
*/
    private:
      //! 
      T alpha;
      //!
      T beta;
      //!
      T kappa;
      //!
      T lambda;
      //!
      T gamma;

      //! Calculates the mean and covariance weights for the unscented transform.
      void computeWeights() {
        T c = 1 / (nx + lambda);
        Wc = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wc(0, 0) = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm(0, 0) = lambda *c; // (36)
      }

      //! Generates sigma points for given state mean and covariance.
      //! @param[in] x State vector.
      //! @param[in] P State covariance matrix. 
      //! @return The generated sigma points
      Eigen::Matrix<T, states, 2*states+1> generateSigmaPoints(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, states, states> S_in) {
        Eigen::Matrix<T, states, 2*states+1> sigmaPoints_ret = Eigen::Matrix<T, states, 2*states+1>::Zero();
        sigmaPoints_ret.col(0) = x;

        for(int k = 0;k<nx; k++) {
          sigmaPoints_ret.col(k+1)  =   x + gamma*S_in.col(k); // (32)
          sigmaPoints_ret.col(nx+k+1) = x - gamma*S_in.col(k); // (33)
        }
        return sigmaPoints_ret;
      }

      //! Calculate the cross-covariance of the state and estimated state based on measurements.
      //! @param[in] x_in State vector.
      //! @param[in] z_in Estimated state vector.
      //! @param[in] sigmas_f Sigma points for the state vector.
      //! @param[in] sigmas_h Sigma points propagated through the measurment model.
      //! @param[in] wc_in Covariance weights.
      //! @return The cross-covariance between state and estimated state based on measurements.
      Eigen::Matrix<T, states, measurements> calculate_cross_variance(Eigen::Matrix<T, states, 1>  x_in, Eigen::Matrix<T, measurements, 1>  z_in, Eigen::Matrix<T, states, 2*states+1> sigmas_f, Eigen::Matrix<T, measurements, 2*states+1> sigmas_h, Eigen::Matrix<T, 1, 2*states+1> wc_in) {
        sigmas_f = sigmas_f.colwise() - x_in;
        sigmas_h = sigmas_h.colwise() - z_in;

        Eigen::Matrix<T,states,measurements> Pxz = Eigen::Matrix<T,states,measurements>::Zero();
        for(int s = 0;s<size_sigmaPoints; s++) {
            Pxz += wc_in(0,s) * sigmas_f.col(s) * sigmas_h.col(s).transpose();
        }
        return Pxz;
      }
  };
}
#endif //OFP_SquareRootUnscentedKalmanFilter