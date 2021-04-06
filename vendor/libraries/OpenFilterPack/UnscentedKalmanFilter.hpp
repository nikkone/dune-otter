#ifndef OFP_UnscentedKalmanFilter
#define OFP_UnscentedKalmanFilter
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <iostream> // For debugging, remove after

namespace OFP
{
  template <class T, int states, int measurements>
  class UnscentedKalmanFilter
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

      //! Kalman Gain
      Eigen::Matrix<T, states, measurements> K;
      //! State Covariance estimate matric
      Eigen::Matrix<T, states, states> PHat;
      //! Process noise covariance matrix
      Eigen::Matrix<T, states, states> Q;
      //! Measurement noise covariance matrix
      Eigen::Matrix<T, measurements, measurements> R;
      //! Time varying innovation vector
      //Eigen::Matrix<T, measurements, 1> innovation;
      //! State estimate vector
      Eigen::Matrix<T, states, 1> xHat;
      //! The size of sigma points for use in code
      const int size_sigmaPoints = 2*states+1;
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
      UnscentedKalmanFilter(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        computeWeights(); 
        active = false;
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
        computeWeights();
      }

      //! Measurment update for the filter. This is dependent on the measurment function being set.
      //! @param[in] yk_in Measurment vector.
      //! @return True if active and end reached, false if filter not active.
      bool update(Eigen::Matrix<T, measurements, 1> yk_in) {
        if(active) {
          // Update sigma points to reflect the prediction 
          
          sigmaPoints = generateSigmaPoints(xHat, PHat);
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_h.col(s) =  h(sigmaPoints.col(s));
          }
          

          Eigen::Matrix<T, measurements, 1> yk_est;
          Eigen::Matrix<T, measurements, measurements> Py = unscented_transform<measurements>(sigmaPoints_h, Wm, Wc, yk_est) + R;
          
          Eigen::Matrix<T, states, measurements> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

          K = Pxy * Py.inverse();
          xHat = xHat + K*(yk_in - yk_est);
          PHat = PHat - K*Pxy.transpose();
          //PHat = PHat - K*Py*K.transpose();

          return true;
        }
        return false;
      }

      //! Time update for the filter. This is dependent on the state propagation function being set.
      //! @return True if active and end reached, false if filter not active.
      bool predict() {
        if(active) {
          // Compute process sigma points
          sigmaPoints = generateSigmaPoints(xHat, PHat);
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_f.col(s) =  f(sigmaPoints.col(s));
          }
          // Unscented transform
          PHat = unscented_transform<states>(sigmaPoints_f, Wm, Wc, xHat) + Q;
          return true;
        }
        return false;
      }

    private:
      //! 
      T alpha;
      //!
      T beta;
      //!
      T kappa;
      //!
      T lambda;

      //! Calculates the mean and covariance weights for the unscented transform.
      void computeWeights() {
        T c = 1 / (nx + lambda);
        Wc = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wc[0] = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm[0] = lambda *c; // (36)
      }

      //! Generates sigma points for given state mean and covariance.
      //! @param[in] x State vector.
      //! @param[in] P State covariance matrix. 
      //! @return The generated sigma points
      Eigen::Matrix<T, states, 2*states+1> generateSigmaPoints(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, states, states> P) {
        Eigen::Matrix<T, states, 2*states+1> sigmaPoints_ret = Eigen::Matrix<T, states, 2*states+1>::Zero();
        sigmaPoints_ret.col(0) = x;
        Eigen::Matrix<T, states, states> U = ((lambda+nx)*P).llt().matrixL();

        for(int k = 0;k<nx; k++) {
          sigmaPoints_ret.col(k+1)  =  x + U.col(k); // (32)
          sigmaPoints_ret.col(nx+k+1) = x - U.col(k); // (33)
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

      //! Calculate mean and covariance from the sigma points and weights.
      //! @param[out] x_out The calculated mean vector.
      //! @param[in] sigmas_in Sigma points for the state vector.
      //! @param[in] wm_in Mean weights.
      //! @param[in] wc_in Covariance weights.
      //! @return The calculated covariance matrix
      template<int dimensions>
      Eigen::Matrix<T, dimensions, dimensions> unscented_transform(Eigen::Matrix<T, dimensions, 2*states+1> sigmas_in, Eigen::Matrix<T, 1, 2*states+1> wm_in, Eigen::Matrix<T, 1, 2*states+1> wc_in, Eigen::Matrix<T, dimensions, 1> &x_out) {
        x_out = (wm_in*sigmas_in.transpose()).colwise().sum();
        sigmas_in = sigmas_in.colwise() - x_out;
        Eigen::Matrix<T, dimensions, dimensions> P = Eigen::Matrix<T, dimensions, dimensions>::Zero();
        for(int s = 0;s<size_sigmaPoints; s++) {
          //P += sigmas_in.col(s) * sigmas_in.col(s).transpose();
          P += wc_in(0,s) * sigmas_in.col(s) * sigmas_in.col(s).transpose();
        }
        return P;
      }
  };
}
#endif //OFP_UnscentedKalmanFilter