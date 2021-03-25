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
      bool active;
      T dt; // Timestep
      const int nx = states;                                 // Num states
      const int ny = measurements;                           // Num outputs
      const int size_sigmaPoints = 2*states+1;               // Num columns of sigmaPoints
      Eigen::Matrix<T, states, measurements> K;              // State Covariance estimate matrix
      Eigen::Matrix<T, states, states> S;                    // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, states, states> Q_sqrt;               // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, measurements, measurements> R_sqrt;   // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, states, 1> xHat;                      // State estimate vector
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints;      // Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_f;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_h;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, 1, 2*states+1> Wc;                    // Sigma point weights for the Unscented transform covariance.
      Eigen::Matrix<T, 1, 2*states+1> Wm;                    // Sigma point weights for the Unscented transform mean.
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, states, 1> x)> f;
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, measurements, 1> x, Eigen::Matrix<T, 9, 1> z)> h;

      SquareRootUnscentedKalmanFilter(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        gamma = std::sqrt(nx+lambda);
        computeWeights();
      }

      void setInitialCovariance(Eigen::Matrix<T, states, states> P0) {
        S = P0.llt().matrixL(); //Upper triangular cholesky
      }

      void setMeasurmentCovariance(Eigen::Matrix<T, states, states> R_in) {
        R_sqrt = R_in.llt().matrixL(); //Upper triangular cholesky
      }

      void setProcessCovariance(Eigen::Matrix<T, states, states> Q_in) {
        Q_sqrt = Q_in.llt().matrixL(); //Upper triangular cholesky
      }
      void computeWeights() {
        T c = 1 / (nx + lambda);
        Wc = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wc[0] = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm[0] = lambda *c; // (36)
      }

      Eigen::Matrix<T, states, 2*states+1> generateSigmaPoints(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, states, states> S_in) {
        Eigen::Matrix<T, states, 2*states+1> sigmaPoints_ret = Eigen::Matrix<T, states, 2*states+1>::Zero();
        sigmaPoints_ret.col(0) = x;

        for(int k = 0;k<nx; k++) {
          sigmaPoints_ret.col(k+1)  =   x + gamma*S_in.col(k); // (32)
          sigmaPoints_ret.col(nx+k+1) = x - gamma*S_in.col(k); // (33)
        }
        return sigmaPoints_ret;
      }

      void update(Eigen::Matrix<T, 9, 1> yk_inn) {
        if(active) {
          // Update sigma points to reflect the prediction
          sigmaPoints = generateSigmaPoints(xHat, S);
          // Propagate the sigma points through the measurment model. 
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_h.col(s) =  h(sigmaPoints.col(s), yk_inn);
          }
          Eigen::Matrix<T, 3, 1> yk_est;
          // Unscented transform - Calculate the a priori estimate mean
          yk_est = (Wm*sigmaPoints_h.transpose()).colwise().sum();
          // Unscented transform - Calculate the a priori estimate Covariance
          Eigen::Matrix<T, states, 2*states+1> sigmaDelta = sigmaPoints_h.colwise() - yk_est;
          Eigen::Matrix<T, states, 3*states> QR;

          QR << (std::sqrt(Wc(1))*sigmaDelta.block(0,1,states, 2*states)), R_sqrt;
          
          Eigen::Matrix<T, measurements, measurements> Sy = QR.transpose().householderQr().matrixQR().topLeftCorner(states, states).template triangularView<Eigen::Upper>();

          Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(Sy, sigmaDelta.col(0), Wc(0));
          Sy.transposeInPlace();
          Eigen::Matrix<T, 3, 3> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

          K = (Pxy * Sy.inverse().transpose())*Sy.inverse();
          xHat = xHat + K*(yk_inn.block(6,0,3,1) - yk_est);
          auto U = K*Sy;
          //Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, U, -1);

          for(std::ptrdiff_t i = 0; i < states; i++) {
            Eigen::internal::llt_inplace<T, Eigen::Lower>::rankUpdate(
                S, U.col(i), T(-1.0));
          }
        }
      }

  void predict() {
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
      QR << (std::sqrt(Wc(1))*sigmaDelta.block(0,1,states, 2*states)), Q_sqrt;

      S = QR.transpose().householderQr().matrixQR().topLeftCorner(states,states).template  triangularView<Eigen::Upper>();
      // Cholesky update
      Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, sigmaDelta.col(0), Wc(0));
      S.transposeInPlace();
    }
  }

  void setUnscentedParameters(T alpha_in, T beta_in, T kappa_in) {
    alpha = alpha_in;
    beta = beta_in; // Optmal for gaussian
    kappa = kappa_in;
    lambda = alpha*alpha * (nx +kappa) - nx;
    computeWeights();
  }

  Eigen::Matrix<T, states, measurements> calculate_cross_variance(Eigen::Matrix<T, states, 1>  x_inn, Eigen::Matrix<T, measurements, 1>  z_inn, Eigen::Matrix<T, states, 2*states+1> sigmas_f, Eigen::Matrix<T, states, 2*states+1> sigmas_h, Eigen::Matrix<T, 1, 2*states+1> Wc_inn) {
    //Compute cross variance of the state x_inn and measurement z_inn.

    sigmas_f = sigmas_f.colwise() - x_inn;
    sigmas_h = sigmas_h.colwise() - z_inn;

    Eigen::Matrix<T,states,states> Pxz = Eigen::Matrix<T,states,states>::Zero();
    for(int s = 0;s<size_sigmaPoints; s++) {
        Pxz += Wc_inn(1,s) * sigmas_f.col(s) * sigmas_h.col(s).transpose();
    }
    return Pxz;
  }
    private:
      T lambda;
      T alpha;
      T beta;
      T kappa;
      T gamma;
  };
}
#endif //OFP_SquareRootUnscentedKalmanFilter