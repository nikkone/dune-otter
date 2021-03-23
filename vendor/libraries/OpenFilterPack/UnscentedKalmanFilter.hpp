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
      bool active;
      T dt;                                                        // Timestep
      const int nx = states;                                       // Number of states
      const int ny = measurements;                                 // Number of measurements
      const int size_sigmaPoints = 2*states+1;                     // Number of Sigma states
      Eigen::Matrix<T, states, measurements> K;                    // State Covariance estimate matric
      Eigen::Matrix<T, states, states> PHat;                       // State Covariance estimate matric
      Eigen::Matrix<T, states, states> Q;                          // Process noise covariance matrix
      Eigen::Matrix<T, measurements, measurements> R;              // Measurement noise covariance matrix
      Eigen::Matrix<T, measurements, 1> innov;                     // Time varying innovation vector
      Eigen::Matrix<T, states, 1> xHat;                            // State estimate vector
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints;      // Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_f;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, measurements, 2*states+1> sigmaPoints_h;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, 1, 2*states+1> Wc;                          // Sigma point weights for the Unscented transform covariance.
      Eigen::Matrix<T, 1, 2*states+1> Wm;                          // Sigma point weights for the Unscented transform mean.
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, states, 1> x)> f;
      std::function<Eigen::Matrix<T, measurements, 1>(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, measurements, 1> z)> h;

      UnscentedKalmanFilter(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        computeWeights();
        //dt=0.1;
      }

      void computeWeights() {
        T c = 1 / (nx + lambda);
        Wc = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wc[0] = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm[0] = lambda *c; // (36)
      }

      Eigen::Matrix<T, states, 2*states+1> generateSigmaPoints(Eigen::Matrix<T, states, 1> x, Eigen::Matrix<T, states, states> P) {
        Eigen::Matrix<T, states, 2*states+1> sigmaPoints_ret = Eigen::Matrix<T, states, 2*states+1>::Zero();
        sigmaPoints_ret.col(0) = x;
        Eigen::Matrix<T, states, states> U = ((lambda+states)*P).llt().matrixL();

        for(int k = 0;k<states; k++) {
          sigmaPoints_ret.col(k+1)  =  x + U.col(k); // (32)
          sigmaPoints_ret.col(states+k+1) = x - U.col(k); // (33)
        }
        return sigmaPoints_ret;
      }

      void update(Eigen::Matrix<T, measurements, 1> yk_inn) {
        if(active) {
          // Update sigma points to reflect the prediction 
          //std::cout << "Unscented update!" << std::endl;
          sigmaPoints = generateSigmaPoints(xHat, PHat);
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_h.col(s) =  h(sigmaPoints.col(s), yk_inn);
          }
          Eigen::Matrix<T, measurements, 1> yk_est;
          Eigen::Matrix<T, measurements, measurements> Py = unscented_transform<measurements>(sigmaPoints_h, Wm, Wc, yk_est);// + R;
          
          Eigen::Matrix<T, states, measurements> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

          K = Pxy * Py.inverse();
          //xHat = xHat + K*(yk_inn.block(6,0,3,1) - yk_est);
          xHat = xHat + K*(yk_inn - yk_est);


          PHat = PHat - K*Pxy.transpose();
        }
      }

      void predict() {
        // Compute process sigma points
        sigmaPoints = generateSigmaPoints(xHat, PHat);
        for(int s = 0;s<size_sigmaPoints; s++) {
          sigmaPoints_f.col(s) =  f(sigmaPoints.col(s));
        }
        // Unscented transform
        //std::cout << "xHat_inn_pred: " << std::endl << xHat << std::endl;
        PHat = unscented_transform<states>(sigmaPoints_f, Wm, Wc, xHat) + Q;
        //std::cout << "xHat_ut_pred: " << std::endl << xHat << std::endl;

      }

      void setUnscentedParameters(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in; // Optmal for gaussian
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        computeWeights();
      }

      Eigen::Matrix<T, states, measurements> calculate_cross_variance(Eigen::Matrix<T, states, 1>  x_inn, Eigen::Matrix<T, measurements, 1>  z_inn, Eigen::Matrix<T, states, 2*states+1> sigmas_f, Eigen::Matrix<T, measurements, 2*states+1> sigmas_h, Eigen::Matrix<T, 1, 2*states+1> Wc_inn) {
        //Compute cross variance of the state x_inn and measurement z_inn.

        sigmas_f = sigmas_f.colwise() - x_inn;
        sigmas_h = sigmas_h.colwise() - z_inn;

        Eigen::Matrix<T,states,measurements> Pxz = Eigen::Matrix<T,states,measurements>::Zero();
        for(int s = 0;s<size_sigmaPoints; s++) {
            Pxz += Wc_inn(1,s) * sigmas_f.col(s) * sigmas_h.col(s).transpose();
        }
        return Pxz;
      }

      template<int dimensions>
      Eigen::Matrix<T, dimensions, dimensions> unscented_transform(Eigen::Matrix<T, dimensions, 2*states+1> sigmas, Eigen::Matrix<T, 1, 2*states+1> Wm_inn, Eigen::Matrix<T, 1, 2*states+1> Wc_inn, Eigen::Matrix<T, dimensions, 1> &x_inn) {
        x_inn = (Wm_inn*sigmas.transpose()).colwise().sum();
        sigmas = sigmas.colwise() - x_inn;
        Eigen::Matrix<T, dimensions, dimensions> P = Eigen::Matrix<T, dimensions, dimensions>::Zero();
        for(int s = 0;s<size_sigmaPoints; s++) {
          P += Wc_inn(1,s) * sigmas.col(s) * sigmas.col(s).transpose();
        }
        return P;
      }

    private:
      T lambda;
      T alpha;
      T beta;
      T kappa;
  };
}
#endif //OFP_UnscentedKalmanFilter