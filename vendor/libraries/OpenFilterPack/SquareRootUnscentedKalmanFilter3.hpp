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
  class SquareRootUnscentedKalmanFilter3
  {
    public:
      bool active;
      T dt; // Timestep
      const int nx = states;                          // Num states
      const int ny = measurements;                    // Num outputs
      const int size_sigmaPoints = 2*states+1;
      Eigen::Matrix<T, states, measurements> K;       // State Covariance estimate matrix
      Eigen::Matrix<T, states, states> PHat;          // State Covariance estimate matrix
      Eigen::Matrix<T, states, states> S;             // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, states, states> Q;             // Process noise covariance matrix
      Eigen::Matrix<T, states, states> Q_sqrt;        // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, measurements, measurements> R; // Measurement noise covariance matrix
      Eigen::Matrix<T, measurements, measurements> R_sqrt;        // Square-Root of Covariance estimate matrix
      Eigen::Matrix<T, measurements, 1> innov;        // Time varying innovation vector
      Eigen::Matrix<T, states, 1> xHat;               // State estimate vector
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints;      // Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_f;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, states, 2*states+1> sigmaPoints_h;    // Sigma points for the Unscented transform.
      Eigen::Matrix<T, 1, 2*states+1> Wc;                          // Sigma point weights for the Unscented transform covariance.
      Eigen::Matrix<T, 1, 2*states+1> Wm;                          // Sigma point weights for the Unscented transform mean.
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, states, 1> x)> f;
      std::function<Eigen::Matrix<T, states, 1>(Eigen::Matrix<T, measurements, 1> x)> h;

      SquareRootUnscentedKalmanFilter3(T alpha_in, T beta_in, T kappa_in) {
        alpha = alpha_in;
        beta = beta_in;
        kappa = kappa_in;
        lambda = alpha*alpha * (nx +kappa) - nx;
        gamma = std::sqrt(nx+lambda);
        computeWeights();
        //dt=0.1;
      }

      void setInitialCovariance(Eigen::Matrix<T, states, states> P0) {
        S = P0.llt().matrixU(); //Upper triangular cholesky
        //S=P0;
      }

      void setMeasurmentCovariance(Eigen::Matrix<T, states, states> R_in) {
        R=R_in;

        R_sqrt = R_in.llt().matrixU(); //Upper triangular cholesky
        //std::cout << "R" << std::endl << R << std::endl;  
        //std::cout << "R_sqrt" << std::endl << R_sqrt << std::endl;  
        //std::cout << "R_sqrt^2" << std::endl << R_sqrt*R_sqrt.transpose() << std::endl;
      }

      void setProcessCovariance(Eigen::Matrix<T, states, states> Q_in) {
        Q=Q_in;
        Q_sqrt = Q_in.llt().matrixU(); //Upper triangular cholesky

      }
      void computeWeights() {
        T c = 1 / (nx + lambda);
        Wc = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<T, 1, 2*states+1>::Constant(0.5*c); // (38)
        Wc[0] = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm[0] = lambda *c; // (36)
        std::cout << "SR_Wm" << std::endl << Wm << std::endl;
        std::cout << "SR_Wc" << std::endl << Wc << std::endl;
        
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

      void update(Eigen::Matrix<T, measurements, 1> yk_inn) {
        if(active) {
          // Update sigma points to reflect the prediction
          sigmaPoints = generateSigmaPoints(xHat, S);
          std::cout << "sigmaPoints" << std::endl << sigmaPoints << std::endl;
          // Propagate the sigma points through the measurment model. 
          for(int s = 0;s<size_sigmaPoints; s++) {
            sigmaPoints_h.col(s) =  h(sigmaPoints.col(s));
          }
          std::cout << "sigmaPoints_h" << std::endl << sigmaPoints_h << std::endl;
          std::cout << "Wm" << std::endl << Wm << std::endl;
          Eigen::Matrix<T, 3, 1> yk_est;
          // Unscented transform - Calculate the a priori estimate mean
          yk_est = sigmaPoints_h*Wm.transpose();

          //yk_est = (sigmaPoints_h*Wm.transpose()).colwise().sum();
//std::cout << "sigmaPoints_h" << std::endl << sigmaPoints_h << std::endl;
//std::cout << "Wm" << std::endl << Wm << std::endl;
//std::cout << "yk_est" << std::endl << yk_est << std::endl;
//std::cout << "Wm*sigmaPoints_h.transpose()" << std::endl << Wm*sigmaPoints_h.transpose() << std::endl;  
//std::cout << "sigmaPoints_h*Wm.transpose()" << std::endl << sigmaPoints_h*Wm.transpose() << std::endl;  
          // Unscented transform - Calculate the a priori estimate Covariance
          Eigen::Matrix<T, states, 2*states+1> sigmaDelta = sigmaPoints_h.colwise() - yk_est;
          //std::cout << "sigmaDelta" << std::endl << sigmaDelta << std::endl;
          Eigen::Matrix<T, states, 3*states> QR;
          //std::cout << "std::sqrt(Wc(1)" << std::endl << std::sqrt(Wc(1)) << std::endl;
          QR << (std::sqrt(Wc(1))*sigmaDelta.block(0,1,states, 2*states)), R_sqrt;
         //std::cout << "QR.transpose().householderQr().matrixQR()" << std::endl << QR.transpose().householderQr().matrixQR() << std::endl;

          Eigen::Matrix<T, 3, 3> Sy = QR.transpose().householderQr().matrixQR().topLeftCorner(states, states).template triangularView<Eigen::Upper>();
          Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(Sy, sigmaDelta.col(0), Wc(0));
          Sy.transposeInPlace();
          Eigen::Matrix<T, 3, 3> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

          K = (Pxy * Sy.inverse().transpose())*Sy.inverse();
          xHat = xHat + K*(yk_inn.block(6,0,3,1) - yk_est);
          auto U = K*Sy;
         std::cout << "S_update_PRE^2" << std::endl << S.transpose()*S << std::endl; 
          for(int i = 0; i < states; i++) {
            Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, U.col(i), T(-1.0));
          }
         std::cout << "S_update_POST^2" << std::endl << S.transpose()*S << std::endl; 

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
       //std::cout << "sigmaPoints_f" << std::endl << sigmaPoints_f << std::endl; 
       //std::cout << "sigmaPoints" << std::endl << sigmaPoints << std::endl; 
      // Unscented transform - Calculate the a priori estimate mean
      xHat = (Wm*sigmaPoints_f.transpose()).colwise().sum();
      // Unscented transform - Calculate the a priori estimate Covariance 
      Eigen::Matrix<T, states, 2*states+1> sigmaDelta = sigmaPoints_f.colwise() - xHat;
     //std::cout << "xHat" << std::endl << xHat << std::endl; 
std::cout << "sigmaDelta" << std::endl << sigmaDelta << std::endl; 

      Eigen::Matrix<T, states, 3*states>  QR;
      QR << (std::sqrt(Wc(1))*sigmaDelta.rightCols(2*states)), Q_sqrt;
      //Eigen::Matrix<T, states, 3*states>  QR_alt;
      //QR_alt << (std::sqrt(Wc(1))*sigmaDelta.block(0,1,states, 2*states)), Q_sqrt;
      //std::cout << "pred_QR_alt" << std::endl << QR_alt << std::endl; 
     //std::cout << "pred_QR_T" << std::endl << QR.transpose() << std::endl; 
     std::cout << "S_pred_pre^2" << std::endl << S.transpose()*S << std::endl; 
     //std::cout << "QR.transpose().householderQr().matrixQR()" << std::endl << QR.transpose().householderQr().matrixQR() << std::endl; 
      S = QR.transpose().householderQr().matrixQR().topLeftCorner(states,states).template  triangularView<Eigen::Upper>();
     std::cout << "S_pred_inter^2" << std::endl << S.transpose()*S << std::endl; 
      // Cholesky update
      Eigen::internal::llt_inplace<T, Eigen::Upper>::rankUpdate(S, sigmaDelta.col(0), Wc(0));
      //S.transposeInPlace();
     std::cout << "S_pred_post^2" << std::endl << S.transpose()*S << std::endl; 
    }
  }

  void setUnscentedParameters(T alpha_in, T beta_in, T kappa_in) {
    alpha = alpha_in;
    beta = beta_in; // Optimal for gaussian
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
        Pxz += Wc_inn(s) * sigmas_f.col(s) * sigmas_h.col(s).transpose();
    }
    return Pxz;
  }

      /*template<int dimensions>
      Eigen::Matrix<T, dimensions, dimensions> unscented_transform(Eigen::Matrix<T, dimensions, 2*states+1> sigmas, Eigen::Matrix<T, 1, 2*states+1> Wm_inn, Eigen::Matrix<T, 1, 2*states+1> Wc_inn, Eigen::Matrix<T, dimensions, 1> &x_inn) {
        // Mean
        x_inn = (Wm_inn*sigmas.transpose()).colwise().sum();
        // Covariance
        //Eigen::Matrix<T, dimensions, dimensions> S = Eigen::Matrix<T, dimensions, dimensions>::Zero();

        //QR.matrixQR().triangularView<Eigen::Upper>();
        sigmas = sigmas.colwise() - x_inn;
        Eigen::Matrix<T, dimensions, dimensions> S = Eigen::Matrix<T, dimensions, dimensions>::Zero();
        for(int s = 0;s<size_sigmaPoints; s++) {
          S += Wc_inn(1,s) * sigmas.col(s) * sigmas.col(s).transpose();
        }

        return S;
      }
*/
    private:
      T lambda;
      T alpha;
      T beta;
      T kappa;
      T gamma;
  };
}
#endif //OFP_SquareRootUnscentedKalmanFilter