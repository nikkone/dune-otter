#include "UnscentedKalmanFilter.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Dense>

namespace OFP
{
  UnscentedKalmanFilter::UnscentedKalmanFilter() {
    n = 3;
    setUnscentedParameters(0.001,2,0);
  }
  void UnscentedKalmanFilter::setUnscentedParameters(double alpha_in, double beta_in, double kappa_in) {
    alpha = alpha_in;
    beta = beta_in; // Optmal for gaussian
    kappa = kappa_in;
    lambda = alpha*alpha * (n +kappa) - n;
    computeWeights();
  }
  void UnscentedKalmanFilter::computeWeights() {
    // [1] Feng et al Nonlinear bayesian estimation...
        double c = 1 / (n + lambda);
        Wc = Eigen::Matrix<double, 1, 7>::Constant(0.5*c); // (38)
        Wm = Eigen::Matrix<double, 1, 7>::Constant(0.5*c); // (38)
        Wc[0] = lambda *c + (1 - alpha*alpha + beta); // (37)
        Wm[0] = lambda *c; // (36)
  }
  Eigen::Matrix<double, 3, 7> UnscentedKalmanFilter::generateSigmaPoints(Eigen::Matrix<double, 3, 1> x, Eigen::Matrix<double, 3, 3> P) {
    Eigen::Matrix<double, 3, 7> sigmaPoints_ret = Eigen::Matrix<double, 3, 7>::Zero();
    sigmaPoints_ret.col(0) = x;
    Eigen::Matrix<double, 3,3> U = ((lambda+n)*P).llt().matrixL();

    for(int k = 0;k<n; k++) {
      sigmaPoints_ret.col(k+1)  =  x + U.col(k); // (32)
      sigmaPoints_ret.col(n+k+1) = x - U.col(k); // (33)
    }
    return sigmaPoints_ret;
  }

  Eigen::Matrix<double, 3, 1> UnscentedKalmanFilter::f(Eigen::Matrix<double, 3, 1> x) {
    return A*x;
  }
  //! TODO: Change
  Eigen::Matrix<double, 3, 1> UnscentedKalmanFilter::h(Eigen::Matrix<double, 3, 1> x, Eigen::Matrix<double, 9, 1> z) {
      // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
      Eigen::Matrix<double, 3, 1> distance1 = x-z.block(0,0,3,1); // X_e-X_rx0
      Eigen::Matrix<double, 3, 1> distance2 = x-z.block(3,0,3,1);  // X_e-X_rx1
      double r1 = distance1.norm();//  ||X_e-X_rx0||
      double r2 = distance2.norm();// ||X_e-X_rx1||

      // Calculate estimated measurements
      ykest(0) = r2 - r1; // h is eq (2.16) in masters
      ykest(1) = r2; // Eq (2.19) in masters
      ykest(2) = x(2); // Depth estimate
    return ykest;
  }

  void UnscentedKalmanFilter::update(Eigen::Matrix<double, 9, 1> yk_inn) {
    if(active) {
      // Update sigma points to reflect the prediction 
      //std::cout << "Unscented update!" << std::endl;
      sigmaPoints = generateSigmaPoints(xHat, PHat);
      for(int s = 0;s<7; s++) {
        sigmaPoints_h.col(s) =  h(sigmaPoints.col(s), yk_inn);
      }
      Eigen::Matrix<double, 3, 1> yk_est;
      Eigen::Matrix<double, 3, 3> Py = unscented_transform<3>(sigmaPoints_h, Wm, Wc, yk_est) + R;

      Eigen::Matrix<double, 3, 3> Pxy = calculate_cross_variance(xHat, yk_est, sigmaPoints, sigmaPoints_h, Wc);

      K = Pxy * Py.inverse();
      //std::cout << "Py: " << std::endl << Py << std::endl;
      //std::cout << "Pxy: " << std::endl << Pxy << std::endl;
      //std::cout << "K: " << std::endl << K << std::endl;
    //std::cout << "xHat_inn_up: " << std::endl << xHat << std::endl;

      xHat = xHat + K*(yk_inn.block(6,0,3,1) - yk_est);
    //std::cout << "xHat_ut_up: " << std::endl << xHat << std::endl;

      PHat = PHat - K*Pxy.transpose();
    }
  }
  void UnscentedKalmanFilter::predict() {
    // Compute process sigma points
    sigmaPoints = generateSigmaPoints(xHat, PHat);
    for(int s = 0;s<7; s++) {
      sigmaPoints_f.col(s) =  f(sigmaPoints.col(s));
    }
    // Unscented transform
    //std::cout << "xHat_inn_pred: " << std::endl << xHat << std::endl;
    PHat = unscented_transform<3>(sigmaPoints_f, Wm, Wc, xHat) + Q;
    //std::cout << "xHat_ut_pred: " << std::endl << xHat << std::endl;

  }

  Eigen::Matrix<double, 3, 3> UnscentedKalmanFilter::calculate_cross_variance(Eigen::Matrix<double, 3, 1>  x_inn, Eigen::Matrix<double, 3, 1>  z_inn, Eigen::Matrix<double, 3, 7> sigmas_f, Eigen::Matrix<double, 3, 7> sigmas_h, Eigen::Matrix<double, 1, 7> Wc_inn) {
    //Compute cross variance of the state x_inn and measurement z_inn.

    sigmas_f = sigmas_f.colwise() - x_inn;
    sigmas_h = sigmas_h.colwise() - z_inn;
    ////std::cout << "x_inn: " << std::endl << x_inn << std::endl;
    ////std::cout << "y_inn: " << std::endl << z_inn << std::endl;
    ////std::cout << "sigmas_f: " << std::endl << sigmas_f << std::endl;
    ////std::cout << "sigmas_h: " << std::endl << sigmas_h << std::endl;
    Eigen::Matrix<double, 3, 3> Pxz = Eigen::Matrix<double, 3, 3>::Zero();
    for(int s = 0;s<7; s++) {
        Pxz += Wc_inn(1,s) * sigmas_f.col(s) * sigmas_h.col(s).transpose();
    }
    return Pxz;
  }
}