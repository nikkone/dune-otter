#ifndef OFP_UnscentedKalmanFilter
#define OFP_UnscentedKalmanFilter
#include <Eigen/Core>
#include "KalmanFilter.hpp"
#include <iostream> // For debugging, remove after

#define MEASUREMENTS_UKF 3
#define STATES_UKF 3
#define SIGMA_POINTS_UKF 2*STATES_UKF+1
namespace OFP
{
  class UnscentedKalmanFilter : public KalmanFilter<double, STATES_UKF, MEASUREMENTS_UKF>
  {
    public:
      UnscentedKalmanFilter();
      void update(Eigen::Matrix<double, 9, 1> yk_inn);
      void predict();
      Eigen::Matrix<double, MEASUREMENTS_UKF, SIGMA_POINTS_UKF> sigmaPoints;// Sigma points for the Unscented transform.
      Eigen::Matrix<double, MEASUREMENTS_UKF, SIGMA_POINTS_UKF> sigmaPoints_f;// Sigma points for the Unscented transform.
      Eigen::Matrix<double, MEASUREMENTS_UKF, SIGMA_POINTS_UKF> sigmaPoints_h;// Sigma points for the Unscented transform.
      Eigen::Matrix<double, 1, SIGMA_POINTS_UKF> Wc;         // Sigma point weights for the Unscented transform of covariance.
      Eigen::Matrix<double, 1, SIGMA_POINTS_UKF> Wm;         // Sigma point weights for the Unscented transform of mean.
      double alpha;
      double beta;
      double kappa;
      double lambda;
      double n;
      // Merwe
      void computeWeights();
      Eigen::Matrix<double, 3, SIGMA_POINTS_UKF> generateSigmaPoints(Eigen::Matrix<double, 3, 1> x, Eigen::Matrix<double, 3, 3> P);
      Eigen::Matrix<double, STATES_UKF, 1> f(Eigen::Matrix<double, STATES_UKF, 1> x);
      Eigen::Matrix<double, STATES_UKF, 1> h(Eigen::Matrix<double, MEASUREMENTS_UKF, 1> x, Eigen::Matrix<double, 9, 1> z);
      
      template<int dimensions>
      Eigen::Matrix<double, dimensions, dimensions> unscented_transform(Eigen::Matrix<double, dimensions, SIGMA_POINTS_UKF> sigmas, Eigen::Matrix<double, 1, SIGMA_POINTS_UKF> Wm_inn, Eigen::Matrix<double, 1, SIGMA_POINTS_UKF> Wc_inn, Eigen::Matrix<double, dimensions, 1> &x_inn) {
        //std::cout << "sigmas: " << std::endl << sigmas << std::endl;
        Eigen::Matrix<double, dimensions, 1> x_alt;
        Eigen::Matrix<double, dimensions, SIGMA_POINTS_UKF> sigmas_alt = sigmas;
        std::cout << "x_inn: " << std::endl << x_inn << std::endl;
        x_inn = (Wm_inn*sigmas.transpose()).colwise().sum();
        std::cout << "x_out: " << std::endl << x_inn << std::endl;
        sigmas = sigmas.colwise() - x_inn;
        Eigen::Matrix<double, dimensions, dimensions> P = Eigen::Matrix<double, dimensions, dimensions>::Zero();
        for(int s = 0;s<SIGMA_POINTS_UKF; s++) {
          P += Wc_inn(1,s) * sigmas.col(s) * sigmas.col(s).transpose();
          x_alt += Wm_inn(1,s)*sigmas_alt.col(s);

        }
        std::cout << "x_alt: " << std::endl << x_alt << std::endl;
        return P;
      }

      Eigen::Matrix<double, 3, 3> calculate_cross_variance(Eigen::Matrix<double, STATES_UKF, 1>  x_inn, Eigen::Matrix<double, MEASUREMENTS_UKF, 1>  z_inn, Eigen::Matrix<double, STATES_UKF, SIGMA_POINTS_UKF> sigmas_f, Eigen::Matrix<double, STATES_UKF, SIGMA_POINTS_UKF> sigmas_h, Eigen::Matrix<double, 1, SIGMA_POINTS_UKF> Wc_inn);

  };
}
#endif //OFP_UnscentedKalmanFilter