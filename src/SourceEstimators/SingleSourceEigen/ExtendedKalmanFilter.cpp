#include "ExtendedKalmanFilter.hpp"
#include <iostream>
namespace SourceEstimators
{
  namespace SingleSourceEigen
  {
    void ExtendedKalmanFilter::updateCmatrix(Eigen::Matrix<double, 3, 1> zk, Eigen::Matrix<double, 3, 1> position_previous, Eigen::Matrix<double, 3, 1> position_current) {
    // Start calculate Jacobian of measurement model at X_k
      if(active) {

        // Find euclidean norm (p-norm, p=2) between measurements and estimated tag position
        Eigen::Matrix<double, 3, 1> distance1 = xHat-position_previous; // X_e-X_rx0
        Eigen::Matrix<double, 3, 1> distance2 = xHat-position_current;  // X_e-X_rx1
        double r1 = distance1.norm();//  ||X_e-X_rx0||
        double r2 = distance2.norm();// ||X_e-X_rx1||

        // Calculate estimated measurements
        ykest(0) = r2 - r1; // h is eq (2.16) in masters
        ykest(1) = r2; // Eq (2.19) in masters
        ykest(2) = xHat(2); // Depth estimate

        // Calculate Jacobian with RDOA, SNR and Depth
        C.row(0) = (distance2/r2) - (distance1/r1); // Eq (2.18)
        C.row(1) = (distance2/r2); // Exends the Jacobian with eq (2.20)
        Eigen::Matrix<double, 1, 3> Hdepth= {0.0,0.0,1.0};
        C.row(2) = Hdepth;  

        // TODO: Move this to another place
        yk = zk;
		  }
    }
    /*Eigen::Matrix<double, 3, 1> ExtendedKalmanFilter::getEstimateOfMeasurment() {
      return ykest;
    }*/
  }
}