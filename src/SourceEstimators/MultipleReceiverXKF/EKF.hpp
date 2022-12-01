#ifndef DUNE_MULTIPLERECEIVERXKF_EKF
#define DUNE_MULTIPLERECEIVERXKF_EKF

#include <OpenFilterPack/KalmanFilterDynamic.hpp>
//#include <DUNE/DUNE.hpp>
//#include "KalmanFilter.hpp"
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    class EKF: public OFP::KalmanFilterDynamic<double, 3>{
        public:
          double qq_cov;  // Process noise Covariance - position
          double rr_cov;  // Measurement noise Covariance - range
          double rz_cov;  // Measurement noise Covariance - depth
        EKF();
        bool constructCandR(Eigen::Matrix<double, 3, Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth, Eigen::Matrix<double, 3, 1> xHatInn);
        void initialize(Eigen::Matrix<double, 3,1> xInit);
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF_EKF
