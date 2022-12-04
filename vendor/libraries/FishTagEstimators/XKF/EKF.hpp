#ifndef FishTagEstimator_XKF_EKF
#define FishTagEstimator_XKF_EKF

#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    class EKF: public OFP::KalmanFilterDynamic<double, 3>{
        public:
          double qq_cov;  // Process noise Covariance - position
          double rr_cov;  // Measurement noise Covariance - range
          double rz_cov;  // Measurement noise Covariance - depth
        EKF();
        bool constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, Eigen::Matrix<double, 3, 1> xHatInn);
        void initialize(Eigen::Matrix<double, 3,1> xInit);
    };
  }
}
#endif // END FishTagEstimator_XKF_EKF
