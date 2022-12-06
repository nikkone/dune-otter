#ifndef FishTagEstimator_XKF_LTVKF
#define FishTagEstimator_XKF_LTVKF

#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include <Eigen/Core>
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    class LTVKF: public OFP::KalmanFilterDynamic<double, 3>{
        public:
          double rr_cov;  // Measurement noise Covariance - range
          double rz_cov;  // Measurement noise Covariance - depth
        LTVKF();
        bool constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, double m_dr);        
    };
  }
}
#endif // END FishTagEstimator_XKF_EKF_LTVKF