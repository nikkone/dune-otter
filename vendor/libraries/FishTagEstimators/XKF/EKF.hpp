#ifndef FishTagEstimator_XKF_EKF
#define FishTagEstimator_XKF_EKF

#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include <Eigen/Core>
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    template <class T>
    class EKF: public OFP::KalmanFilterDynamic<T, 3>{
        public:
        //EKF();
          T rr_cov;  // Measurement noise Covariance - range
          T rz_cov;  // Measurement noise Covariance - depth
        bool constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, Eigen::Matrix<T, 3, 1> xHatInn);
    };
    template class EKF<float>;
    template class EKF<double>;
    template class EKF<long double>;
  }
}
#endif // END FishTagEstimator_XKF_EKF
