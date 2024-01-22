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
    template <class T>
    class LTVKF: public OFP::KalmanFilterDynamic<T, 3>{
        public:
          T rr_cov;  // Measurement noise Covariance - range
          T rz_cov;  // Measurement noise Covariance - depth
        //LTVKF();
        bool constructCandR(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations, T m_dr);        
    };
    template class LTVKF<float>;
    template class LTVKF<double>;
    template class LTVKF<long double>;
  }
}
#endif // END FishTagEstimator_XKF_EKF_LTVKF