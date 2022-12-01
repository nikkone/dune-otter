#ifndef DUNE_MULTIPLERECEIVERXKF_LTVKF
#define DUNE_MULTIPLERECEIVERXKF_LTVKF
//#include <DUNE/DUNE.hpp>
#include <OpenFilterPack/KalmanFilterDynamic.hpp>
#include <Eigen/Core>
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    class LTVKF: public OFP::KalmanFilterDynamic<double, 3>{
        public:
          double qq_cov;  // Process noise Covariance - position
          double rr_cov;  // Measurement noise Covariance - range
          double rz_cov;  // Measurement noise Covariance - depth
        LTVKF();
        bool constructCandR(Eigen::Matrix<double, 3, Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth, double m_dr);
        void initialize(Eigen::Matrix<double, 3,1> xInit);
        
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF_LTVKF