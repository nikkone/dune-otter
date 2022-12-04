
#ifndef FishTagEstimator_XKF_LEASTSQUARES
#define FishTagEstimator_XKF_LEASTSQUARES
#include <Eigen/Core>
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    class LeastSquares{
      public:
        //! Reference coordinate system
        Eigen::Matrix<double, 3,1> xHat;
        double m_dr;

        LeastSquares();
        void initialize();
        //bool update(Eigen::Matrix<double, 3, Eigen::Dynamic>receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth);
        bool update(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);
      private:
        double resolveRAmbiguity(double R1, double R2);
    };
  }
}
#endif // END FishTagEstimator_XKF_EKF