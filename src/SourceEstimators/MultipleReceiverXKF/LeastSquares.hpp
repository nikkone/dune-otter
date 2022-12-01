
#ifndef DUNE_MULTIPLERECEIVERXKF_LEASTSQUARES
#define DUNE_MULTIPLERECEIVERXKF_LEASTSQUARES
#include <DUNE/DUNE.hpp>
#include <Eigen/Core>
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    class LeastSquares{
      public:
        //! Reference coordinate system
        Eigen::Matrix<double, 3,1> xHat;
        double m_dr;

        LeastSquares();
        void initialize();
        bool update(Eigen::Matrix<double, 3, Eigen::Dynamic>receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth);

      private:
        double resolveRAmbiguity(double R1, double R2);
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF_LEASTSQUARES