
#ifndef DUNE_MULTIPLERECEIVERXKF_XKF
#define DUNE_MULTIPLERECEIVERXKF_XKF

#include <DUNE/DUNE.hpp>
#include "LTVKF.hpp"
#include "EKF.hpp"
#include "LeastSquares.hpp"
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    class XKF {
      public:
        LeastSquares stage1;
        LTVKF stage2;
        EKF stage3;

        XKF();

        void initialize(Eigen::Matrix<double, 3,1> xInit);
        void predict();
        void update(Eigen::Matrix<double, 3, Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth);


        bool isInitialized(void);
        void setReferenceCoordinate(double lat, double lon, double hae);
        void setInitialPosition(double north, double east, double down);
        void setTimestep(double timestep);
        void setDiagonalCovarianceR(double rr_cov);
        void setDiagonalCovarianceQ(double qq_cov);
        void setVarianceRZ(double rz_var);
        void setActive(bool activate);
        bool isActive(void);
      private:
        bool initialized;
        bool active;
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF