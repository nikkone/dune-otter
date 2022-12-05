
#ifndef FishTagEstimator_XKF
#define FishTagEstimator_XKF

#include "LTVKF.hpp"
#include "EKF.hpp"
#include "LeastSquares.hpp"
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    class XKF {
      public:
        LeastSquares stage1;
        LTVKF stage2;
        EKF stage3;

      //! Initializer for the filter
      //! @param [in] A_inn State transition matrix
      //! @param [in] Q_inn Measurement covariance matrix
      //! @param [in] P0_inn Initial covarinace matrix
      //! @param [in] x0_inn Initial state
      void initialize(const Eigen::Matrix<double, 3, 3> &A_inn,
                              const Eigen::Matrix<double, 3, 3> &Q_inn,
                              const Eigen::Matrix<double, 3, 3> &P0_inn,
                              const Eigen::Matrix<double, 3, 1> &x0_inn);        
        void predict();
        void update(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);

        bool isInitialized(void) const;
        void setReferenceCoordinate(double lat, double lon, double hae);
        void setInitialPosition(double north, double east, double down);
        void setTimestep(double timestep);
        void setDiagonalCovarianceR(double rr_cov);
        void setDiagonalCovarianceQ(double qq_cov);
        void setVarianceRZ(double rz_var);
        //void setActive(bool activate);
        //bool isActive(void);
      private:
        bool initialized;
        //bool active;
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF