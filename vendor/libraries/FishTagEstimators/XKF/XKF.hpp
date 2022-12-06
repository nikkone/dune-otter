
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
    /// @brief 
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

        /// @brief Time update of the filter
        void predict();

        /// @brief Filter update
        /// @param tagBuffer Buffer structure of all receivers and tags considered.
        /// @param RDOA Pre-calculated range difference of arrival vector
        /// @param RDOAcombinations Vector containing the pairs of receivers that were used in calculating RDOA
        /// @return returns True when stage3 filter updated
        bool update(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);

        /// @brief Check if filter matrices has been initialized
        /// @return returns initialized boolean
        bool isInitialized(void) const;

        /// @brief Filter timestep used in calculations
        /// @param timestep in seconds
        void setTimestep(double timestep);

        /// @brief Time/Range Difference of Arrival covariance
        /// @param rr_cov Covariance used when generating the R matrix
        void setDiagonalCovarianceR(double rr_cov);

        /// @brief Covariance of the Depth measurements
        /// @param rz_var Covariance used when generating the R matrix
        void setVarianceRZ(double rz_var);

      private:

        /// @brief Keeps track on if the estimator matrices are initialized
        bool initialized;
    };
  }
}
#endif //FishTagEstimator_XKF