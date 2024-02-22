
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
    /// @brief Implementation of the eXogenous fish positioning filter outlined in
    /// ( R. P. Jain et al., “Localization of an Acoustic Fish-Tag using the Time-of-Arrival Measurements: Preliminary results using eXogenous Kalman Filter,” in 
    /// 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2018, pp. 1695–1702. doi: 10.1109/IROS.2018.8593659.
    template <class T>
    class XKF {
      public:
        LeastSquares<T> stage1;
        LTVKF<T> stage2;
        EKF<T> stage3;

      //! Initializer for the filter
      //! @param [in] A_inn State transition matrix
      //! @param [in] Q_inn Measurement covariance matrix
      //! @param [in] P0_inn Initial covarinace matrix
      //! @param [in] x0_inn Initial state
      void initialize(const Eigen::Matrix<T, 3, 3> &A_inn,
                              const Eigen::Matrix<T, 3, 3> &Q_inn,
                              const Eigen::Matrix<T, 3, 3> &P0_inn,
                              const Eigen::Matrix<T, 3, 1> &x0_inn);  

        /// @brief Time update of the filter
        void predict();

        /// @brief Filter update
        /// @param tagBuffer Buffer structure of all receivers and tags considered.
        /// @param RDOA Pre-calculated range difference of arrival vector
        /// @param RDOAcombinations Vector containing the pairs of receivers that were used in calculating RDOA
        /// @return returns True when stage3 filter updated
        uint8_t update(const TagBuffer *tagBuffer, const Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, const std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);

        /// @brief Check if filter matrices has been initialized
        /// @return returns initialized boolean
        bool isInitialized(void) const;

        /// @brief Filter timestep used in calculations
        /// @param timestep in seconds
        void setTimestep(T timestep);

        /// @brief Time/Range Difference of Arrival covariance
        /// @param rr_cov Covariance used when generating the R matrix
        void setDiagonalCovarianceR(T rr_cov);

        /// @brief Covariance of the Depth measurements
        /// @param rz_var Covariance used when generating the R matrix
        void setVarianceRZ(T rz_var);

      private:

        /// @brief Keeps track on if the estimator matrices are initialized
        bool initialized;
    };
    template class XKF<float>;
    template class XKF<double>;
    template class XKF<long double>;
  }
}
#endif //FishTagEstimator_XKF