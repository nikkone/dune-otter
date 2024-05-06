
#ifndef FishTagEstimator_XKF_LEASTSQUARES
#define FishTagEstimator_XKF_LEASTSQUARES
#include <Eigen/Core>
#include "../TagBuffer.hpp"
#include <vector>
namespace FishTagEstimators
{
  namespace XKF
  {
    /// @brief 
    template <class T>
    class LeastSquares {
      public:
        //! Reference coordinate system
        Eigen::Matrix<T, 3,1> xHat;

        /// @brief Initializes to zero position estimate and default maxDm(700)
        LeastSquares(bool useTOASNRtieBreakerInn = true);

        /// @brief Set function for maxDm
        /// @param in_maxDm Desired maxDm, should be positive
        void setMaxDm(T in_maxDm);

        /// @brief 
        /// @param tagBuffer 
        /// @param RDOA 
        /// @param RDOAcombinations 
        /// @return 
        bool update(const TagBuffer *tagBuffer, const Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, const std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);

        /// @brief Get function for most recent dm
        /// @return The most recent estimate of dm
        T getDm() {
          return dm;
        }

        /// @brief Setter for the SNR model used in the tie breaker. Model: SNR = B_fit - k_fit*log(dist) - a_fit*dist
        /// @param B_fit_inn Constant coefficient combining SL-NL-BW.
        /// @param k_fit_inn Logarithic dispersion coefficient
        /// @param a_fit_inn Absorbtion coefficient. For physicaly realistic model, should be positive.
        void setSNRmodel(T B_fit_inn, T k_fit_inn, T a_fit_inn) {
          B_fit = B_fit_inn;
          k_fit = k_fit_inn;
          a_fit = a_fit_inn;
        }
      private:
        /// @brief Max distance between reference receiver and target considered realistic. Larger solutions are discarded.
        T maxDm;

        /// @brief Estimated distance between reference receiver and target
        T dm;

        /// @brief Enables TOA tie breaker
        bool useTOASNRtieBreaker;

        //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
        T B_fit;
        //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
        T k_fit;
        //! Model coefficient for SNR = B_fit - k_fit*log(dist) - a_fit*dist
        T a_fit;

        /// @brief Internal function to decide which solution of the quadratic formula to use, see (11) in paper
        /// @param R1 First solution from quadratic formula
        /// @param R2 Second olution from quadratic formula
        /// @return Zero if unresolved, else valid dm
        T resolveRAmbiguity(T R1, T R2, std::vector<TBRFishTag> &tagDetectionsD, const Eigen::Matrix<T, 3,1> c, const Eigen::Matrix<T, 3,1> &w);
    };
    template class LeastSquares<float>;
    template class LeastSquares<double>;
    template class LeastSquares<long double>;
  }
}
#endif // END FishTagEstimator_XKF_EKF