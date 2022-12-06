
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
        LeastSquares();

        /// @brief Set function for maxDm
        /// @param in_maxDm Desired maxDm, should be positive
        void setMaxDm(T in_maxDm);

        /// @brief 
        /// @param tagBuffer 
        /// @param RDOA 
        /// @param RDOAcombinations 
        /// @return 
        bool update(TagBuffer *tagBuffer, Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations);

        /// @brief Get function for most recent dm
        /// @return The most recent estimate of dm
        T getDm() {
          return dm;
        }

      private:
        /// @brief Max distance between reference receiver and target considered realistic. Larger solutions are discarded.
        T maxDm;

        /// @brief Estimated distance between reference receiver and target
        T dm;

        /// @brief Internal function to decide which solution of the quadratic formula to use, see (11) in paper
        /// @param R1 First solution from quadratic formula
        /// @param R2 Second olution from quadratic formula
        /// @return Zero if unresolved, else valid dm
        T resolveRAmbiguity(T R1, T R2);
    };
    template class LeastSquares<float>;
    template class LeastSquares<double>;
    template class LeastSquares<long double>;
  }
}
#endif // END FishTagEstimator_XKF_EKF