
#ifndef DUNE_SOURCEESTIMATORS_XKF_LEASTSQUARES
#define DUNE_SOURCEESTIMATORS_XKF_LEASTSQUARES
#include <DUNE/DUNE.hpp>
namespace SourceEstimators
{
  namespace FishTagXKF
  {
    class LeastSquares{
      public:
        //! Reference coordinate system
        DUNE::Math::Matrix xHat;
        double m_dr;

        LeastSquares();
        void initialize();
        uint8_t LeastSquaresEstimate(DUNE::Math::Matrix receiverPositions, DUNE::Math::Matrix RDOA, DUNE::Math::Matrix tagDepth, bool newMeasurement[3]);

      private:
        double resolveRAmbiguity(double R1, double R2);
    };
  }
}
#endif //DUNE_SOURCEESTIMATORS_XKF_LEASTSQUARES