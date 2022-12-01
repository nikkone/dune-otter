#include "XKF.hpp"
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
  {
    XKF::XKF() {
      //initialize();
    }

    //! Initialize Extended Kalman filter
    //! @param [in] initPosition[0] North coordinate in reference frame given in meters.
    //! @param [in] initPosition[1] East coordinate in reference frame given in meters.
    //! @param [in] initPosition[2] Height in reference frame given in meters.
    void
    XKF::initialize(Eigen::Matrix<double, 3,1> xInit)
    {
      stage2.initialize(xInit);
      stage3.initialize(xInit);
      initialized = true;
    } // End of initializeXKF function


    void
    XKF::predict()
    {
      stage2.predict();
      stage3.predict();
    }


    void XKF::update(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RDOA, double tagDepth) {

        if(RDOA.size()>0)
        {
          //std::cout << "Entering LeastSquares::update" <<  std::endl;
          bool isSuccess = stage1.update(receiverPositions, RDOA, tagDepth);
          //std::cout << "Leaving LeastSquares::update" <<  std::endl;
          if(isSuccess && !stage2.active)
          {
              stage3.active = 1;
              stage2.active = 1;
          }
        }
//std::cout << "RDOA.rows()" << std::endl << RDOA.rows() << std::endl;
        //>> Step 2.2: Measurement Update - stage - 2
        if(stage2.active && RDOA.rows()>1) // conditions for stage 2 update
        {
          //std::cout << "Entering LTVKF::constructCandR" <<  std::endl;
          if(stage2.constructCandR(receiverPositions, RDOA, tagDepth, stage1.m_dr)) {
          //std::cout << "Entering LTVKF::update" <<  std::endl;
          stage2.update(stage2.yk);
          //std::cout << "Leaving LTVKF::update" <<  std::endl;
          }
        }
        if(stage3.active)
        {
          //std::cout << "Entering EKF::constructCandR" <<  std::endl;
          if(stage3.constructCandR(receiverPositions, RDOA, tagDepth, stage2.xHat)) {
            //std::cout << "Entering EKF::update" <<  std::endl;
            stage3.update(stage3.yk);
            //std::cout << "Leaving EKF::update" <<  std::endl;
          }
        }
    }


    bool XKF::isInitialized(void) {
      return initialized;
    }

    void XKF::setTimestep(double timestep) {
      stage2.dt=timestep;
      stage3.dt=timestep;
    }

    void XKF::setDiagonalCovarianceR(double rr_cov) {
      stage3.rr_cov = rr_cov;
      stage2.rr_cov = rr_cov;
    }

    void XKF::setDiagonalCovarianceQ(double qq_cov) {
      stage3.qq_cov = qq_cov;
      stage2.qq_cov = qq_cov;
    }

    void XKF::setVarianceRZ(double rz_var) {
      stage3.rz_cov = rz_var;
      stage2.rz_cov = rz_var;
    }

    void XKF::setActive(bool activate) {
      active=activate;
    }

    bool XKF::isActive(void) {
      return active;
    }
  }
}