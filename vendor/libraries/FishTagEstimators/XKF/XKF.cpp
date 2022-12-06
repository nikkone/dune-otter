#include "XKF.hpp"
#include <iostream>

namespace FishTagEstimators
{
  namespace XKF
  {

      void XKF::initialize(const Eigen::Matrix<double, 3, 3> &A_inn,
                           const Eigen::Matrix<double, 3, 3> &Q_inn,
                           const Eigen::Matrix<double, 3, 3> &P0_inn,
                           const Eigen::Matrix<double, 3, 1> &x0_inn) {
      stage2.initialize(A_inn, Q_inn, P0_inn, x0_inn);
      stage3.initialize(A_inn, Q_inn, P0_inn, x0_inn);
      initialized = true;
      }

    void
    XKF::predict()
    {
      stage2.predict();
      stage3.predict();
    }

    bool XKF::update(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations) {

      if(RDOA.rows()>0)
      {
        if(stage1.update(tagBuffer, RDOA, RDOAcombinations)) {
          if(!stage2.active) {
            stage2.active = true;
          }
          if(stage2.constructCandR(tagBuffer, RDOA, RDOAcombinations, stage1.m_dr)) {
            stage2.update(stage2.yk);
            if(!stage3.active) {
              stage3.active = true;
            }
          }
        }

        if(stage2.active && stage3.active) {
          if(stage3.constructCandR(tagBuffer, RDOA, RDOAcombinations, stage2.xHat)) {
            stage3.update(stage3.yk);
            return true;
          }
        }
      }
      return false;
    }

    bool XKF::isInitialized(void) const {
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

    void XKF::setVarianceRZ(double rz_var) {
      stage3.rz_cov = rz_var;
      stage2.rz_cov = rz_var;
    }
  }
}