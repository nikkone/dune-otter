#include "XKF.hpp"
#include <iostream>

namespace FishTagEstimators
{
  namespace XKFVelocity
  {
    template <class T> 
      void XKF<T>::initialize(const Eigen::Matrix<T, 6, 6> &A_inn,
                           const Eigen::Matrix<T, 3, 3> &Q_inn,
                           const Eigen::Matrix<T, 6, 6> &P0_inn,
                           const Eigen::Matrix<T, 6, 1> &x0_inn) {
      stage2.initialize(A_inn, Q_inn, P0_inn, x0_inn);
      stage3.initialize(A_inn, Q_inn, P0_inn, x0_inn);
      initialized = true;
      }
    template <class T>
    void XKF<T>::predict()
    {
      stage2.predict();
      stage3.predict();
    }
    template <class T>
    uint8_t XKF<T>::update(const TagBuffer *tagBuffer, const Eigen::Matrix<T, Eigen::Dynamic, 1> RDOA, const std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations) {
      std::cout << std::endl << "RDOA.rows(): " << RDOA.rows() << std::endl;
      uint8_t status = 0;
      if(RDOA.rows()>0)
      {
        if(stage1.update(tagBuffer, RDOA, RDOAcombinations)) {
          status += 1;
          std::cout << std::endl << "Stage1 update" << std::endl;
          if(!stage2.active) {
            stage2.xHat(0) = stage1.xHat(0);
            stage2.xHat(1) = stage1.xHat(1);
            stage2.xHat(2) = stage1.xHat(2);
            stage2.active = true;
          }
        }
        if(stage2.active) {
          if(stage2.constructCandR(tagBuffer, RDOA, RDOAcombinations, stage1.getDm())) {
            if(stage2.update(stage2.yk)) {
              stage2.A = Eigen::Matrix<T, 6, 6>::Identity();
              stage2.A.topRightCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * stage2.dt;
              stage2.D = Eigen::Matrix<T, 6, 3>::Zero();
              stage2.D.topLeftCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * (stage2.dt*stage2.dt/2);
              stage2.D.bottomLeftCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * stage2.dt;
              status += 2;
              std::cout << std::endl << "Stage2 update, ts: "<< stage2.dt << std::endl;
              if(!stage3.active) {
                stage3.xHat(0) = stage2.xHat(0);
                stage3.xHat(1) = stage2.xHat(1);
                stage3.xHat(2) = stage2.xHat(2);
                stage3.active = true;
              }
            }
          }
        }
        
        if(stage2.active && stage3.active) {
          if(stage3.constructCandR(tagBuffer, RDOA, RDOAcombinations, stage2.xHat)) {
            if(stage3.update(stage3.yk)) {
              stage3.A = Eigen::Matrix<T, 6, 6>::Identity();
              stage3.A.topRightCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * stage3.dt;
              stage3.D = Eigen::Matrix<T, 6, 3>::Zero();
              stage3.D.topLeftCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * (stage3.dt*stage3.dt/2);
              stage3.D.bottomLeftCorner(3,3) = Eigen::Matrix<T, 3, 3>::Identity() * stage3.dt;
              //std::cout << "D:" << std::endl << stage3.D<< std::endl;
              status += 4;
              std::cout << std::endl << "Stage3 update" << std::endl;
            }
          }
        }
      }
      return status;
    }
    template <class T>
    bool XKF<T>::isInitialized(void) const {
      return initialized;
    }
    template <class T>
    void XKF<T>::setTimestep(T timestep) {
      stage2.dt=timestep;
      stage3.dt=timestep;
    }
    template <class T>
    void XKF<T>::setDiagonalCovarianceR(T rr_cov) {
      stage3.rr_cov = rr_cov;
      stage2.rr_cov = rr_cov;
    }
    template <class T>
    void XKF<T>::setVarianceRZ(T rz_var) {
      stage3.rz_cov = rz_var;
      stage2.rz_cov = rz_var;
    }
  }
}