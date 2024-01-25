#include "MultipleReceiverEKF.hpp"
#include <iostream>
  //! Task that runst source position estimation algorithms for IMC::TBRFishTag
  //! @author Nikolai Lauvås
namespace FishTagEstimators
{
  std::tuple<double, double, double> MultipleReceiverEKF::getEstimate() {
    return {ekf.xHat(0),ekf.xHat(1),ekf.xHat(2)};
  }

  void MultipleReceiverEKF::initialize(const Eigen::Matrix<double, c_states, c_states> &A_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &Q_inn,
                                        const Eigen::Matrix<double, c_states, c_states> &P0_inn,
                                        const Eigen::Matrix<double, c_states, 1> &x0_inn)
  {
    name = "MultiReceiverEKF";
    ekf.A =     A_inn;
    ekf.Q =     Q_inn;
    ekf.PHat = P0_inn;
    ekf.xHat = x0_inn;
    ekf.active = true;
    Estimator::initialize(A_inn, Q_inn, P0_inn, x0_inn);
  }

  bool MultipleReceiverEKF::update(TagBuffer *tagBuffer) {
    //std::cout << "update"<< std::endl << std::endl;
    if(tagBuffer->tagBuffer.size() <3)
      return false;


    
    Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA(1  ,1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> depth(tagBuffer->tagBuffer.size(),1);
    ekf.ykest.resize(1,1);
    ekf.C.resize(1,c_states);
    tagBool_t used;

    unsigned baselines = 0;
    for (tagBufferMap_t::const_iterator outerreceiver = tagBuffer->tagBuffer.begin(); outerreceiver != tagBuffer->tagBuffer.end(); outerreceiver++) {
      for (tagBufferMap_t::const_iterator receiver = std::next(outerreceiver); receiver != tagBuffer->tagBuffer.end(); receiver++) {
        //inf("%u - %u", outerreceiver->first, receiver->first);
        if( unprocessedData[outerreceiver->first] || unprocessedData[receiver->first] ) {
          if( used.find(outerreceiver->first) == used.end() || used.find(receiver->first) == used.end()) {
            long int tempTDOA_ms = ((long int)outerreceiver->second->rbegin()->unix_timestamp - receiver->second->rbegin()->unix_timestamp)*1000 + ((int)outerreceiver->second->rbegin()->millis - receiver->second->rbegin()->millis);
            if(timeShiftCorrect(tempTDOA_ms)) {
              RDOA(RDOA.rows()-1,0) = c_speed*tempTDOA_ms/1000;
              RDOA.conservativeResize(RDOA.rows()+1,1);

              depth.row(baselines) << (outerreceiver->second->rbegin()->trans_data + receiver->second->rbegin()->trans_data)/2;

              if(ekf.active) {
                //std::cout << "xHat" <<ekf.xHat << std::endl; 
                Eigen::Matrix<double, c_states, 1> distance1 = ekf.xHat-getNED(*(outerreceiver->second->rbegin()));
                Eigen::Matrix<double, c_states, 1> distance2 = ekf.xHat-getNED(*(receiver->second->rbegin()));
                //std::cout << "Dist 1 and two: " << distance1 << ", " << distance2 << std::endl;
                double r1 = distance1.norm();//  ||X_e-X_rx0||
                double r2 = distance2.norm();// ||X_e-X_rx1||
                // Division by zero mitigation
                if(r1 == 0) {
                  r1=0.01;
                }
                if(r2 == 0) {
                  r2=0.01;
                }
                // Update ykest
                ekf.ykest(ekf.ykest.rows() -1,0) = r1 - r2;
                ekf.ykest.conservativeResize(ekf.ykest.rows()+1,1);
                //printf("%u - %u :: r1-r2: %lf, ekf: %lf\n", outerreceiver->first, receiver->first, r1 - r2, ekf.ykest(ekf.ykest.rows() -1,1));
                //std::cout << "ykest" << std::endl << ekf.ykest << std::endl;

                // Update C and R
                ekf.C.row(ekf.C.rows() -1) =  (distance1/r1) - (distance2/r2);
                ekf.C.conservativeResize(ekf.C.rows()+1,c_states);

                // Set data as used so that no baseline is with only old/used data
                used[outerreceiver->first] = false;
                used[receiver->first] = false;
                baselines++;
              }
            }
          }
        }
      }
    }


    if(baselines <2) {
      return false; // Do not process data/update filter if fewer than two baselines available
    }
    // Set unprocessedData to false for used data receivers
    for(tagBool_t::iterator it = used.begin();it != used.end();it++) {
      unprocessedData[it->first] = false;
    }

    // Add depth measurement
    double avgDepth = tagBuffer->getDepthConversionCoefficient()*depth.block(0,0,baselines,1).mean(); 
    ekf.ykest(ekf.ykest.rows()-1,0) = ekf.xHat(2,0);
    ekf.C.row(ekf.C.rows()-1) << 0, 0, 1;

    //std::cout << "RDOA" << std::endl << RDOA << std::endl;
    Eigen::VectorXd measurements(RDOA.cols()*RDOA.rows(),1);
    measurements << Eigen::Map<Eigen::VectorXd>(RDOA.data(), RDOA.cols()*RDOA.rows());
//return;
measurements.tail(1) << avgDepth;
    //measurements(RDOA.cols()*RDOA.rows()-1,1) = avgDepth;

//return;
    //ekf.R.resize(measurements.rows(), measurements.rows());
    ekf.R = rr_cov*Eigen::MatrixXd::Identity(ekf.C.rows(), ekf.C.rows());
    ekf.R(ekf.C.rows() - 1, ekf.C.rows() - 1) = rz_cov;
    //std::cout << "measurements" << std::endl << measurements << std::endl;
    //print(std::cout);
//return;
    return ekf.update(measurements);
  }
  
  void MultipleReceiverEKF::predict() {
    ekf.predict();
  }
  void MultipleReceiverEKF::print(std::ostream& os) const {
    os << "A" << std::endl << ekf.A << std::endl;
    os << "C" << std::endl << ekf.C << std::endl;
    os << "PHat" << std::endl << ekf.PHat << std::endl;
    os << "xHat" << std::endl << ekf.xHat << std::endl;
    os << "R" << std::endl << ekf.R << std::endl;
    os << "Q" << std::endl << ekf.Q << std::endl;
    os << "K" << std::endl << ekf.K << std::endl;
    os << "ykest" << std::endl << ekf.ykest << std::endl;
    os << "yk" << std::endl << ekf.yk << std::endl;
    os << "innov" << std::endl << ekf.innov << std::endl;
  }
  bool MultipleReceiverEKF::checkTime(double transmissionFirstTime, double currentTime) const {
    if(currentTime - transmissionFirstTime > 1.5) {
      return true;
    } else {
      return false;
    }
  }
}