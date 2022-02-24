#include "MultipleReceiverEKF.hpp"
namespace SourceEstimators
{
  //! Task that runst source position estimation algorithms for IMC::TBRFishTag
  //! @author Nikolai Lauvås
  namespace FishTag
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

    void MultipleReceiverEKF::update(const tagBuffer_t &tagBuffer, tagBool_t &unprocessedData) {
      //std::cout << "update"<< std::endl << std::endl;
      if(tagBuffer.size() <2)
        return;
      //int com = boost::math::binomial_coefficient<double>(tagBuffer.size(), 2);
      //spew("com: %i", com);
      Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA(1  ,1);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> NED(3,tagBuffer.size());
      Eigen::Matrix<double, Eigen::Dynamic, 1> depth(tagBuffer.size(),1);
      ekf.ykest.resize(1,1);
      ekf.C.resize(1,c_states);
      tagBool_t used;


// TODO: Fix that NED is calculated two times
      std::tuple<double, double, double> tempNED;
      unsigned combinations = 0;
      unsigned outer = 0;
      unsigned inner;
      unsigned baselines = 0;
      for (tagBuffer_t::const_iterator outerreceiver = tagBuffer.begin(); outerreceiver != tagBuffer.end(); outerreceiver++) {
        toNEDframe(*outerreceiver->second->rbegin(), tempNED);
        NED.col(outer) << std::get<0>(tempNED), std::get<1>(tempNED),std::get<2>(tempNED);
        inner = outer;
        for (tagBuffer_t::const_iterator receiver = std::next(outerreceiver); receiver != tagBuffer.end(); receiver++) {
          inner++;
          //inf("%u - %u", outerreceiver->first, receiver->first);
          if( unprocessedData[outerreceiver->first] || unprocessedData[receiver->first] ) {
            if( used.find(outerreceiver->first) == used.end() || used.find(receiver->first) == used.end()) {
              long int tempTDOA_ms = ((long int)outerreceiver->second->rbegin()->unix_timestamp - receiver->second->rbegin()->unix_timestamp)*1000 + ((int)outerreceiver->second->rbegin()->millis - receiver->second->rbegin()->millis);
              if(timeShiftCorrect(tempTDOA_ms)) {
                toNEDframe(*receiver->second->rbegin(), tempNED);
                NED.col(inner) << std::get<0>(tempNED), std::get<1>(tempNED),std::get<2>(tempNED);
                //inf("Usable TDOA: %li", tempTDOA_ms);
                RDOA(RDOA.rows()-1,0) = c_speed*tempTDOA_ms/1000;
                //RDOA.row(combinations) << c_speed*tempTDOA_ms/1000;
                RDOA.conservativeResize(RDOA.rows()+1,1);

                depth.row(baselines) << (outerreceiver->second->rbegin()->trans_data + receiver->second->rbegin()->trans_data)/2;

                if(ekf.active) {
                  Eigen::Matrix<double, c_states, 1> distance1 = ekf.xHat-NED.col(outer); // X_e-X_rx0
                  Eigen::Matrix<double, c_states, 1> distance2 = ekf.xHat-NED.col(inner); // X_e-X_rx1
                  double r1 = distance1.norm();//  ||X_e-X_rx0||
                  double r2 = distance2.norm();// ||X_e-X_rx1||

                  // Update ykest
                  ekf.ykest(ekf.ykest.rows() -1,0) = r1 - r2;
                  //printf("%u - %u :: r1-r2: %lf, ekf: %lf\n", outerreceiver->first, receiver->first, r1 - r2, ekf.ykest(ekf.ykest.rows() -1,1));
                  ekf.ykest.conservativeResize(ekf.ykest.rows()+1,1);
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
          combinations++;
        }
        outer++;
      }


      if(baselines <2) {
        return; // Do not process data/update filter if fewer than two baselines available
      }
      // Set unprocessedData to false for used data receivers
      for(tagBool_t::iterator it = used.begin();it != used.end();it++) {
        unprocessedData[it->first] = false;
      }

      // Add depth measurement
      double avgDepth = 0.392*depth.block(0,0,baselines,1).mean(); // Only valid for S256 tags with depth
      //inf("Depth: %lf, baselines: %u, alternative depth: %lf", avgDepth, baselines, 0.392*depth.block(0,0,baselines,1).sum()/baselines);
      ekf.ykest(ekf.ykest.rows()-1,0) = ekf.xHat(2,0);
      ekf.C.row(ekf.C.rows()-1) << 0, 0, 1;

      //std::cout << "RDOA" << std::endl << RDOA << std::endl;
      Eigen::Matrix<double, Eigen::Dynamic, 1> measurements(RDOA.cols()*RDOA.rows(),1);
      measurements << Eigen::Map<Eigen::VectorXd>(RDOA.data(), RDOA.cols()*RDOA.rows());
      measurements(measurements.rows() -1,1) = avgDepth;

      //ekf.R.resize(measurements.rows(), measurements.rows());
      ekf.R = rr_cov*Eigen::MatrixXd::Identity(ekf.C.rows(), ekf.C.rows());
      ekf.R(ekf.C.rows() - 1, ekf.C.rows() - 1) = rz_cov;
      //std::cout << "measurements" << std::endl << measurements << std::endl;
      ekf.update(measurements);
      //std::cout << "yk" << std::endl << ekf.yk << std::endl;
      
      //std::cout << "Updated" << std::endl;

      //std::cout << "R" << std::endl << ekf.R << std::endl;
      //std::cout << "C" << std::endl << ekf.C << std::endl;
      //inf("Measurements");
      //std::cout << measurements << std::endl;
      //inf("NED");
      //std::cout << NED << std::endl;
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
  }
}