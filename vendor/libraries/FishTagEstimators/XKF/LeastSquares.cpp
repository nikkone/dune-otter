#include "LeastSquares.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace FishTagEstimators
{
  namespace XKF
  {
    LeastSquares::LeastSquares() {
        xHat <<0,0,0;
    }

    double
    LeastSquares::resolveRAmbiguity(double R1, double R2)
    {
        double R_temp;
        double max_range = 700; // TODO: Move this elsewhere

        if((R1 > 0.0) && (R1 < max_range))
        {
            if((R2 > 0.0) && (R2 < max_range))
            {
            //printf("FishTagXKF2: resolveRAmbiguity: Cannot resolve ambiguity [Valid] %f %f", R1, R2);
            R_temp = R1; // choose one of them TODO: some trick here will help
            }
            else
            {
            //debug(DTR("FishTagXKF2: resolveRAmbiguity: R1 valid! %f %f"), R1, R2);
            R_temp = R1;
            }
        }
        else
        {
            if((R2 > 0.0) && (R2 < max_range))
            {
            //debug(DTR("FishTagXKF2: resolveRAmbiguity: R2 valid! %f %f"), R1, R2);
            R_temp = R2;
            }
            else
            {
            //debug(DTR("FishTagXKF2: resolveRAmbiguity: Cannot resolve ambiguity [Invalid] %f %f"), R1, R2);
            R_temp = 0;
            }
        }
        return R_temp;
    } // End resolveRAmbiguity function
    
    bool
    //LeastSquares::update(Eigen::Matrix<double, 3, Eigen::Dynamic>receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth)
    LeastSquares::update(TagBuffer *tagBuffer, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, std::vector<std::pair<uint32_t, uint32_t>> RDOAcombinations)
    {
      if(RDOA.rows() < 2) {
        return false;
      }
        bool isSuccess = 0;
//std::cout << "Entered FISHTAG::LeastSquares::update" <<  std::endl;
        uint32_t referenceReceiver = RDOAcombinations.front().first;
        Eigen::Matrix<double, 3, 1> referenceReceiverNED = Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[referenceReceiver]->rbegin())->N, (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->E,(tagBuffer->tagBuffer[referenceReceiver]->rbegin())->D);
//std::cout << "referenceReceiver: " << referenceReceiver << std::endl;
        // >> Construct Least squares and measurement matrices
        Eigen::Matrix<double, 3,1> l; // nja in paper
        l << RDOA(0), RDOA(1), 0;

        double tagDepth = (tagBuffer->tagBuffer[referenceReceiver]->rbegin())->getS256Depth();
        Eigen::Matrix<double, 3,3> Czq;
        Eigen::Matrix<double, 3,1> z; // Eq (8) in paper  (May also be Y)
        uint8_t used = 0,combination = 0;
        for(std::vector<std::pair<uint32_t, uint32_t>>::iterator it = RDOAcombinations.begin(); it != RDOAcombinations.end(); it++) {
            if(it->first == referenceReceiver) {
                Czq.row(used) << -(
                Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D) -
                referenceReceiverNED
                ).transpose();
                z.row(used) << RDOA(combination)*RDOA(combination) -  
                Eigen::Matrix<double, 3, 1>((tagBuffer->tagBuffer[it->second]->rbegin())->N, (tagBuffer->tagBuffer[it->second]->rbegin())->E,(tagBuffer->tagBuffer[it->second]->rbegin())->D).squaredNorm() + 
                referenceReceiverNED.squaredNorm();
                //std::cout << "NED-: " << ret << std::endl;
                used++;
            }
            if(used == 2) {
                Czq.row(used) << 0, 0, 0.5;
                z.row(used) << tagDepth;
                break;
            }
            combination++;
        }
        if(used < 2) {
            return false;
        }
        Eigen::Matrix<double, 3,3> invCzq = (Czq.transpose()*Czq).inverse()*Czq.transpose();//inverse(transpose(Czq)*Czq)*transpose(Czq);
        Eigen::Matrix<double, 3,1> c = invCzq*l; // nja overline
        Eigen::Matrix<double, 3,1> w = 0.5*invCzq*z; // Y overline
        // temp variables
        double ctc = c(0,0)*c(0,0) + c(1,0)*c(1,0) + c(2,0)*c(2,0);
        double aa = 1 - c.transpose()*c;
        double bb = 2*(referenceReceiverNED.transpose()*c-w.transpose()*c)(0);
        double cc = -1*(referenceReceiverNED - w).squaredNorm();
        // Compute solution for quadratic term
        double R1, R2;
        double max_range = 700;
        Eigen::Matrix<double, 3,1> fp_ls;
        if ((ctc == 1) || ((bb*bb - 4*aa*cc) <= 0.0))
        {
            if(ctc == 1) {
                R1 = -cc/bb;
            } else {
                R1 = -bb/(2*aa);
            }

            if((R1 > 0.0) && (R1 <= max_range)) {
                // Unique solution
                fp_ls = (R1*c + w);
                m_dr = R1;
                isSuccess = 1;
            }
            // Invalid solution
        } else {
            double s = sqrt(bb*bb - 4*aa*cc);
            R1 = (-bb + s)/(2*aa);
            R2 = (-bb - s)/(2*aa);
            R1 = resolveRAmbiguity(R1, R2);
            // Compute two candidate solutions - required to resolve ambiguity and select a particular R
            if(R1 == 0) {
                isSuccess = 0;
            } else {
                fp_ls = (R1*c + w);
                m_dr = R1;
                isSuccess = 1;
            }
        }

        // Compute position of the source for logging
        if(isSuccess)
        {
            xHat = fp_ls;
        }
        return isSuccess;
    } // End of LeastSquaresEstimate function

  }
}