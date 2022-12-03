#include "LeastSquares.hpp"
#include <Eigen/Dense>
namespace SourceEstimators
{
  namespace MultipleReceiverXKF
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
    LeastSquares::update(Eigen::Matrix<double, 3, Eigen::Dynamic>receiverPositions, Eigen::Matrix<double, Eigen::Dynamic, 1> RDOA, double tagDepth)
    {
      if(RDOA.rows() < 2) {
        return false;
      }
        bool isSuccess = 0;
std::cout << "Entered LeastSquares::update" <<  std::endl;
//std::cout << "receiverPositionsnew" <<  std::endl << receiverPositions << std::endl; 
//std::cout << "RDOAnew" <<  std::endl << RDOA << std::endl; 
        // >> Construct Least squares and measurement matrices
        Eigen::Matrix<double, 3,1> l; // nja in paper
        l << RDOA(0), RDOA(1), 0;
//std::cout << "l" <<  std::endl << l << std::endl; 

        Eigen::Matrix<double, 3,3> Czq;

        //Czq << -(receiverPositions(0,0) - receiverPositions(0,2)), -(receiverPositions(1,0) - receiverPositions(1,2)), 0.0,
        //       -(receiverPositions(0,1) - receiverPositions(0,2)), -(receiverPositions(1,1) - receiverPositions(1,2)), 0.0,
        //        0,                0, 0.5;
        Czq << -(receiverPositions.col(1) - receiverPositions.col(0)).transpose(),
               -(receiverPositions.col(2) - receiverPositions.col(0)).transpose(),
                0,                0, 0.5;

//std::cout << "czq" <<  std::endl << Czq << std::endl;
        Eigen::Matrix<double, 3,1> z;
        // Eq (8) in paper (TODO: Update to eigen squaredNorm()) (May also be Y)
        //z << (RDOA(1)*RDOA(1) - receiverPositions(0,0)*receiverPositions(0,0) - receiverPositions(1,0)*receiverPositions(1,0) + receiverPositions(0,2)*receiverPositions(0,2) + (receiverPositions(1,2))*(receiverPositions(1,2))),
        //     (RDOA(0)*RDOA(0) - receiverPositions(0,1)*receiverPositions(0,1) - receiverPositions(1,1)*receiverPositions(1,1) + receiverPositions(0,2)*receiverPositions(0,2) + (receiverPositions(1,2))*(receiverPositions(1,2))),
        //     tagDepth;
        z << (RDOA(0)*RDOA(0) - receiverPositions.col(1).squaredNorm() + receiverPositions.col(0).squaredNorm()),
             (RDOA(1)*RDOA(1) - receiverPositions.col(2).squaredNorm() + receiverPositions.col(0).squaredNorm()),
             tagDepth;

//std::cout << "z" <<  std::endl << z << std::endl;
        Eigen::Matrix<double, 3,3> invCzq = (Czq.transpose()*Czq).inverse()*Czq.transpose();//inverse(transpose(Czq)*Czq)*transpose(Czq);
        Eigen::Matrix<double, 3,1> c = invCzq*l; // nja overline
        Eigen::Matrix<double, 3,1> w = 0.5*invCzq*z; // Y overline
//std::cout << "Calculations" <<  std::endl;
        // temp variables
        double ctc = c(0,0)*c(0,0) + c(1,0)*c(1,0) + c(2,0)*c(2,0);
        //double ptc = receiverPositions(0,2)*c(0,0) +  receiverPositions(1,2)*c(1,0);
        //double wtc = w(0,0)*c(0,0) + w(1,0)*c(1,0) + w(2,0)*c(2,0);
        //double ptw = receiverPositions(0,2)*w(0,0) +  receiverPositions(1,2)*w(1,0);
        //double wtw = w(0,0)*w(0,0) + w(1,0)*w(1,0) + w(2,0)*w(2,0);
        //double ptp = (receiverPositions(0,2))*(receiverPositions(0,2)) + (receiverPositions(1,2))*(receiverPositions(1,2));
    //std::cout << "Created Temp vars" <<  std::endl;
//std::cout << "ctc" <<  std::endl << ctc << std::endl;
        // quadratic equation coefficients for computation of d3 = m_dr, p = p_r
        //double aa = 1 - ctc; // 1 - c'c
        //double bb = 2*(ptc - wtc); // 2(p'c - w'c)
        //double cc = 2*ptw - wtw - ptp; //2p'w - w'w - p'p

        double aa = 1 - c.transpose()*c;
        double bb = 2*(receiverPositions.col(0).transpose()*c-w.transpose()*c)(0);
        double cc = -1*(receiverPositions.col(0) - w).squaredNorm();
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
            //m_rlog.precision(15);
            //m_rlog << Clock::getSinceEpochMsec() << "," << R1 << "," << R2 << std::endl;
//std::cout << "R1" <<  std::endl << R1 << std::endl;
//std::cout << "R2" <<  std::endl << R2 << std::endl;
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