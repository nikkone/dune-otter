// CURRRENTLY NOT FUNCTIONAL

#include <Eigen/Core>

#ifndef OTTERMODEL_Simulator
#define OTTERMODEL_Simulator
namespace Ottermodel
{
    class Simulator {

      // trim: theta = -7.5 deg corresponds to 13.5 cm less height aft maximum load
      double trim_setpoint = 280;

      // trim_setpoint is a step input, which is filtered using the state trim_moment
      double static trim_moment;

    /* if isempty(trim_moment)
        trim_moment = 0;
      end*/

      // Main data
      const double g   = 9.81;         // acceleration of gravity (m/s^2)
      const double rho = 1025;         // density of water
      const double L = 2.0;            // length (m)
      const double B = 1.08;           // beam (m)
      const double m = 55.0;           // mass (kg)
      //double rg = [0.2 0 -0.2]'; // CG for hull only (m)
      const double R44 = 0.4 * B;      // radii of gyrations (m)
      const double R55 = 0.25 * L;
      const double R66 = 0.25 * L;
      const double T_yaw = 1;          // time constant in yaw (s)
      const double Umax = 6 * 0.5144;  // max forward speed (m/s)

      // Data for one pontoon
      const double B_pont  = 0.25;     // beam of one pontoon (m)
      const double y_pont  = 0.395;    // distance from centerline to waterline area center (m)
      const double Cw_pont = 0.75;     // waterline area coefficient (-)
      const double Cb_pont = 0.4;      // block coefficient, computed from m = 55 kg

      public:
        typedef Eigen::Matrix<double,12,1> StateVector;
        double run(StateVector x, Eigen::Matrix<double,2,1>n, double mp, Eigen::Matrix<double,3,1>rp, double V_c = 0, double beta_c = 0);

        //! S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
        //! The corss product satisfies: a x b = S(a)b. The inverse can be computed
        //! as: vex(S(a)) = a 
        Eigen::Matrix<double,3,3> Smtrx(Eigen::Matrix<double,3,1> a) {
          Eigen::Matrix<double,3,3> S;
          S << 0,    -a(3), a(2),
              a(3), 0    , -a(1),
              -a(2), a(1) , 0;
          return S;
        }
    };
/*    template<typename double>
    class StateVector {
//    x = [ u v w p q r x y z phi theta psi ]' 
      double u;      // surge velocity          (m/s)
      double v;      // sway velocity           (m/s)
      double w;      // heave velocity          (m/s)
      double p;      // roll velocity           (rad/s)
      double q;      // pitch velocity          (rad/s)
      double r;      // yaw velocity            (rad/s)
      double x;      // position in x direction (m)
      double y;      // position in y direction (m)
      double z;      // position in z direction (m)
      double phi;    // roll angle              (rad)
      double theta;  // pitch angle             (rad)
      double psi;    // yaw angle               (rad)
    }*/
}
#include "simulator.tpp"
#endif