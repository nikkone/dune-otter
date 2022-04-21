
namespace Ottermodel
{
    double Simulator::run(StateVector x, Eigen::Matrix<double,2,1>n, double mp, Eigen::Matrix<double,3,1>rp, double V_c, double beta_c) {
        /*
// State and current variables
  Eigen::Matrix<double,6,1> nu = x.block<6,1>(0,0);
  Eigen::Matrix<double,3,1> nu1 = x.block<3,1>(0,0);
  Eigen::Matrix<double,3,1> nu2 = x.block<3,1>(3,0);   // velocities
  Eigen::Matrix<double,6,1> eta = x.block<6,1>(6,0);   // positions
  double U = std::sqrt(pow(nu(1),2) +
                  pow(nu(2),2) + 
                  pow(nu(3),2));      // speed
  double u_c = V_c * std::cos(beta_c - eta(6));           // current surge velocity
  double v_c = V_c * std::sin(beta_c - eta(6));           // current sway velocity
  Eigen::Matrix<double,6,1> nu_r;// relative velocity vector
  nu_r << u_c, v_c, 0, 0, 0, 0;
  nu_r = nu - nu_r;

// Inertia dyadic, volume displacement and draft
double nabla = (m+mp)/rho;                         // volume
double T = nabla / (2 * Cb_pont * B_pont*L);       // draft
Eigen::DiagonalMatrix<double, 3> Ig_CG(pow(R44,2), pow(R55,2), pow(R66,2));
Ig_CG = m * Ig_CG;    // only hull in CG
Eigen::Matrix<double,3,1> rg = (m*rg + mp*rp)/(m+mp);           // CG location corrected for payload
Eigen::Matrix<double, 3,3> Ig = (Simulator::Smtrx(rg).pow(2.0)).matrix();
//Ig -= m * Simulator<double>::Smtrx(rg).pow(2);// - mp * Smtrx(rp)^2;  // hull + payload in CO

// Experimental propeller data including lever arms
double l1 = -1*y_pont;                           // lever arm, left propeller (m)
double l2 = y_pont;                            // lever arm, right propeller (m)
double k_pos = 0.02216/2;                      // Positive Bollard, one propeller 
double k_neg = 0.01289/2;                      // Negative Bollard, one propeller 
double n_max =  sqrt((0.5*24.4 * g)/k_pos);    // maximum propeller rev. (rad/s)
double n_min = -1*sqrt((0.5*13.6 * g)/k_neg);    // minimum propeller rev. (rad/s)

// MRB and CRB (Fossen 2021)
Eigen::Matrix<double,3,3> I3 = Eigen::Matrix<double,3,3>::Identity();
Eigen::Matrix<double,3,3> O3 = Eigen::Matrix<double,3,3>::Zero();

Eigen::Matrix<double,6,6> MRB_CG;
MRB_CG << (m+mp) * I3,O3,
           O3, Ig;
           
           /*
CRB_CG = [ (m+mp) * Smtrx(nu2)         O3
           O3               -Smtrx(Ig*nu2)  ];

H = Hmtrx(rg);              // Transform MRB and CRB from the CG to the CO 
MRB = H' * MRB_CG * H;
CRB = H' * CRB_CG * H;
*/
// Hydrodynamic added mass (best practise)
/*double Xudot = -0.1 * m;   
double Yvdot = -1.5 * m;
double Zwdot = -1.0 * m;
double Kpdot = -0.2 * Ig(1,1);
double Mqdot = -0.8 * Ig(2,2);
double Nrdot = -1.7 * Ig(3,3);*/
/*
MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);   
CA  = m2c(MA, nu_r);
CA(6,1) = 0; // Assume that the Munk moment in yaw can be neglected
CA(6,2) = 0; // These terms, if nonzero, must be balanced by adding nonlinear damping

// System mass and Coriolis-centripetal matrices
M = MRB + MA;
C = CRB + CA;

// Hydrostatic quantities (Fossen 2021)
Aw_pont = Cw_pont * L * B_pont;    // waterline area, one pontoon 
I_T = 2 * (1/12)*L*B_pont^3 * (6*Cw_pont^3/((1+Cw_pont)*(1+2*Cw_pont)))...
    + 2 * Aw_pont * y_pont^2;
I_L = 0.8 * 2 * (1/12) * B_pont * L^3;
KB = (1/3)*(5*T/2 - 0.5*nabla/(L*B_pont) );
BM_T = I_T/nabla;       // BM values
BM_L = I_L/nabla;
KM_T = KB + BM_T;       // KM values
KM_L = KB + BM_L;
KG = T - rg(3);
GM_T = KM_T - KG;       // GM values
GM_L = KM_L - KG;

G33 = rho * g * (2 * Aw_pont);      // spring stiffness
G44 = rho * g *nabla * GM_T;
G55 = rho * g *nabla * GM_L;

G_CF = diag([0 0 G33 G44 G55 0]);   // spring stiffness matrix in the CF
LCF = -0.2;
H = Hmtrx([LCF 0 0]);               // transform G_CF from the CF to the CO 
G = H' * G_CF * H;
*/
    }

    

}