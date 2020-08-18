#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve( Eigen::VectorXd state, 
                             Eigen::VectorXd coeffs);
    std::vector<double> mpc_x, mpc_y;                        

constexpr double pi() { return M_PI; }
double deg2rad(double x ) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
 
  std::vector<double> mpc_path_x_; // MPC's planned path x coords
  std::vector<double> mpc_path_y_;
};


#endif  // MPC_H
