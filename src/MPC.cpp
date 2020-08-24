#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;
using Eigen::VectorXd;
using std::string;

double deg2rad(double x ) { return x * M_PI / 180; }
 
 /**
 * Set the timestep length and duration (sample window).
 */
const size_t N = 8;
double dt = 0.15;

//Set the goal speed.
double target_v = 120;

// For every state values set an offset.
size_t x_offset = 0;
size_t y_offset = x_offset + N;
size_t psi_offset = y_offset + N;
size_t v_offset = psi_offset + N;
size_t cte_offset = v_offset + N;
size_t epsi_offset = cte_offset + N;
size_t angle_offset = epsi_offset + N;
size_t acc_offset = angle_offset + N - 1;


// This value was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) {
   this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     *  implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     */
   
     // Panalize values:
     double penalize_cte = 100;
     double penalize_espi = 200000;
     double penalize_v_diff_form_t = 3;
     double penalize_angle_diff = 150000;

     
     // Sum the cost into index 0.
     fg[0] = 0;

     for ( int t = 0; t < N; t++){
        fg[0] += penalize_cte * CppAD::pow(vars[t + cte_offset], 2);
        fg[0] += penalize_espi * CppAD::pow(vars[t + epsi_offset], 2);
        fg[0] += penalize_v_diff_form_t * CppAD::pow(target_v -vars[t + v_offset] , 2);
        }


     for(int t = 0; t < N - 2; t++){
        fg[0] += penalize_angle_diff * CppAD::pow(vars[angle_offset + t + 1] - vars[angle_offset + t] , 2);
        fg[0] += CppAD::pow(vars[acc_offset + t + 1] - vars[acc_offset + t ] , 2);
        }
        
     // Set constrains:

     fg[1 + x_offset] = vars[x_offset];
     fg[1 + y_offset] = vars[y_offset];
     fg[1 + psi_offset] = vars[psi_offset];
     fg[1 + v_offset] = vars[v_offset];
     fg[1 + cte_offset] = vars[cte_offset];
     fg[1 + epsi_offset] = vars[epsi_offset];
     
     //For each timestamp 
     for(int t = 1; t < N; t++){
        //Set state in t-1
        CppAD::AD<double> x0 = vars[x_offset + t - 1];     
        CppAD::AD<double> y0 = vars[y_offset + t - 1];     
        CppAD::AD<double> psi0 = vars[psi_offset + t - 1];        
        CppAD::AD<double> v0 = vars[v_offset + t - 1];       
        CppAD::AD<double> cte0 = vars[cte_offset + t - 1];        
        CppAD::AD<double> epsi0 = vars[epsi_offset + t - 1];        
          
        //Set state in t                
        CppAD::AD<double> x1 = vars[x_offset + t];
        CppAD::AD<double> y1 = vars[y_offset + t];
        CppAD::AD<double> psi1 = vars[psi_offset + t];
        CppAD::AD<double> v1 = vars[v_offset + t];
        CppAD::AD<double> cte1 = vars[cte_offset + t];
        CppAD::AD<double> epsi1 = vars[epsi_offset + t];
         
        // Actuation at time t-1 
        CppAD::AD<double> angle = vars[angle_offset + t -1];
        CppAD::AD<double> acc = vars[acc_offset + t - 1];

        CppAD::AD<double> f = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        CppAD::AD<double> psi_des = CppAD::atan(coeffs[1] + 2*coeffs[2] * x0 + 3*coeffs[3] * CppAD::pow(x0, 2));
         
        //Model's equations
        fg[1 + t + x_offset] = x1 - ( x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + t + y_offset] = y1 -( y0 + v0 * CppAD::sin(psi0) * dt);
        
        fg[1 + t + psi_offset] = psi1 -(psi0 - (v0 / Lf) * angle * dt);
        fg[1 + t + v_offset] = v1 - (v0 +acc * dt);
        
        fg[1 + t + cte_offset] = cte1 -((f - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + t + epsi_offset] = epsi1 -((psi0 - psi_des) -( (v0 / Lf) * angle * dt));

       }
     }
};

//
// MPC class definition implementation.
///
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = 6 * N + 2* (N - 1);
  /**
   * Set the number of constraints
   */
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_offset] = state[0];
  vars[v_offset] = state[1];
  vars[psi_offset] = state[2];
  vars[v_offset] = state[3];
  vars[cte_offset] = state[4];
  vars[epsi_offset] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /**
   * Set lower and upper limits for variables.
   */
   
  // Max positive and negative values.
  for(int i = 0; i < angle_offset; i++){
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
  }
  // Limits on steering angles : [-25, 25]
  for(int i = angle_offset; i < acc_offset; i++){
     vars_lowerbound[i] = deg2rad(-25);
     vars_upperbound[i] = deg2rad(25);  
  }
  // Limits on throttle use[1, -1] 
  //(negative throttle = break)
  for(int i = acc_offset; i < n_vars; i++){
     vars_lowerbound[i] = -1.0;
     vars_upperbound[i] = 1.0;  
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);

  constraints_lowerbound[x_offset] = state[0];
  constraints_lowerbound[y_offset] = state[1];
  constraints_lowerbound[psi_offset] = state[2];
  constraints_lowerbound[v_offset] = state[3];
  constraints_lowerbound[cte_offset] = state[4];
  constraints_lowerbound[epsi_offset] = state[5];
  
  Dvector constraints_upperbound(n_constraints);
  
  constraints_upperbound[x_offset] = state[0];
  constraints_upperbound[y_offset] = state[1];
  constraints_upperbound[psi_offset] = state[2];
  constraints_upperbound[v_offset] = state[3];
  constraints_upperbound[cte_offset] = state[4];
  constraints_upperbound[epsi_offset] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);


  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
 
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
 // std::cout << "Cost " << cost << std::endl;

  /**
   * Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */
 

   // Set actuation vector and add Ipopt solutions.
   std::vector<double> actuation;

   actuation.push_back(solution.x[angle_offset]);
   actuation.push_back(solution.x[acc_offset]);
   
  
  return actuation;
}
