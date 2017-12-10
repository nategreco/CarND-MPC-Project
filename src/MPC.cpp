#include "MPC.h"
#include <cppad/cppad.hpp>
#define HAVE_STDDEF_H
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include "Eigen-3.3/Eigen/Core"
#include <limits>
#include <math.h>

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Setpoints
const double v_sp = 90.0;

// Start positions in vars
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

// Weights
const double w_cte = 2000.0;
const double w_psi_err = 1500.0;
const double w_v_diff = 1.0;
const double w_act_str = 100.0;
const double w_act_accel = 100.0;
const double w_gap_str = 10000.0;
const double w_gap_accel = 1000.0;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Initialize cost
    fg[0] = 0;
      
    // The part of the cost based on the reference state.
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_psi_err * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w_v_diff * CppAD::pow(vars[v_start + t] - v_sp, 2);
    }

    // Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += w_act_str * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_act_accel * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += w_gap_str * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_gap_accel * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // Constraints
    fg[x_start + 1] = vars[x_start];
    fg[y_start + 1] = vars[y_start];
    fg[psi_start + 1] = vars[psi_start];
    fg[v_start + 1] = vars[v_start];
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];
    for (unsigned int i = 0; i < N - 1; ++i) {
      // Time t0 variables
      AD<double> x_0 = vars[x_start + i];
      AD<double> y_0 = vars[y_start + i];
      AD<double> psi_0 = vars[psi_start + i];
      AD<double> v_0 = vars[v_start + i];
      AD<double> cte_0 = vars[cte_start + i];
      AD<double> epsi_0 = vars[epsi_start + i];
      AD<double> f_0 = coeffs[0] +
                       coeffs[1] * x_0 +
                       coeffs[2] * CppAD::pow(x_0, 2) +
                       coeffs[3] * CppAD::pow(x_0, 3);
      AD<double> psides_0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_0 + 3 * coeffs[3] * CppAD::pow(x_0, 2));
      AD<double> delta = vars[delta_start + i];
      AD<double> a = vars[a_start + i];
      
      // Time t1 variables
      AD<double> x_1 = vars[x_start + i + 1];
      AD<double> y_1 = vars[y_start + i + 1];
      AD<double> psi_1 = vars[psi_start + i + 1];
      AD<double> v_1 = vars[v_start + i + 1];
      AD<double> cte_1 = vars[cte_start + i + 1];
      AD<double> epsi_1 = vars[epsi_start + i + 1];

      // Calculate other constraints
      fg[1 + x_start + i] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      fg[1 + y_start + i] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      fg[1 + psi_start + i] = psi_1 - (psi_0 - v_0 / Lf * delta * dt);
      fg[1 + v_start + i] = v_1 - (v_0 + a * dt);
      fg[1 + cte_start + i] = cte_1 - ((f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt));
      fg[1 + epsi_start + i] = epsi_1 - ((psi_0 - psides_0) - v_0 / Lf * delta * dt);
    }
    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + 2 * (N - 1);
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  
  // Set initial state variables
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  // Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // No limits 
  for (unsigned int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  
  // Turning limits (radians)
  for (unsigned int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = (M_PI / 180.0) * -turn_lim;
    vars_upperbound[i] = (M_PI / 180.0) * turn_lim;
  }
  
  // Accel/decel limits (normalized)
  for (unsigned int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // Set initial state variables
  constraints_lowerbound[x_start] = state[0];
  constraints_upperbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_upperbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_upperbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_upperbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_upperbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
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
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  // Create values vector
  std::vector<double> values = {
    solution.x[delta_start],
    solution.x[a_start]
  };

  // Get x and y points
  for (unsigned int i = 1; i < N; ++i) {
    values.push_back(solution.x[x_start + i]);
    values.push_back(solution.x[y_start + i]);
  }

  // Return solution values
  return values;
}
