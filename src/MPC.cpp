#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;                                      // Tunable parameter
double dt = 0.05;                                   // Tunable parameter  

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

// set references and maximum limits for variables
const double ref_cte = 0;
const double ref_epsi = 0;
const double ref_v = 120;                                               // Set this to 120 MPH if no time delay; 100 MPH for time delay                                     
const double max_cte = 5.0;
const double max_epsi = 25;
const double max_delta = 0.436332;


// set start positions for 1D vector used by Ipopt
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  Eigen::VectorXd initial_state;
  Eigen::VectorXd initial_control;
  double time_delay;

  // cost functions coefficients
  double adaptive_v;    
  double cte_coeff;
  double epsi_coeff;
  double ve_coeff;
  double delta_coeff;
  double a_coeff;
  double dslope_coeff;
  double aslope_coeff;
  double dcurve_coeff;
  double acurve_coeff;

  FG_eval(Eigen::VectorXd coeffs, Eigen::VectorXd state, Eigen::VectorXd control, double time_delay) { 
    this->coeffs = coeffs;
    this->time_delay = time_delay;
    initial_state = state;
    initial_control = control;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
  
    fg[0] = 0;

    // set cost function coefficients according to delay
    if (time_delay == 0){
      adaptive_v = ref_v*(1-0.25*tanh(fabs(initial_state[4]))-0.25*tanh(fabs(initial_state[5])));    // set adaptive velocity 
      cte_coeff = 1000;                                                                                                    
      epsi_coeff = 1000;                                                                                                   
      ve_coeff = 10000;                                                                                                     
      delta_coeff = 1;                                                                                                     
      a_coeff = 1;                                                                                                         
      dslope_coeff = 7500;                                                                                                  
      aslope_coeff = 1;                                                                                                   
      dcurve_coeff = 7500;                                                                                                 
      acurve_coeff = 1;                                                                                                   
    }
    else{
      adaptive_v = ref_v*(1-0.25*tanh(fabs(initial_state[4])));                                      // set adaptive velocity
      cte_coeff = 100;
      epsi_coeff = 100;
      ve_coeff = 7500;
      delta_coeff = 100;
      a_coeff = 1;
      dslope_coeff = 3000;
      aslope_coeff = 3000;
      dcurve_coeff = 3000;
      acurve_coeff = 3000; 
    }

    //Set up cost function

    for (int i =0; i < N; i++) {
      fg[0] += cte_coeff*CppAD::pow((vars[cte_start + i] - ref_cte)/(2*max_cte), 2);                          // cross track error                                                                                       
      fg[0] += epsi_coeff*CppAD::pow((vars[epsi_start + i] - ref_epsi)/(2*max_epsi), 2);                      // heading error                                            
      fg[0] += ve_coeff*CppAD::pow((vars[v_start + i] - adaptive_v)/(2*adaptive_v), 2);                       // velocity error                                       
    }

    for (int i =0; i < N - 1; i++) {
      fg[0] += delta_coeff*CppAD::pow(vars[delta_start + i]/(2*max_delta), 2);                                // delta magnitude error                                                                   
      fg[0] += a_coeff*CppAD::pow(vars[a_start + i]/2, 2);                                                    // acceleration error error                                         
    }

    for (int i = 0; i < N - 2; i++) {
      fg[0] += dslope_coeff *CppAD::pow((vars[delta_start + i +1] - vars[delta_start + i])/(2*max_delta), 2);  // delta slope error                                                       
      fg[0] += aslope_coeff*CppAD::pow((vars[a_start + i +1] - vars[a_start + i])/2, 2);                       // acceleration slope (jerk rate) error                                   
    } 

    for (int i = 0; i < N - 3; i++) {
      fg[0] += dcurve_coeff*CppAD::pow((vars[delta_start + i] - 2*vars[delta_start + i + 1] + vars[delta_start + i + 2])/(4*max_delta), 2); // delta curvature error           
      fg[0] += acurve_coeff*CppAD::pow((vars[a_start + i] - 2*vars[a_start + i + 1] + vars[a_start + i + 2])/4, 2);                         // acceleration curvature error      
    }

    // add errors with initial control inputs if delay zero  
    if(time_delay == 0.0){
      fg[0] += dslope_coeff *CppAD::pow((vars[delta_start] - initial_control[0])/(2*max_delta), 2);
      fg[0] += aslope_coeff*CppAD::pow((vars[a_start] - initial_control[1])/2, 2);
      fg[0] += dcurve_coeff*CppAD::pow((initial_control[0] - 2*vars[delta_start] + vars[delta_start + 1])/(4*max_delta), 2);          
      fg[0] += acurve_coeff*CppAD::pow((initial_control[1] - 2*vars[a_start] + vars[a_start + 1])/4, 2);                       
    }

    //We initialize the model to the initial state. 
    //fg[0] is reserved for the cost value, so the other indices are bumped up by 1.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int i = 0; i < N-1; i++) {
      // The state at time i + 2 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time i + 1.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time i + 1.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 *x0;
      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

      // use dynamic equations to setup fg
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }

  }
};

// MPC class definition implementation.
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd control, Eigen::VectorXd coeffs, double time_delay) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // extract state and control inputs
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  double steering = control[0];
  double throttle = control[1];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  size_t n_vars = N*6 + (N-1)*2;
  // Set the number of constraints
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all upper bounds and lowers bounds for all variables
  // except actuators to max negative or positve values
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower bounds for delta
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -max_delta;                                    // tunable parameter
    vars_upperbound[i] = max_delta;                                     // tunable parameter
  }

  // Acceleration upper and lower limits
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;                                          // tunable parameter
    vars_upperbound[i] = 1.0;                                           // tunable parameter
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // set constraints upper/lower bounds as initial states
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, state, control, time_delay);

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
  options += "Numeric max_cpu_time          0.5\n";         // Tunable parameter

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

  // Return the first actuator values and predicted x, y points in vehicle frame

  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i = 0; i < N; ++i)
  {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }


  return result;
}
