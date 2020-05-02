#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <limits>
#include <string>
#include <vector>

using std::vector;
using std::string;
using std::numeric_limits;

class Twiddle {
 public:
  /**
   * Constructor
   */
  Twiddle();

  /**
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Initialize Twiddle.
   * @param (tol) the tolerance of the Twiddle algorithm
   * @param (Kp, Ki, Kd) the initial PID parameters
   */
  void Init(double tol, double Kp, double Ki, double Kd);

  /**
   * Continues to have the vehicle run until it's ready to update.
   * @param (err) error value of the current iteration
   */
  void Run(double err);

  /**
   * Updates error values based on the last run.
   * @param (err) error value of the current iteration
   */
  void Update(double err);

  /**
   * Returns whether or not the Twiddle algorithm is complete based on tolerance.
   */
  bool Finished();

  /**
   * Resets the iterations and vehicle to start a new phase.
   */
  void Reset();

  /**
   * Returns the optimized Twiddle parameters once it has finished running.
   */
  vector<double> Parameters();

  /**
   * Returns whether or not it is time to reset the vehicle.
   */
  bool TimeToReset();

 private:
  
  double tol;
  int param; // indicates which param we are updating in p
  int phase; // indicates which phase we are in (increase p, decrease p, or decrease dp)

  bool first_run; // holds whether or not Twiddle is on its first full cycle
  int iteration; // current iteration since the last reset
  int max_iteration; // max number of iterations to run for until checking the error

  // store the parameters and the parameter delta values
  vector<double> p;
  vector<double> dp;

  // best error value seen so far
  double best_err;

  // values we will use to scale dp up or down
  double scale_up;
  double scale_down;

  // stores whether or not its time to reset the vehicle
  bool time_to_reset;

  // stores when Twiddle is done calibrating
  bool finished;
};

#endif  // TWIDDLE_H