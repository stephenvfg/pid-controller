#include "Twiddle.h"
#include <limits>
#include <string>
#include <vector>
#include <iostream>

using std::vector;
using std::string;
using std::numeric_limits;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

/**
 * Initialize Twiddle.
 * @param (tol) the tolerance of the Twiddle algorithm
 * @param (Kp, Ki, Kd) the initial PID parameters
 */
void Twiddle::Init(double tol, double Kp, double Ki, double Kd) {
	// Set the relevant Twiddle parameters
	this->tol = tol;
	this->p = {Kp, Ki, Kd};

  this->param = 0; // indicates which param we are updating in p
  this->phase = 0; // indicates which phase we are in (increase p, decrease p, or decrease dp)

  this->first_run = true; // holds whether or not Twiddle is on its first full cycle
	this->iteration = 0; // current iteration since the last reset
  this->max_iteration = 10000; // max number of iterations to run for until checking the error

  // store the parameters and the parameter delta values
  this->dp = {Kp/30, Ki/30, Kd/30};

  // set maximum error value
  this->total_err = 0.0;
  this->best_err = numeric_limits<double>::max();

  // values we will use to scale dp up or down
  this->scale_up = 1.1;
  this->scale_down = 0.9;

  // not yet time to reset
  this->time_to_reset = false;
  this->finished = false;

  // if this value is true then display output showing Twiddle values to help debug
  this->twiddle_debug_mode = false; // CHANGE TO TRUE TO DEBUG
}

/**
 * Continues to have the vehicle run until it's ready to update.
 * @param (err) error value of the current iteration
 */
void Twiddle::Run(double err) {
	if (iteration == 0) {
		time_to_reset = false;
	}

	total_err += err;
	iteration += 1;

	if (iteration >= max_iteration) {
		if (twiddle_debug_mode) {
  		std::cout << "P: " << p[0] << " I: " << p[1] << " D: " << p[2] << std::endl;
  		std::cout << "dP: " << dp[0] << " dI: " << dp[1] << " dD: " << dp[2] << std::endl;
  	}
		this->Update();
	}
}

/**
 * Updates error values based on the last run.
 */
void Twiddle::Update() {
	// if it's the first cycle, simply take the current error as the best error
	if (first_run) {
		// capture the current error and then update one coefficient to compare in the next cycle
		best_err = total_err;
		p[param] += dp[param];
		phase = 0;
		first_run = false;
		if (twiddle_debug_mode) {
			std::cout << "Best error so far: " << best_err << std::endl;
		}
	} 
	// adjust the coefficients and modifiers according to the phase
	else if (phase == 0) {
  	if (total_err < best_err) {
  		best_err = total_err;
  		dp[param] *= scale_up;
  		param = (param+1)%p.size();
			if (twiddle_debug_mode) {
  			std::cout << "*********** New best error for above params: " << best_err << std::endl;
  		}
  	} else {
  		p[param] -= 2.0*dp[param];
			phase = 1;
  	}
  } else if (phase == 1) {
  	if (total_err < best_err) {
  		best_err = total_err;
  		dp[param] *= scale_up;	
  		// prepare for next cycle
			param = (param+1)%p.size();
			p[param] += dp[param];
			phase = 0;
			if (twiddle_debug_mode) {
  			std::cout << "*********** New best error for above params: " << best_err << std::endl;
  		}
  	} else {
  		p[param] += dp[param];
  		dp[param] *= scale_down;
  		// prepare for next cycle
			param = (param+1)%p.size();
			p[param] += dp[param];
			phase = 0;
  	}
  }
	if (twiddle_debug_mode) {
  	std::cout << "Error for above params: " << total_err << std::endl;
  }
	this->Reset();
}

/**
 * Returns whether or not the Twiddle algorithm is complete based on tolerance.
 */
bool Twiddle::Finished() {
	// if not finished, check to see if it's time to finish
	if (!finished) {
		double sum = 0.0;
		for (int i = 0; i < dp.size(); i++) {
			sum += dp[i];
		}
		if (sum < tol) {
			finished = true;
			std::cout << "***FINISHED CALIBRATING***" << std::endl;
		}
	}

	// return current value of finished
	return finished;
}

/**
 * Resets the iterations and vehicle to start a new phase.
 */
void Twiddle::Reset() {
	iteration = 0;
	total_err = 0.0;
	time_to_reset = true;
}

/**
 * Returns the optimized Twiddle parameters once it has finished running.
 */
vector<double> Twiddle::Parameters() {
	return p;
}

/**
 * Returns whether or not it is time to reset the vehicle.
 */
bool Twiddle::TimeToReset() {
	return time_to_reset;
}