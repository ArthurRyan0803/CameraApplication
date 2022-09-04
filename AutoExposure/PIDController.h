#pragma once


class PIDController
{
	/* Controller gains */
	double kp_ = 0;
	double ki_ = 0;
	double kd_ = 0;

	/* Derivative low-pass filter time constant */
	double tau_ = 0.02;

	/* Output limits */
	double lim_min_;
	double lim_max_;
	
	/* Integrator limits */
	double lim_min_int_ = -5.0;
	double lim_max_int_ = 5.0;

	/* Sample time (in seconds) */
	double t_ = 0.01;

	/* Controller "memory" */
	double integrator_ {0};			// integral
	double prev_error_ {0};			/* Required for integrator */

	double differentiator_ {0};		// differential
	double prev_measurement_ {0};		/* Required for differentiator */

	/* Controller output */
	double out_ = 0;

public:
	PIDController(double kp, double ki, double kd, double min, double max);

	double update(double set_point, double measurement);
};

