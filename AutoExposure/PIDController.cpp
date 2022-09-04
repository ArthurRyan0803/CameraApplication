#include "PIDController.h"

#include <algorithm>


PIDController::PIDController(double kp, double ki, double kd, double min, double max): kp_(kp), ki_(ki), kd_(kd), lim_min_(min), lim_max_(max)
{

}

double PIDController::update(double set_point, double measurement)
{
	// Error signal
    double error = set_point - measurement;
    
	// Proportional
    double proportional = kp_ * error;
    
	// Integral
    integrator_ = integrator_ + 0.5 * ki_ * t_ * (error + prev_error_);

	// Anti-wind-up via integrator clamping
    integrator_ = integrator_ < lim_max_int_ ? integrator_: lim_max_int_;
    integrator_ = integrator_ > lim_min_int_ ? integrator_: lim_min_int_;

    //  Derivative (band-limited differentiator)
    differentiator_ = -(2.0 * kd_ * (measurement - prev_measurement_)	// Note: derivative on measurement, therefore minus sign in front of equation!
                        + (2.0 * tau_ - t_) * differentiator_)
                        / (2.0 * tau_ + t_);

    // Compute output and apply limits
    out_ = proportional + integrator_ + differentiator_;

    if (out_ > lim_max_) {

        out_ = lim_max_;

    } else if (out_ < lim_min_) {

        out_ = lim_min_;

    }

	/* Store error and measurement for later use */
    prev_error_       = error;
    prev_measurement_ = measurement;

	/* Return controller output */
    return out_;
}
