#include "PID.hpp"
#include <cmath> 
#include <chrono>

// Constructor definition
PID::PID(double kp, double ki, double kd, double tolerance, int max_iterations, double roc_threshold, std::chrono::seconds timeout_duration)
    : kp_(kp), ki_(ki), kd_(kd), tolerance_(tolerance), max_iterations_(max_iterations),
      rate_of_change_threshold_(roc_threshold), timeout_(timeout_duration),
      integral_(0), last_error_(0), iterations_(0), initialized_(false) {}

void PID::init(double setpoint, double initial_measure) {
    setpoint_ = setpoint;
    integral_ = 0;
    last_error_ = setpoint - initial_measure;
    iterations_ = 0;
    initialized_ = true;
    start_time_ = std::chrono::steady_clock::now();
}

void PID::reset() {
    integral_ = 0;
    last_error_ = 0;
    iterations_ = 0;
    initialized_ = false;
}

double PID::compute(double measure) {
    if (!initialized_ || checkExitConditions()) {
        return 0;
    }

    double error = setpoint_ - measure;
    double derivative = error - last_error_;
    if (std::fabs(derivative) <= rate_of_change_threshold_ && iterations_ > 0) {
        return 0;
    }

    integral_ += error;
    last_error_ = error;
    iterations_++;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

bool PID::checkExitConditions() {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
    return (std::fabs(last_error_) <= tolerance_ || iterations_ >= max_iterations_ || elapsed >= timeout_);
}
