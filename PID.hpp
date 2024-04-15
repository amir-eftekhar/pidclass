#ifndef PID_HPP
#define PID_HPP

#include <chrono> // For timing

class PID {
public:
    PID(double kp, double ki, double kd, double tolerance, int max_iterations, double roc_threshold, std::chrono::seconds timeout_duration);
    void init(double setpoint, double initial_measure);
    void reset();
    double compute(double measure);
    bool checkExitConditions();

private:
    double kp_, ki_, kd_;     // PID gains
    double setpoint_;         // Desired value
    double tolerance_;        // Tolerence for stopping condition
    double integral_;         // Integral term
    double last_error_;       // Last error value for derivative term
    double rate_of_change_threshold_; // Threshold for rate of change of error
    int iterations_;  // Count of iterations
    int max_iterations_; // Max iteations allowed
    bool initialized_;        // Initzilization status
    std::chrono::steady_clock::time_point start_time_; // Start time of the PID control
    std::chrono::seconds timeout_; // Maximum duration to run the PID controller
};

#endif
