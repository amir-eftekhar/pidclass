#include "PID.hpp"
#include <iostream>
#include <vector>
#include <cstdlib> 
#include <ctime>  

int main() {
    // Seed the random number generator for reproducibility
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // PID controller settings
    double kp = 0.1;
    double ki = 0.01;
    double kd = 0.05;
    double tolerance = 0.1;
    int max_iterations = 100; 
    double roc_threshold = 0.01; // Rate of change threshold
    std::chrono::seconds timeout(10); // 10 second timeout

    // Create PID object
    PID pid(kp, ki, kd, tolerance, max_iterations, roc_threshold, timeout);

    // Simulation setup
    double setpoint = 100.0; // Target setpoint
    double measure = 50.0; // Initial measurement
    pid.init(setpoint, measure);

    std::vector<double> measurements;
    measurements.push_back(measure);

    
    for (int i = 0; i < max_iterations; i++) {
        double control_signal = pid.compute(measure);
        
        
        measure += control_signal + (std::rand() % 20 - 10); // Random disturbance between -10 and 10
        measurements.push_back(measure);

        // Output the current state
        std::cout << "Iteration " << i << ": Control Signal = " << control_signal << ", Measure = " << measure << std::endl;

        // Check if exit conditions are met
        if (pid.checkExitConditions()) {
            std::cout << "Exit conditions met after " << i+1 << " iterations." << std::endl;
            break;
        }
    }

    // Check if maximum iterations were reached without meeting exit conditions
    if (!pid.checkExitConditions()) {
        std::cout << "Max iterations reached without meeting exit conditions." << std::endl;
    }

    return 0;
}
