// Example of how to implement a simple controller
// The controller gets the current state from KPS, and returns the actuator torques on the satellite
// The code is compartmentalized - all the implementation details for the estimator and controller should be contained herein, and separated from the higher-level operation of KPS
//
// The attitude actuation method is assumed to be able to arbitrarily apply torque to all three axes, but the details of the actuator are not defined
// Reaction wheels are one method of providing 3 axis actuation
// As a reference of scale, a typical Cubesat reaction wheel assembly should be able to provide torque on the order of 0.1 to 1 milliNewton-meters

#pragma once

#include "State.h"
#include "glm_util.h"
#include "Eigen_util.h"

class Controller {
public:
	
	// public members

private:

	// satellite moment of inertia in body coordinates
	const mat3 MOI;

public:
	
	Controller(const mat3 sat_MOI) : MOI(sat_MOI) {}

	vec3 getControlTorque(const double t, const State& current_state);

private:

	vec3 proportionalFeedback(const double t, const State& current_state, const quat desired_q, const vec3 desired_w);

};


