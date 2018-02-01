#include "Controller.h"

#include <iostream>

// takes the time, and current state of the satellite (i.e. the position, velocity, orientation (as a quaternion), and angular velocity (in the body frame coordinates))
//
// returns the actuator torques on the satellite, in each of the three body axes, in the body frame coordinates
vec3 Controller::getControlTorque(const double t, const State& current_state) {

	// example of a completely arbitrary control sequence

	if (t < 60*3) {
		// minutes 1 to 10, controller off
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60*5) {
		// minutes 11 to 20, steer the satellite to a desired orientation
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else if (t < 60 * 10) {
		// minutes 11 to 20, steer the satellite to a desired orientation
		return vec3{ 0.0, 0.0, 0.00006 };
	} 
	else if (t < 60 * 13) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 15) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else if (t < 60 * 20) {
		return vec3{ 0.0, 0.0, 0.00007 };
	}
	else if (t < 60 * 23) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 25) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60*30) {
		return vec3{ 0.0, 0.0, 0.00008 };
	}
	else if (t < 60 * 33) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 35) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60 * 40) {
		return vec3{ 0.0, 0.0, 0.00009 };
	}
	else if (t < 60 * 43) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 45) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  {
		return vec3{ 0.0, 0.0, 0.0001 };
	}

	/*
	else if (t < 60 * 53) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 55) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60 * 60) {
		return vec3{ 0.0, 0.0, 0.00006 };
	}
	else if (t < 60 * 63) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 65) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60 * 70) {
		return vec3{ 0.0, 0.0, 0.00007 };
	}
	else if (t < 60 * 73) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 75) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60 * 80) {
		return vec3{ 0.0, 0.0, 0.00008 };
	}
	else if (t < 60 * 83) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 85) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  if (t< 60 * 90) {
		return vec3{ 0.0, 0.0, 0.00009 };
	}
	else if (t < 60 * 93) {
		quat desired_q{ 0.525,{ 0.592, 0.158, 0.592 } };

		vec3 desired_w{ 0.0, 0.0, 0.0 };

		return proportionalFeedback(t, current_state, desired_q, desired_w);
	}
	else if (t < 60 * 95) {
		return vec3{ 0.0, 0.0, 0.0 };
	}
	else  {
		return vec3{ 0.0, 0.0, 0.0001 };
	}
	*/

	
}



vec3 Controller::proportionalFeedback(const double t, const State& current_state, const quat desired_q, const vec3 desired_w) {
	// for control strategy explanation, refer to "Quaternion Based Rate/Attitude Tracking System with Application to Gimbal Attitude Control" by H Weiss
	// the implementation here is exactly the same as that published in the paper

	// this control strategy will simultaneously attempt to drive the satellite to both a desired orientation and a desired angular velocity
	// the calling function must ensure that the provided desired_q and desired_w are consistent. If these arguments are not consistent with eachother, neither goal will be achieved
	// (e.g. if your desired_w is nonzero, you must vary desired_q appropriately as a function of time)


	// first, calculate control torque that would be necessary to exactly cancel the natural precession of the rotating body
	// i.e. cancel out the omega-cross-I-omega term
	// be very careful when working with glm vectors and matrices - you'd expect vec3 to be a column vector, but it is actually a row vector
	// note that: transpose(A*B) = transpose(B)*transpose(A)
	// therefore: transpose(MOI*(column vector omega)) = (row vector omega)*transpose(MOI) = (row vector omega)*MOI
	vec3 torque_precession = glm::cross(current_state.w, current_state.w * MOI);

	
	// second, calculate a feedback torque to drive the satellite angular velocity to the desired angular velocity

	// define the proportional gain for angular velocity correction
	// determined by trial and error - want to be as large as possible while still keeping control torque within reasonable bounds
	double GAIN_W = 0.05;

	vec3 error_w = current_state.w - desired_w;

	vec3 torque_w = -1.0 * GAIN_W * error_w * MOI;


	// third, calculate a feedback torque to drive the satellite orientation to the desired orientation

	// define the proportional gain for orientation correction
	// determined by trial and error - want to be as large as possible while still keeping control torque within reasonable bounds
	double GAIN_Q = 0.005;

	// there's probably a short-hand way to do this with the built-in glm quaternion functionality but I'll just write it out
	quat error_q;
	error_q.x =  desired_q.w*current_state.q.x + desired_q.z*current_state.q.y - desired_q.y*current_state.q.z - desired_q.x*current_state.q.w;
	error_q.y = -desired_q.z*current_state.q.x + desired_q.w*current_state.q.y + desired_q.x*current_state.q.z - desired_q.y*current_state.q.w;
	error_q.z =  desired_q.y*current_state.q.x - desired_q.x*current_state.q.y + desired_q.w*current_state.q.z - desired_q.z*current_state.q.w;
	error_q.w =  desired_q.x*current_state.q.x + desired_q.y*current_state.q.y + desired_q.z*current_state.q.z + desired_q.w*current_state.q.w;

	vec3 error_q_vecpart{ error_q.x, error_q.y, error_q.z };

	vec3 torque_q = -1.0 * GAIN_Q * error_q_vecpart * MOI;


	// sum the three torques to get the total control torque
	vec3 torque_total = torque_precession + torque_w + torque_q;

	// maximum achievable torque (in Newton-meters)
	double torque_max = 0.001;

	// check the magnitude of the total torque to ensure it is within sensible limits - display an error message if not
	if (glm::length(torque_total) > torque_max) {
		std::cout << std::endl << "Warning large control torque. Desired torque in Newton-meters:  " << glm::length(torque_total) << std::endl;
		std::cout << "Capping to " << torque_max << " Newton-meters. Try reducing control gains." << std::endl;
		torque_total = (torque_max / glm::length(torque_total)) * torque_total;
	}

	return torque_total;

	// note that a proportional controller (like this one) will not yield zero steady state error as time goes to infinity
	// to achieve zero steady state error as time goes to infinity, an integral term would be necessary
	// fortunately, the magnitude of all other torques on the satellite will be very small compared to that of the control actuators
	// and since the magnitude of all other torques is small, the final steady state error will be very small
}

