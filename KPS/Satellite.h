/*******************************************************************
*   Satellite.h
*   KPS
*
*	Author: Kareem Omar
*	kareem.omar@uah.edu
*	https://github.com/komrad36
*
*	Last updated Feb 27, 2016
*   This application is entirely my own work.
*******************************************************************/
//
// Satellite model, including satellite orbital and attitude ODE.
//

#pragma once

#include "Earth.h"
#include "Aero.h"
#include "State.h"
#include "US1976.h"
#include "glm_util.h"
#include "Eigen_util.h"
#include "Controller.h"

class Satellite {
public:
	State state;
	dEvec13& e_state;

	const Earth& earth;
	const double m;

	double aer_force_mag;

	Controller attitude_controller;

private:

	const double mag_gain;

	const double time_since_epoch_at_deploy;

	const mat3 MOI;
	const mat3 inv_MOI;

	Aero& aero;

public:
	Satellite(const Earth& earth_model, Aero& aero_model, const double sat_mass, const double magnetorquer_gain, const double seconds_since_epoch_at_deploy, const State& initial_state, const mat3 sat_MOI) : state(initial_state), e_state(s_to_e(state)), earth(earth_model), m(sat_mass), attitude_controller(sat_MOI), mag_gain(magnetorquer_gain), time_since_epoch_at_deploy(seconds_since_epoch_at_deploy), MOI(sat_MOI), inv_MOI(glm::inverse(MOI)), aero(aero_model) {}

	bool isInValidState() const;

	dEvec13 ode(const double t, const dEvec13& ode_e_state);

private:

	vec3 getMagFieldInBodyFrame(const double t, const State& ode_state) const;

};


