#pragma once
#include <cmath>
class TDPA_Algorithm {
public:
	double sample_interval = 0.001;   //1kHz
	double E_in = 0, E_out = 0;
	double E_in_last = 0;
	double E_trans = 0, E_recv = 0;
	double alpha = 0;
	bool TDPAon = false;
	void ComputeEnergy(double vel[3], double force[3])
	{
		// only for z direction
		double power = vel[2] * (-1 * force[2]);
		if (power >= 0) {
			E_in = E_in + sample_interval*power;
		}
		else {
			E_out = E_out - sample_interval*power;
		}
	};
	// only used by master
	void ForceRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);

		if (E_out > E_recv && abs(Vel[2]) > 0.001)
		{
			alpha = (E_out - E_recv) / (sample_interval*Vel[2] * Vel[2]);
			E_out = E_recv;
		}
		else
			alpha = 0;

		// 3. revise force and apply the revised force
		if (TDPAon)
			force[2] = force[2] - alpha*Vel[2];
	};
	// only used by slavor
	void VelocityRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);
		if (E_out > E_recv && abs(force[2]) > 0.001)
		{
			alpha = (E_out - E_recv) / (sample_interval*force[2] * force[2]);
			E_out = E_recv;
		}
		else
			alpha = 0;

		// 3. revise slave vel 
		if (TDPAon)
			Vel[2] = Vel[2] - alpha*force[2];
	};

	void Initialize()
	{
		E_in = E_out = 0;
		E_in_last = 0;
		E_trans = E_recv = 0;
		alpha = 0;
	};
};

class ISS_Algorithm {
public:
	double mu_max = 10;
	float stiff_factor = 0.5;
	double d_force = 0.0;
	double tau = 0.005;
	bool ISS_enabled = false;
	double last_force = 0.0;
	float mu_factor = 1.7;

	void VelocityRevise(double* vel) {
		if (ISS_enabled) {
			vel[2] = vel[2] + d_force / (mu_max*mu_factor);
		}
	};

	void ForceRevise(double* force) {
		d_force = (force[2] - last_force) / 0.001;  // get derivation of force respect to time 
		last_force = force[2];
		force[2] = force[2] + d_force*tau;  // use "+" because MasterForce direction is opposite to f_e in the paper
	};

	void Initialize()
	{
		last_force = 0;
	};
};