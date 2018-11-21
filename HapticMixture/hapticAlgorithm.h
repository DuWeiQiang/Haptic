#pragma once
#include <cmath>
class TDPA_Algorithm {
public:
	double sample_interval = 0.001;   //1kHz
	double E_in[3] = {0,0,0}, E_out[3] = { 0,0,0 };
	double E_in_last[3] = { 0,0,0 };
	double E_trans[3] = { 0,0,0 }, E_recv[3] = { 0,0,0 };
	double alpha[3] = { 0,0,0 }, beta[3] = { 0,0,0 };
	bool TDPAon = false;
	void ComputeEnergy(double vel[3], double force[3])
	{
		for (int i = 0; i < 3; i++) {
			// only for z direction
			double power = vel[i] * (-1 * force[i]);
			if (power >= 0) {
				E_in[i] = E_in[i] + sample_interval*power;
			}
			else {
				E_out[i] = E_out[i] - sample_interval*power;
			}
		}
		
	};
	// only used by master
	void ForceRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);
		for (int i = 0; i < 3; i++) {
			if (E_out[i] > E_recv[i] && abs(Vel[i]) > 0.001)
			{
				alpha[i] = (E_out[i] - E_recv[i]) / (sample_interval*Vel[i] * Vel[i]);
				E_out[i] = E_recv[i];
			}
			else
				alpha[i] = 0;

			// 3. revise force and apply the revised force
			if (TDPAon)
				force[i] = force[i] - alpha[i] *Vel[i];
		}
	};
	// only used by slavor
	void VelocityRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);
		for (int i = 0; i < 3; i++) {
			if (E_out[i] > E_recv[i] && abs(force[i]) > 0.001)
			{
				beta[i] = (E_out[i] - E_recv[i]) / (sample_interval*force[i] * force[i]);
				E_out[i] = E_recv[i];
			}
			else
				beta[i] = 0;

			// 3. revise slave vel 
			if (TDPAon)
				Vel[i] = Vel[i] - beta[i] * force[i];
		}
	};

	void Initialize()
	{
		memset(E_in, 0, 3 * sizeof(double));
		memset(E_out, 0, 3 * sizeof(double));
		memset(E_in_last, 0, 3 * sizeof(double));
		memset(E_trans, 0, 3 * sizeof(double));
		memset(E_recv, 0, 3 * sizeof(double));
		memset(alpha, 0, 3 * sizeof(double));
		memset(beta, 0, 3 * sizeof(double));
	};
};

class ISS_Algorithm {
public:
	double mu_max = 10;
	float stiff_factor = 0.5;
	double d_force[3] = { 0,0,0 };
	double tau = 0.005;
	bool ISS_enabled = false;
	double last_force[3] = { 0,0,0 };
	float mu_factor = 1.7;

	void VelocityRevise(double* vel) {
		if (ISS_enabled) {
			for (int i = 0; i < 3; i++) {
				vel[i] = vel[i] + d_force[i] / (mu_max*mu_factor);
			}
			
		}
	};

	void ForceRevise(double* force) {
		for (int i = 0; i < 3; i++) {
			d_force[i] = (force[i] - last_force[i]) / 0.001;  // get derivation of force respect to time 
			last_force[i] = force[i];
			force[i] = force[i] + d_force[i] * tau;  // use "+" because MasterForce direction is opposite to f_e in the paper
		}
		
	};

	void Initialize()
	{
		memset(last_force, 0, 3*sizeof(double));
	};
};

class MMT_algorithm {
public:

};