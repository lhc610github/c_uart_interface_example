#ifndef AVOID_POTENTIAL_INTERFACE_H_
#define AVOID_POTENTIAL_INTERFACE_H_

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#define AVOID_POTENTIAL_TIMEOUT 1000000 //1s
#define MAX_NUM_VEHICLE 4
#define PERCEIVED_RADIUS 0.8
#define DIMENSION 3
#define CONFLICT_PARAM_PA 0.5
#define CONFLICT_PARAM_RA 0.8
#define CONFLICT_PARAM_NJ 1.0
#define CONFLICT_PARAM_EA 0.25

uint64_t ap_get_time_usec();

struct Ap_vehicle_s
{
	Ap_vehicle_s()
	{
		reset_Ap_vehicle();
	}

	bool vehicle_valid;
	float pos[DIMENSION]; //NED
	float q[4];
	uint64_t last_get_timestamp;

	void
	reset_Ap_vehicle()
	{
		vehicle_valid = false;
		memset(pos, 0.0f, sizeof(float)*DIMENSION);
		memset(q, 0.0f, sizeof(float)*4);
		last_get_timestamp = 0;
	}

};

struct Ap_ctrl_out_s
{
	Ap_ctrl_out_s()
	{
		reset_Ap_ctrl_out();
	}

	bool valid;
	float vel_ctrl_output[DIMENSION]; //NED velcity controll value
	uint64_t timestamp;

	void
	reset_Ap_ctrl_out()
	{
		valid = false;
		memset(vel_ctrl_output, 0.0f, sizeof(float)*DIMENSION);
		timestamp = 0;
	}
};

struct Ap_ctrl_func_s
{
	Ap_ctrl_func_s()
	{
		reset_Ap_ctrl_func();
	}

	bool valid;
	float ctrl_output[DIMENSION];

	void
	reset_Ap_ctrl_func()
	{
		valid = false;
		memset(ctrl_output, 0.0f, sizeof(float)*DIMENSION);
	}
};

class Avoid_Potential_Interface
{
public:
	Avoid_Potential_Interface();
	~Avoid_Potential_Interface();
	void set_vehicle_valid(int num_vehicle); // update the status of vehicle_valid
	bool check_vehicle_status_timeout(int num_vehicle);
	void get_vehicle_status(int num_vehicle, Ap_vehicle_s vehicle_status);
	Ap_ctrl_out_s Avoid_Potential_run();
	void set_self_pos(float x, float y, float z);
	void info();
private:
	// int max_num_vehicle = MAX_NUM_VEHICLE;
	
	float do_norm(float temp_vector[DIMENSION]) {
		float temp = 0.001;
		for (int i = 0; i < DIMENSION; i++) {
			temp = temp + temp_vector[i]*temp_vector[i];
		}
		return sqrt(temp);
	};

	Ap_ctrl_out_s do_add_ctrl_res(Ap_ctrl_out_s last_res, Ap_ctrl_func_s func_res) {
		Ap_ctrl_out_s res;
		for (int i = 0; i < DIMENSION; i++ ) {
			res.vel_ctrl_output[i] = last_res.vel_ctrl_output[i] + func_res.ctrl_output[i];
			res.valid = false;
		}
		return res;
	};
	float update_rate;
	float self_pos[DIMENSION];
	uint64_t self_pos_timestamp;
	Ap_ctrl_out_s last_ctrl_output;
	Ap_vehicle_s around_vehicle[MAX_NUM_VEHICLE];
	Ap_ctrl_func_s Avoid_Potential_function(float other_pos[DIMENSION], float self_pos[DIMENSION]);

	float c_param_pa;
	float c_param_Ra;
	float c_param_nj;
	float c_param_ea;

};

#endif
