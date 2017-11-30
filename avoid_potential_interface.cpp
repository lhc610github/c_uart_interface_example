// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "avoid_potential_interface.h"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
ap_get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
    uint64_t temp_unix_timestamp = (uint64_t)_time_stamp.tv_sec*1000000;
    temp_unix_timestamp += (uint64_t)_time_stamp.tv_usec;
	return temp_unix_timestamp;
}

Avoid_Potential_Interface::
Avoid_Potential_Interface()
{
	// memset(around_vehicle, 0, (sizeof(Ap_vehicle_s)*MAX_NUM_VEHICLE));
}

Avoid_Potential_Interface::
~Avoid_Potential_Interface()
{}

void
Avoid_Potential_Interface::
set_vehicle_valid( int num_vehicle)
{	
	if (num_vehicle <= MAX_NUM_VEHICLE)
		around_vehicle[num_vehicle-1].vehicle_valid = true;
	else
		printf("over the max num of vehicle\n");
}

bool
Avoid_Potential_Interface::
check_vehicle_status_timeout( int num_vehicle)
{
	if (num_vehicle <= MAX_NUM_VEHICLE && num_vehicle > 0) {
		around_vehicle[num_vehicle-1].vehicle_valid = true;
		uint64_t now = ap_get_time_usec();
		if (around_vehicle[num_vehicle-1].vehicle_valid 
			&& (now - around_vehicle[num_vehicle-1].last_get_timestamp) < AVOID_POTENTIAL_TIMEOUT) {
			return false;
		} else {
			printf("around_vehicle[%d] status timeout or not valid\n",num_vehicle);
			return true;
		}
	} else if(num_vehicle == 0) {
		uint64_t now = ap_get_time_usec();
		if (now - self_pos_timestamp  < AVOID_POTENTIAL_TIMEOUT)
			return false;
	} else {
		printf("over the max num of vehicle\n");
		return true;
	}
}

void
Avoid_Potential_Interface::
get_vehicle_status(int num_vehicle, Ap_vehicle_s vehicle_status)
{
	if (num_vehicle <= MAX_NUM_VEHICLE)
		around_vehicle[num_vehicle-1] = vehicle_status;
	else
		printf("over the max num of vehicle\n");
}

Ap_ctrl_out_s
Avoid_Potential_Interface::
Avoid_Potential_run()
{
	Ap_ctrl_out_s res;
	bool ctl_valid_flag = false;
	for (int i = 1; i <= MAX_NUM_VEHICLE; i++) {
		if (!check_vehicle_status_timeout(i) && !check_vehicle_status_timeout(0)) {
			ctl_valid_flag = true;
			Ap_ctrl_func_s temp_out;
			temp_out = Avoid_Potential_function(around_vehicle[i-1].pos, self_pos);		
			if (temp_out.valid) {
				res = do_add_ctrl_res(res, temp_out);
			}
		}
	}

	if (ctl_valid_flag) {
		res.valid = true;
		res.timestamp = ap_get_time_usec();
		last_ctrl_output = res;
		return res;
	}

	res.reset_Ap_ctrl_out();
	last_ctrl_output = res;
	return res;
}

void
Avoid_Potential_Interface::
set_self_pos(float x, float y, float z)
{
	self_pos[0] = x; //N
	self_pos[1] = y; //E
	self_pos[2] = z; //D
	self_pos_timestamp = ap_get_time_usec();
}

Ap_ctrl_func_s
Avoid_Potential_Interface::
Avoid_Potential_function(float other_pos[DIMENSION], float self_pos[DIMENSION])
{			
	float delta_pos[DIMENSION];
	Ap_ctrl_func_s res;
	for (int dim_i = 0; dim_i < DIMENSION; dim_i++) {
		delta_pos[dim_i] = other_pos[dim_i] - self_pos[dim_i];
	}

	if (do_norm(delta_pos) < PERCEIVED_RADIUS) {
		// TODO: add AP function
		for (int dim_i = 0; dim_i < DIMENSION; dim_i++) {
			res.ctrl_output[dim_i] = delta_pos[dim_i];
		}
		res.valid = true;
		return res;
	} 
	res.valid = false;
	return res;
}

void
Avoid_Potential_Interface::
info()
{
	printf("* Avoid Potential function info: \n");

	if (!check_vehicle_status_timeout(0))
		printf("  self pos: [ %2.3f, %2.3f, %2.3f]\n", self_pos[0], self_pos[1], self_pos[2]);
	else
		printf("  self pos is not validable\n");

	for (int i = 0; i < MAX_NUM_VEHICLE; i++) {
		if (!check_vehicle_status_timeout(i+1))
			printf("  uav[%d]: [ %2.3f, %2.3f, %2.3f]\n",i+1,around_vehicle[i].pos[0], around_vehicle[i].pos[1], around_vehicle[i].pos[2]);
		else
			printf("  uav[%d] is not validable\n",i+1);
	}

	if (last_ctrl_output.valid)
		printf("  ctrl output: [ %2.3f, %2.3f, %2.3f]\n", last_ctrl_output.vel_ctrl_output[0], last_ctrl_output.vel_ctrl_output[1], last_ctrl_output.vel_ctrl_output[2]);
}