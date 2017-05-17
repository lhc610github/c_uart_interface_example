#ifndef LCM_INTERFACE_H_
#define LCM_INTERFACE_H_

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include <lcm/lcm-cpp.hpp>
#include "lcm_msg/uav_status_t.hpp"
using namespace std;
static string base_channel="uav_status_";

uint64_t lcm_get_time_usec();

void* start_lcm_interface_send_thread(void *args);

class Lcm_Interface
{

public:
	Lcm_Interface();
	~Lcm_Interface();
	
	uint64_t send_count;
	uint64_t pos_receive_time;
	uint64_t att_receive_time;
	uint64_t send_time;
	
	int mav_sys_id;
	

	string name_channel;
	uav_status::uav_status_t lcm_uav_status;
	
	bool has_init;

	int init(int mav_sys_id_);
	void send_lcm_messages();
	void receive_uav_pos(float x,float y,float z);
	void q_from_eular(float roll,float pitch,float yaw);
    void receive_uav_att(float roll,float pitch,float yaw);
	
	void start();
	void stop();
	
	void send_thread();
	void start_send_thread();
	

private:
	
	bool time_to_exit;
	float att_q_from_euler[4];// quaternion att from euler
	
	pthread_t send_tid;
	lcm::LCM lcm;
	
};
#endif
