/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
    uint64_t temp_unix_timestamp = (uint64_t)_time_stamp.tv_sec*1000000;
    temp_unix_timestamp += (uint64_t)_time_stamp.tv_usec;
	return temp_unix_timestamp;
}





// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode

	ap_status = 0;           // whether the avoid potential function is runing

	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					lcm_interface.receive_uav_pos(current_messages.local_position_ned.x,
					current_messages.local_position_ned.y,
					current_messages.local_position_ned.z);
					AP_interface.set_self_pos(current_messages.local_position_ned.x,
					current_messages.local_position_ned.y,
					current_messages.local_position_ned.z);
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					lcm_interface.receive_uav_att(current_messages.attitude.roll,
					current_messages.attitude.pitch,
					current_messages.attitude.yaw);
                    break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 100Hz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_ca_traject()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	//mavlink_ca_traject_t ca_traject;

	pthread_mutex_lock(&lcm_interface.l_u_t_handler.traject_pthread_lock);
    if (lcm_interface.l_u_t_handler.init_flage)
    {
        uint64_t _now = get_time_usec();
        //printf("now time is %lld \n ",_now);
        //printf("PC time is %lld \n ",lcm_interface.l_u_t_handler.PC_time);
        if (_now > lcm_interface.l_u_t_handler.PC_time - 5000000)
        {
            int _index = 0;
            bool _traject_vaild = false;
            double _delta_t = 0;
            if (_now <  lcm_interface.l_u_t_handler.PC_time)
            {
                _delta_t = 0;
                _traject_vaild = true;
                _index = 0;
            }
            else 
            {
                _delta_t = (double)(_now - lcm_interface.l_u_t_handler.PC_time)/1000000;
                for (int i = 1; i < lcm_interface.l_u_t_handler.num_keyframe+1 ; i++)
                {
                    if( _delta_t < lcm_interface.l_u_t_handler.t[i] )
                    { _index = i-1;
                     _traject_vaild = true;
                        break;
                    }
                    _traject_vaild = (i >= lcm_interface.l_u_t_handler.num_keyframe)?false:true;
                }
            }
            float _P_d[4];
            float _vel_d[4];
            float _acc_d[4];
            if (_traject_vaild)
            {
              /* caculate the P_d */
               for (int i = 0; i < 4 ;i++)
               {
                    int _order_p_1 = lcm_interface.l_u_t_handler.order_p_1;
                    double _temp_sum = 0;
                    for (int j = 0; j < _order_p_1; j++)
                    {
                        double _temp_poly_p_t = 1;
                        for (int k = 0; k < (_order_p_1 - 1 - j) ;k++)
                        {
                           _temp_poly_p_t *= _delta_t;
                        }
                        _temp_sum  += _temp_poly_p_t * lcm_interface.l_u_t_handler.traject[_index][j][i];
                    }
                    _P_d[i] = (float)_temp_sum;
               }

              /* caculate the vel_d */
               for (int i = 0; i < 4 ;i++)
               {
                    int _order_p_1 = lcm_interface.l_u_t_handler.order_p_1;
                    double _temp_sum = 0;
                    for (int j = 0; j < _order_p_1-1 ; j++)
                    {
                        double _temp_poly_p_t = 1;
                        for (int k = 0; k < (_order_p_1 - 2 - j) ;k++)
                        {
                           _temp_poly_p_t *= _delta_t;
                        }
                        _temp_sum  += _temp_poly_p_t * lcm_interface.l_u_t_handler.traject[_index][j][i] * (_order_p_1 -1 -j);
                    }
                    _vel_d[i] = (float)_temp_sum;
               }

              /* caculate the acc_d */
               for (int i = 0; i < 4 ;i++)
               {
                    int _order_p_1 = lcm_interface.l_u_t_handler.order_p_1;
                    double _temp_sum = 0;
                    for (int j = 0; j < _order_p_1-2 ; j++)
                    {
                        double _temp_poly_p_t = 1;
                        for (int k = 0; k < (_order_p_1 - 3 - j) ;k++)
                        {
                           _temp_poly_p_t *= _delta_t;
                        }
                        _temp_sum  += _temp_poly_p_t * lcm_interface.l_u_t_handler.traject[_index][j][i] * (_order_p_1 -1 -j) * (_order_p_1 -2 -j);
                    }
                    _acc_d[i] = (float)_temp_sum;
               }

              /* send the message */
               //printf("P_d : [ %.2f , %.2f , %.2f , %.2f ]\n",_P_d[0],_P_d[1],_P_d[2],_P_d[3]);
               //printf("v_d : [ %.2f , %.2f , %.2f , %.2f ]\n",_vel_d[0],_vel_d[1],_vel_d[2],_vel_d[3]);
               //printf("a_d : [ %.2f , %.2f , %.2f , %.2f ]\n",_acc_d[0],_acc_d[1],_acc_d[2],_acc_d[3]);
                mavlink_ca_traject_res_t ca_traject_res;
                ca_traject_res.PC_time_usec = (uint64_t) lcm_interface.l_u_t_handler.PC_time;
                ca_traject_res.time_usec = (uint64_t) get_time_usec();
                for (int i=0 ; i<4 ;i++)
                {
                    ca_traject_res.P_d[i] = _P_d[i];
                    ca_traject_res.vel_d[i] = _vel_d[i];
                    ca_traject_res.acc_d[i] = _acc_d[i];
                }
                // --------------------------------------------------------------------------
                //   ENCODE
                // --------------------------------------------------------------------------

                mavlink_message_t message;
                mavlink_msg_ca_traject_res_encode(system_id, companion_id, &message, &ca_traject_res);


                // --------------------------------------------------------------------------
                //   WRITE
                // --------------------------------------------------------------------------
                pthread_mutex_lock(&write_msg_pthread_lock);
                // do the write
                int len = write_message(message);

                // check the write
                if ( len <= 0 )
                    fprintf(stderr,"WARNING: could not send CA_TRAJECT \n");
                pthread_mutex_unlock(&write_msg_pthread_lock);
            }
        }
    }
	pthread_mutex_unlock(&lcm_interface.l_u_t_handler.traject_pthread_lock);
	return;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_ap_res(Ap_ctrl_out_s Ap_ctrl_out)
{
	mavlink_ap_uavs_func_res_t ap_func_res;
	ap_func_res.time_usec = (uint64_t) get_time_usec();
	ap_func_res.ap_ctrl_vel[0] = Ap_ctrl_out.vel_ctrl_output[0];
	ap_func_res.ap_ctrl_vel[1] = Ap_ctrl_out.vel_ctrl_output[1];
	ap_func_res.ap_ctrl_vel[2] = Ap_ctrl_out.vel_ctrl_output[2];
	mavlink_message_t message;
	mavlink_msg_ap_uavs_func_res_encode(system_id, companion_id, &message, &ap_func_res);
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    pthread_mutex_lock(&write_msg_pthread_lock);
    // do the write
    int len = write_message(message);
    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send AP_RES \n");
    pthread_mutex_unlock(&write_msg_pthread_lock);
}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
			if(!lcm_interface.has_init && system_id !=0 )
			{
				lcm_interface.init((int)system_id);
				lcm_interface.start();
			}
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// --------------------------------------------------------------------------
	//   AP FUNCTION THREAD
	// --------------------------------------------------------------------------
	result = pthread_create( &ap_tid, NULL, &start_avoid_potential_thread , this );
	if ( result ) throw result;

	while ( not ap_status )
		usleep(100000);


	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");
	
	lcm_interface.stop();
	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Avoid Potential Function Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_ap_thread()
{
	if ( not ap_status == false ) {
		fprintf(stderr, "avoid potential thread already running\n");
		return;
	} else {
		ap_thread();
		return;
	}
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}

void
Autopilot_Interface::
update_Ap_status()
{
	for (int i = 0; i < MAX_NUM_VEHICLE ; i++) {
		AP_interface.get_vehicle_status(i+1, get_Aps_from_lcm(i+1));
	}
}


Ap_vehicle_s
Autopilot_Interface::
get_Aps_from_lcm( int number)
{
	Ap_vehicle_s res;
	if ( number <= lcm_interface.max_num_quad ) {
		pthread_mutex_lock(&lcm_interface.l_s_handler[number-1].status_pthread_lock);
		if (!lcm_interface.l_s_handler[number-1].check_timeout() 
			&& lcm_interface.l_s_handler[number-1].init_flage) {
			res.vehicle_valid = true;
			res.last_get_timestamp = lcm_interface.l_s_handler[number-1].last_receive_time;
			for ( int i = 0; i < 3 ; i++) {
				res.pos[i] = lcm_interface.l_s_handler[number-1].oth_uav_status.position[i];
			}
			for ( int i = 0; i < 4 ; i++) {
				res.q[i] = lcm_interface.l_s_handler[number-1].oth_uav_status.orientation[i];
			}
		} else {
			res.reset_Ap_vehicle();
		}
		pthread_mutex_unlock(&lcm_interface.l_s_handler[number-1].status_pthread_lock);
	} else {
		printf("over max num \n");
		res.reset_Ap_vehicle();
	}
	return res;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	//mavlink_set_position_target_local_ned_t sp;
	//sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   //MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	//sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	//sp.vx       = 0.0;
	//sp.vy       = 0.0;
	//sp.vz       = 0.0;
	//sp.yaw_rate = 0.0;

	// set position target
	//current_setpoint = sp;

	// write a message and signal writing
	//write_setpoint();
    write_ca_traject();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
	    usleep(8000);   // Stream at 100Hz
        write_ca_traject();
	}

	// signal end
	writing_status = false;

	return;

}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
ap_thread(void)
{
	// signal startup
	ap_status = 2;


    // update the ap status
    float local_pos[3];
    update_Ap_status();
    // run ap funciton
    AP_interface.Avoid_Potential_run();
	ap_status = true;
	// run it period
	while ( !time_to_exit )
	{
	    // update the ap status
	    update_Ap_status();
	    // run ap funciton
	    // AP_interface.Avoid_Potential_run();
	    write_ap_res(AP_interface.Avoid_Potential_run());
	    usleep(80000);   // Stream at 10Hz
	}

	// signal end
	ap_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}


void*
start_avoid_potential_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_ap_thread();

	// done!
	return NULL;
}