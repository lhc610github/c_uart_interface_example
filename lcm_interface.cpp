// ---------------------------------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------------------------------

#include "lcm_interface.h"
// ---------------------------------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------------------------------
uint64_t
lcm_get_time_usec()
{
    static struct timeval _time_stamp2;
    gettimeofday(&_time_stamp2, NULL);
    return _time_stamp2.tv_sec*1000000 + _time_stamp2.tv_usec;
}

// ---------------------------------------------------------------------------------------------------
// Lcm Interface Class 
// ---------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------
//  Con/De structors 
// ---------------------------------------------------------------------------------------------------
Lcm_Interface::
Lcm_Interface()
{
    // initialize
    send_count = 0;
    pos_receive_time = 0;
    att_receive_time = 0;
    
    time_to_exit = false;

    send_tid = 0; // send thread id

    mav_sys_id = 0;//mav_sys_id_;

    has_init = false;

}

Lcm_Interface::
~Lcm_Interface()
{}

// ---------------------------------------------------------------------------------------------------
//  Init 
// ---------------------------------------------------------------------------------------------------
int
Lcm_Interface::
init(int mav_sys_id_)
{
    mav_sys_id = mav_sys_id_;
    stringstream ss;
    ss<<base_channel;
    ss<<mav_sys_id;
    name_channel = ss.str();
    if (!lcm.good())
    {
        has_init = false;
        return 0;
    }
    else
    {
        printf("Lcm_Interface has initialized");
        printf("Lcm channel is: `%s` \n",name_channel.c_str());
        has_init = true;
        return 1;
    }
}

// ---------------------------------------------------------------------------------------------------
//  Send Messages 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
send_lcm_messages()
{
    send_time = lcm_get_time_usec();
    if( pos_receive_time!=0 && ((pos_receive_time + 500000) > send_time) 
           &&att_receive_time!=0 && ((att_receive_time + 500000) > send_time) 
                && has_init)
    {
        send_count ++;
        lcm_uav_status.send_count = send_count;
        lcm.publish(name_channel, &lcm_uav_status);
    }else{
                if (pos_receive_time!=0 && ((pos_receive_time + 500000) > send_time)) {
                printf("POS TIME OUT : about %f ms at %f \n",(send_time-pos_receive_time)*0.001,send_time*0.000001);
                }
                if (att_receive_time!=0 && ((att_receive_time + 500000) > send_time)) {
                printf("ATT TIME OUT : about %f ms at %f \n",(send_time-att_receive_time)*0.001,send_time*0.000001);
                }
          }
}

// ---------------------------------------------------------------------------------------------------
//  Receive Pos 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
receive_uav_pos(float x,float y,float z)
{
    pos_receive_time = lcm_get_time_usec();
    lcm_uav_status.position[0]=x;
    lcm_uav_status.position[1]=y;
    lcm_uav_status.position[2]=z;
}

// ---------------------------------------------------------------------------------------------------
//  Get q form eular
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
q_from_eular(float roll,float pitch,float yaw)
{
		float cosPhi_2 = cos(roll / 2.0);
		float sinPhi_2 = sin(roll / 2.0);
		float cosTheta_2 = cos(pitch / 2.0);
		float sinTheta_2 = sin(pitch / 2.0);
		float cosPsi_2 = cos(yaw / 2.0);
		float sinPsi_2 = sin(yaw / 2.0);

		/* operations executed in double to avoid loss of precision through
		 * consecutive multiplications. Result stored as float.
		 */
		att_q_from_euler[0] = (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
		att_q_from_euler[1] = (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
		att_q_from_euler[2] = (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
		att_q_from_euler[3] = (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}
// ---------------------------------------------------------------------------------------------------
//  Receive Att 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
receive_uav_att(float roll,float pitch,float yaw)
{
    att_receive_time = lcm_get_time_usec();
    printf("ATT : %.2f %.2f %.2f",roll,pitch,yaw);
    q_from_eular(roll,pitch,yaw);
    lcm_uav_status.orientation[0]=att_q_from_euler[0];
    lcm_uav_status.orientation[1]=att_q_from_euler[1];
    lcm_uav_status.orientation[2]=att_q_from_euler[2];
    lcm_uav_status.orientation[3]=att_q_from_euler[3];
}

// ---------------------------------------------------------------------------------------------------
//  STARTUP 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
start()
{
    int result;
    printf("START SEND THREAD \n");

    result = pthread_create( &send_tid, NULL, &start_lcm_interface_send_thread, this );
    if ( result ) throw result;
}

// ---------------------------------------------------------------------------------------------------
//  SHUTDOWN 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
stop()
{
    printf("CLOSE LCM THREAD\n");

    time_to_exit = true;

    pthread_join(send_tid ,NULL);

    printf("\n");
}

// ---------------------------------------------------------------------------------------------------
//  Send Thread 
// ---------------------------------------------------------------------------------------------------
void
Lcm_Interface::
send_thread()
{
    while( ! time_to_exit )
    {
        send_lcm_messages();
        usleep(100000); // Send at 10Hz
    }

    return;
}
void
Lcm_Interface::
start_send_thread()
{
    send_thread();
    return;
}

// ---------------------------------------------------------------------------------------------------
//  Pthread Starter Helper Functions 
// ---------------------------------------------------------------------------------------------------

void*
start_lcm_interface_send_thread(void *args)
{
    // takes an lcm object argument
    Lcm_Interface *lcm_interface = (Lcm_Interface *)args;

    // run the object's send thread
    lcm_interface->start_send_thread();

    // done!
    return NULL;
}
