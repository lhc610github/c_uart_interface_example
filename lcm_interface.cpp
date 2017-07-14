// --------------------------------------------------------------------------------------------------- 
// Includes 
// ---------------------------------------------------------------------------------------------------

#include "lcm_interface.h"
// --------------------------------------------------------------------------------------------------- 
// Time 
// --------------------------------------------------------------------------------------------------- 
uint64_t lcm_get_time_usec() {
    static struct timeval _time_stamp2;
    gettimeofday(&_time_stamp2, NULL);
    return _time_stamp2.tv_sec*1000000 + _time_stamp2.tv_usec;
}
// --------------------------------------------------------------------------------------------------- 
// LCM uav_status topic Handler 
// ---------------------------------------------------------------------------------------------------
 Lcm_u_s_Sub_Handler:: Lcm_u_s_Sub_Handler() { reset_mem();
}
Lcm_u_s_Sub_Handler:: ~Lcm_u_s_Sub_Handler() {}

void Lcm_u_s_Sub_Handler:: reset_mem() {

    init_flage = false;

    receive_time = 0; //us
    last_receive_time = 0; //us
    last_send_time = 0; //us
    last_send_count = 0; //us
    receive_rate = 0; //Hz
    send_rate =0; //Hz
}

void Lcm_u_s_Sub_Handler:: lcm_subscrib_function(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const uav_status::uav_status_t* msg) {
    //printf("Received message on channel \"%s\":\n", chan.c_str());
    //printf(" timestamp = %lld\n", msg->timestamp);
    //printf(" position = (%f, %f, %f)\n",
            //msg->position[0], msg->position[1], msg->position[2]);
    //printf(" orientation = (%f, %f, %f, %f)\n",
            //msg->orientation[0],msg->orientation[1],
            //msg->orientation[2],msg->orientation[3]);
    //printf(" mode = %d\n",msg->mode);
    //printf(" send_count = %lld\n",msg->send_count);
    oth_uav_status.timestamp = msg->timestamp;
    memcpy(oth_uav_status.position,msg->position,sizeof(msg->position));
    memcpy(oth_uav_status.orientation,msg->orientation,sizeof(msg->orientation));
    oth_uav_status.mode = msg->mode;
    oth_uav_status.send_count = msg->send_count;
    receive_time = lcm_get_time_usec();
    if (last_receive_time!=0)
    {
        init_flage = true;
        receive_rate = (1000000/(float)(receive_time-last_receive_time));
        send_rate = (1000000*(oth_uav_status.send_count-last_send_count))/(float)(oth_uav_status.timestamp-last_send_time);
    }
    else{
        receive_rate = 0;
        send_rate = 0;
    }
    last_receive_time = receive_time;
    last_send_time = oth_uav_status.timestamp;
    last_send_count = msg->send_count;
}

float Lcm_u_s_Sub_Handler:: get_send_rate() {
    return send_rate;
}

float Lcm_u_s_Sub_Handler:: get_receive_rate() {
    return receive_rate;
}

bool Lcm_u_s_Sub_Handler:: check_timeout() {
    if (lcm_get_time_usec()-last_receive_time > 1000000 && init_flage)
    {
        // receive time_out
        reset_mem();
        return true;
    }
    else
    {
        return false;
    }
}
// --------------------------------------------------------------------------------------------------- 
// LCM 'uav_traject' topic Handler 
// ---------------------------------------------------------------------------------------------------
Lcm_u_t_Sub_Handler:: Lcm_u_t_Sub_Handler() { reset_mem();
}
Lcm_u_t_Sub_Handler:: ~Lcm_u_t_Sub_Handler() {}

void Lcm_u_t_Sub_Handler:: reset_mem() {

    init_flage = false;

    receive_time = 0; //us
}

void Lcm_u_t_Sub_Handler:: lcm_u_t_subscrib_function(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const uav_traject::uav_traject_t* msg) {
    printf("Received message on channel \"%s\":\n", chan.c_str());
    printf(" timestamp = %lld\n", msg->timestamp);
    printf(" num_keyframe = %d\n", msg->num_keyframe);
    printf(" order+1 = %d\n", msg->order_p_1);
    printf(" t = [");
    for (int i=0; i< msg->num_keyframe;i++)
        printf("%.2f ",msg->t[i]);
    printf(" ]\n");
    printf("trajectory:\n");
    for (int i=0; i< msg->num_keyframe; i++)
    {
        printf("\tphase%d:\n",i);
        for (int k=0; k< 4; k++)
        {
        printf("\t\t");
            for (int j=0; j< msg->order_p_1; j++)
            {
                printf("%.2f ",msg->traject[i][j][k]);
            }
        printf("\n");
        }
    }
    printf(" traject print done \n");
    init_flage = true;
    receive_time = lcm_get_time_usec();
    PC_time = msg->timestamp;
    num_keyframe= msg->num_keyframe;
    order_p_1 = msg->order_p_1;
    for (int i=0; i< num_keyframe; i++)
        t[i] = msg->t[i];
    printf(" get traject.t \n");
    for (int i=0; i< num_keyframe; i++)
    {
        for (int k=0; k< 4; k++)
        {
            for (int j=0; j< order_p_1; j++)
            {
                traject[i][j][k] = msg->traject[i][j][k];
            }
        }
    }
    printf(" get traject.traject \n");
}
// --------------------------------------------------------------------------------------------------- 
// Lcm Interface Class 
// ---------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------- 
// Con/De structors 
// --------------------------------------------------------------------------------------------------- 
Lcm_Interface:: Lcm_Interface() {
    // initialize
    //std::string lcm_url = "udpm://239.255.76.67:7667?ttl=1";
    //lcm = lcm::LCM::LCM(lcm_url);
    send_count = 0;
    pos_receive_time = 0;
    att_receive_time = 0;
    
    time_to_exit = false;

    send_tid = 0; // send thread id

    mav_sys_id = 0;//mav_sys_id_;
    max_num_quad = 4;

    has_init = false;

}

Lcm_Interface:: ~Lcm_Interface() {}

// --------------------------------------------------------------------------------------------------- 
// Init 
// --------------------------------------------------------------------------------------------------- 
int Lcm_Interface:: init(int mav_sys_id_) {
    mav_sys_id = mav_sys_id_;
    max_num_quad = 4;
    stringstream ss;
    ss<<base_channel;
    ss<<status_channel;
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
// Send Messages 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: send_lcm_messages() {
    send_time = lcm_get_time_usec();
    if( pos_receive_time!=0 && ((pos_receive_time + 500000) > send_time)
           &&att_receive_time!=0 && ((att_receive_time + 500000) > send_time)
                && has_init)
    {
        send_count ++;
        lcm_uav_status.send_count = send_count;
	    lcm_uav_status.timestamp = send_time;
        /* */
        if (l_u_t_handler.init_flage) {
            lcm_uav_status.mode = 3;
        }
        else {
            lcm_uav_status.mode = 1;
        }
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
// Receive Pos
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: receive_uav_pos(float x,float y,float z) 
{
    pos_receive_time = lcm_get_time_usec();
    lcm_uav_status.position[0]=x;
    lcm_uav_status.position[1]=y;
    lcm_uav_status.position[2]=z;
}

// --------------------------------------------------------------------------------------------------- 
// Get q form eular 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: q_from_eular(float roll1,float pitch1,float yaw1) {
		float cosPhi_2 = cos(roll1 / 2.0);
		float sinPhi_2 = sin(roll1 / 2.0);
		float cosTheta_2 = cos(pitch1 / 2.0);
		float sinTheta_2 = sin(pitch1 / 2.0);
		float cosPsi_2 = cos(yaw1 / 2.0);
		float sinPsi_2 = sin(yaw1 / 2.0);

		/* operations executed in double to avoid loss of precision through
		 * consecutive multiplications. Result stored as float.
		 */
		att_q_from_euler[0] = (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
		att_q_from_euler[1] = (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
		att_q_from_euler[2] = (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
		att_q_from_euler[3] = (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}
// --------------------------------------------------------------------------------------------------- 
// Receive Att 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: receive_uav_att(float roll2,float pitch2,float yaw2) {
    att_receive_time = lcm_get_time_usec();
    //printf("ATT : %.2f %.2f %.2f \n",roll2,pitch2,yaw2);
    q_from_eular(roll2,pitch2,yaw2);
    lcm_uav_status.orientation[0]=att_q_from_euler[0];
    lcm_uav_status.orientation[1]=att_q_from_euler[1];
    lcm_uav_status.orientation[2]=att_q_from_euler[2];
    lcm_uav_status.orientation[3]=att_q_from_euler[3];
}

// --------------------------------------------------------------------------------------------------- 
// STARTUP 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: start() {
    int result;
    printf("START SEND THREAD \n");

    result = pthread_create( &send_tid, NULL, &start_lcm_interface_send_thread, this );
    result = pthread_create( &subscrib_tid, NULL, &start_lcm_subscribe_thread, this );
    result = pthread_create( &traject_subscrib_tid, NULL, &start_lcm_u_t_subscribe_thread, this );
    if ( result ) throw result;
}

// --------------------------------------------------------------------------------------------------- 
// SHUTDOWN 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: stop() {
    printf("CLOSE LCM THREAD\n");

    time_to_exit = true;

    pthread_join(send_tid ,NULL);
    pthread_join(subscrib_tid ,NULL);

    printf("\n");
}

// --------------------------------------------------------------------------------------------------- 
// Send Thread 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: send_thread() {
    while( ! time_to_exit )
    {
        send_lcm_messages();
        usleep(100000); // Send at 10Hz
    }

    return;
}
void Lcm_Interface:: start_send_thread() {
    send_thread();
    return;
}

// ---------------------------------------------------------------------------------------------------
// Subscrib Thread for 'uav_status' topic 
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: status_subscrib_thread() {
    stringstream ss1;
   printf("STATUS SUBSCRIBE THREAD START \n");
    for (int i = 0;i < max_num_quad; i++)
    {
        if( i != (mav_sys_id-1 ))
        {
           ss1.str("");
	   //ss1.clear();
           ss1<<base_channel;
           ss1<<status_channel;
           ss1<<(i+1);
	   l_s_handler[i].sub_name_channel = ss1.str();
           lcm.subscribe(l_s_handler[i].sub_name_channel, &Lcm_u_s_Sub_Handler::lcm_subscrib_function, &l_s_handler[i]);
           cout << "Subscrib Channel :" << ss1.str() << endl;
        }
    }
    while( ! time_to_exit )
    {
        lcm.handle();
    }

    return;
}

// --------------------------------------------------------------------------------------------------- 
// Subscrib Thread for 'uav_traject' topic
// --------------------------------------------------------------------------------------------------- 
void Lcm_Interface:: traject_subscrib_thread() {
    stringstream ss1;
   printf("TRAJECT SUBSCRIBE THREAD START \n");
           ss1.str("");
           ss1<<base_channel;
           ss1<<traject_channel;
           ss1<<mav_sys_id;
	       l_u_t_handler.sub_name_channel = ss1.str();
           lcm.subscribe(l_u_t_handler.sub_name_channel, &Lcm_u_t_Sub_Handler::lcm_u_t_subscrib_function, &l_u_t_handler);
           cout << "Subscrib Channel :" << ss1.str() << endl;
    while( ! time_to_exit )
    {
        lcm.handle();
    }

    return;
}
// --------------------------------------------------------------------------------------------------- 
// Pthread Starter Helper Functions 
// ---------------------------------------------------------------------------------------------------

void* start_lcm_interface_send_thread(void *args) {
    // takes an lcm object argument
    Lcm_Interface *lcm_interface = (Lcm_Interface *)args;
    // run the object's send thread
    lcm_interface->start_send_thread();

    // done!
    return NULL;
}
// --------------------------------------------------------------------------------------------------- 
// lcm subscrib Pthread Starter Helper Functions 
// ---------------------------------------------------------------------------------------------------

void* start_lcm_subscribe_thread(void *args) {
    // takes an lcm object argument
    Lcm_Interface *lcm_interface = (Lcm_Interface *)args;

    // run the object's subscrib thread
    lcm_interface->status_subscrib_thread();

    // done!
    return NULL;
}

// --------------------------------------------------------------------------------------------------- 
// lcm subscrib Pthread Starter Helper Functions for 'uav_traject' topic 
// ---------------------------------------------------------------------------------------------------

void* start_lcm_u_t_subscribe_thread(void *args) {
    // takes an lcm object argument
    Lcm_Interface *lcm_interface = (Lcm_Interface *)args;

    // run the object's subscrib thread
    lcm_interface->traject_subscrib_thread();

    // done!
    return NULL;
}
