#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include "DJI_guidance.h"
#include "DJI_utility.h"

using namespace std;
void test_using_data();
int WIDTH = 320;
int HEIGHT = 240;
#define IMAGE_SIZE (HEIGHT * WIDTH)

#define USE_GUIDANCE_ASSISTANT_CONFIG 0 //use GUIDANCE ASSISTANT's configure
#define SELECT_DEPTH_DATA 1

e_vbus_index sensor_id = e_vbus1;

DJI_lock    g_lock;
DJI_event   g_event;
char		key = 0;

position_info A;

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value) {
	const char* s = 0;
	static char str[100] = { 0 };
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch (value) {
		PROCESS_VAL(e_OK);
		PROCESS_VAL(e_load_libusb_err);
		PROCESS_VAL(e_sdk_not_inited);
		PROCESS_VAL(e_disparity_not_allowed);
		PROCESS_VAL(e_image_frequency_not_allowed);
		PROCESS_VAL(e_config_not_ready);
		PROCESS_VAL(e_online_flag_not_ready);
		PROCESS_VAL(e_stereo_cali_not_ready);
		PROCESS_VAL(e_libusb_io_err);
		PROCESS_VAL(e_timeout);
	default:
		strcpy(str, "Unknown error");
		s = str;
		break;
	}
#undef PROCESS_VAL

	return out << s;
}

int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_data* data = (image_data*)content;
		//	printf( "frame index:%d,stamp:%d\n", data->frame_index, data->time_stamp );

	}


	if (e_obstacle_distance == data_type && NULL != content)
	{
		obstacle_distance *oa = (obstacle_distance*)content;
		//	printf( "obstacle distance(m):" );
		for (int i = 0; i < CAMERA_PAIR_NUM; ++i)
		{
			//	printf(" %f ", 0.01f * oa->distance[i]);
			float a = 0.01f * oa->distance[i];
		/*	cout << "size" << endl;
			cout << sizeof(a) << endl;
			cout << sizeof(A.obstacle_p[1]) << endl;
*/
			memcpy(&A.obstacle_p[i], &a, sizeof(a));
		}


		//	printf( "\n" );
		//	printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );
	}

	if (e_ultrasonic == data_type && NULL != content)
	{
		ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
		for (int d = 0; d < CAMERA_PAIR_NUM; ++d)
		{
			//printf("ultrasonic distance(m):%f,reliability:%d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d]);
			float a = ultrasonic->ultrasonic[d] * 0.001f;
			memcpy(&A.ultrasonic_p[d], &a, sizeof(a));
		}
		//printf( "frame index:%d,stamp:%d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
	}

	if (e_motion == data_type && NULL != content) {
		motion* m = (motion*)content;
		//printf("(px,py,pz)=(%.2f,%.2f,%.2f)\n", m->position_in_global_x,m->position_in_global_y,m->position_in_global_z);
	}
	g_lock.leave();
	g_event.set_event();

	return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int obstacle()
{

	reset_config();  // clear all data subscription

					 //NEC

	int err_code = init_transfer(); //wait for board ready and init transfer thread 
	RETURN_IF_ERR(err_code);
	//show the situation of guidance
	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
	cout << "Sensor online status: ";
	for (int i = 0; i<CAMERA_PAIR_NUM; i++)
		cout << online_status[i] << " ";
	cout << endl;

	

	select_ultrasonic();
	select_obstacle_distance();

	select_motion();


	//start to show the value
	err_code = set_sdk_event_handler(my_callback);
	RETURN_IF_ERR(err_code);
	err_code = start_transfer();
	RETURN_IF_ERR(err_code);


	//wait for ten seconds
	sleep(1000000);



	g_event.wait_event();
	/*cout << "key ?" << endl;
	cin >> key;*/
	if (key > 0) {
		
		key = 0;
	}

	err_code = stop_transfer();
	RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
	sleep(1000000);
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);

	test_using_data();

	printf("safety clean");
	return 0;
}
void test_using_data()
{
	for (int i = 0; i < 5; i++)
		std::cout << "obstacle"<<A.obstacle_p[i] << endl;
	for (int i = 0; i < 5; i++)
		std::cout << "ultrasonic"<< A.ultrasonic_p[i] << endl;
}