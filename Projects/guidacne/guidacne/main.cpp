#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include "DJI_guidance.h"
#include "DJI_utility.h"

using namespace std;
void test_using_data();
int WIDTH=320;
int HEIGHT=240;
#define IMAGE_SIZE (HEIGHT * WIDTH)

#define USE_GUIDANCE_ASSISTANT_CONFIG 0 //use GUIDANCE ASSISTANT's configure
#define SELECT_DEPTH_DATA 1

e_vbus_index sensor_id = e_vbus1;

DJI_lock    g_lock;
DJI_event   g_event;
char		key = 0;

position_info A;





std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
	const char* s = 0;
	static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value){
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
		image_data* data = (image_data* )content;
     //	printf( "frame index:%d,stamp:%d\n", data->frame_index, data->time_stamp );

	}


	if ( e_obstacle_distance == data_type && NULL != content )
	{
		obstacle_distance *oa = (obstacle_distance*)content;
	//	printf( "obstacle distance(m):" );
		for (int i = 0; i < CAMERA_PAIR_NUM; ++i)
		{
		//	printf(" %f ", 0.01f * oa->distance[i]);
			float a = 0.01f * oa->distance[i];
			cout << "size" << endl;
			cout << sizeof(a) << endl;
			cout << sizeof(A.obstacle_p[1]) << endl;
			
			memcpy(& A.obstacle_p[i], &a, sizeof(a));
		}
			

	//	printf( "\n" );
	//	printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );
	}

	if ( e_ultrasonic == data_type && NULL != content )
	{
		ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
		for (int d = 0; d < CAMERA_PAIR_NUM; ++d)
		{
		printf("ultrasonic distance(m):%f,reliability:%d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d]);
			float a = ultrasonic->ultrasonic[d] * 0.001f;
			memcpy(&A.ultrasonic_p[d], &a, sizeof(a));
		}
		//printf( "frame index:%d,stamp:%d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
	}

	if(e_motion == data_type && NULL!=content){
		motion* m=(motion*)content;
		//printf("(px,py,pz)=(%.2f,%.2f,%.2f)\n", m->position_in_global_x,m->position_in_global_y,m->position_in_global_z);
	}
	g_lock.leave();
	g_event.set_event();

	return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int main(int argc, const char** argv)
{
	/*
	if(argc>1){
		printf("This is demo program showing data from Guidance.\n\t" 
			" 'a','d','w','s','x' to select sensor direction.\n\t"
			" 'j','k' to change the exposure parameters.\n\t"
			" 'm' to switch between AEC and constant exposure modes.\n\t"
			" 'n' to return to default exposure mode and parameters.\n\t"
			" 'q' to quit.");
		return 0;
	}*/
	//NEC
	reset_config();  // clear all data subscription

	//NEC

	int err_code = init_transfer(); //wait for board ready and init transfer thread 
	RETURN_IF_ERR( err_code );
	//show the situation of guidance
	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
	cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
		cout<<online_status[i]<<" ";
	cout<<endl;

	// get cali param
	//stereo_cali cali[CAMERA_PAIR_NUM];
	//err_code = get_stereo_cali(cali);
	//RETURN_IF_ERR(err_code);
	//cout<<"cu\tcv\tfocal\tbaseline\n";
	//for (int i=0; i<CAMERA_PAIR_NUM; i++)
	//{
	//	cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<endl;
	//}

//#if !USE_GUIDANCE_ASSISTANT_CONFIG
//	err_code = select_greyscale_image( sensor_id, true );
//	RETURN_IF_ERR( err_code );
//	err_code = select_greyscale_image( sensor_id, false );
//	RETURN_IF_ERR( err_code );
//#if SELECT_DEPTH_DATA
//	err_code = select_depth_image( sensor_id );
//	RETURN_IF_ERR( err_code );
//	err_code = select_disparity_image( sensor_id );
//	RETURN_IF_ERR( err_code );
//#endif

	select_ultrasonic();
	select_obstacle_distance();
	
	select_motion();
//#endif

	//e_device_type dt;
	//get_device_type(&dt);
	//cout<<"device type: "<<(dt==Guidance?"Guidance":"GuidanceLite")<<endl;

	//get_image_size(&WIDTH, &HEIGHT);
	//cout<<"(width, height)="<<WIDTH<<", "<<HEIGHT<<endl;
	//NEC set callback function handler

	//start to show the value
	err_code = set_sdk_event_handler( my_callback );
	RETURN_IF_ERR( err_code );
	err_code = start_transfer();
	RETURN_IF_ERR( err_code );


	//wait for ten seconds
	sleep(1000000);

	// for setting exposure
	//exposure_param para;
	//para.m_is_auto_exposure = 1;
	//para.m_step = 10;
	//para.m_expected_brightness = 120;
	//para.m_camera_pair_index = sensor_id;


		g_event.wait_event();
		/*cout << "key ?" << endl;
		cin >> key;*/
		if (key > 0) {
			// set exposure parameters
			//if (key == 'j' || key == 'k' || key == 'm' || key == 'n') {
			//	if (key == 'j') {
			//		if (para.m_is_auto_exposure) para.m_expected_brightness += 20;
			//		else para.m_exposure_time += 3;
			//	}
			//	else if (key == 'k') {
			//		if (para.m_is_auto_exposure) para.m_expected_brightness -= 20;
			//		else para.m_exposure_time -= 3;
			//	}
			//	else if (key == 'm') {
			//		para.m_is_auto_exposure = !para.m_is_auto_exposure;
			//		cout << "exposure is " << para.m_is_auto_exposure << endl;
			//	}
			//	else if (key == 'n') {//return to default
			//		para.m_expected_brightness = para.m_exposure_time = 0;
			//	}

				//cout << "Setting exposure parameters....SensorId=" << sensor_id << endl;
				//para.m_camera_pair_index = sensor_id;
				//set_exposure_param(&para);
				key = 0;
			}
		/*if (key == 'q' || key == 'w' || key == 'd' || key == 'x' || key == 'a' || key == 's') 
		{*/
			
			// switch image direction
		    
		/*
			if (key == 'q')
				break;*/
		
//			/*	if (key == 'w') sensor_id = e_vbus1;
//				if (key == 'd') sensor_id = e_vbus2;
//				if (key == 'x') sensor_id = e_vbus3;
//				if (key == 'a') sensor_id = e_vbus4;
//				if (key == 's') sensor_id = e_vbus5;
//*/
//			/*	select_greyscale_image(sensor_id, true);
//				select_greyscale_image(sensor_id, false);*/
//				//#if SELECT_DEPTH_DATA
//				//				select_depth_image(sensor_id);
//				//				select_disparity_image(sensor_id);
//				//#endif
//				err_code = start_transfer();
//				RETURN_IF_ERR(err_code);
//				key = 0;
//			}
		
	
	
	
	err_code = stop_transfer();
	RETURN_IF_ERR( err_code );
	//make sure the ack packet from GUIDANCE is received
	sleep( 1000000 );
	err_code = release_transfer();
	RETURN_IF_ERR( err_code );

	test_using_data();

	printf("safety clean");
	return 0;
}
void test_using_data()
{
	for (int i = 0; i < 5; i++)
		std::cout << A.obstacle_p[i]<< endl;
	for (int i = 0; i < 5; i++)
		std::cout << A.ultrasonic_p[i]<< endl;
}