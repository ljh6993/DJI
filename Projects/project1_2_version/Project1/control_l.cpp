// first change the include title
////need to change localoffset
//check height 
#include "control_l.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <DJI_guidance.h>
#include "DJI_utility.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
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

	test_using_data();

	return 0;
}
void end_guidance()
{

	g_event.wait_event();
	/*cout << "key ?" << endl;
	cin >> key;*/
	if (key > 0) {

		key = 0;
	}

	stop_transfer();
	//RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
	sleep(1000000);
	release_transfer();
	//RETURN_IF_ERR(err_code);
	printf("safety clean");
}

void test_using_data()
{
	for (int i = 0; i < 5; i++)
		std::cout << "obstacle" << A.obstacle_p[i] << endl;
	for (int i = 0; i < 5; i++)
		std::cout << "ultrasonic" << A.ultrasonic_p[i] << endl;
}




bool
monitoredTakeoff(Vehicle* vehicle, int timeout)
{
	//@todo: remove this once the getErrorCode function signature changes
	char func[50];
	int  pkgIndex;
	ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
	if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
	{
		ACK::getErrorCodeMessage(takeoffStatus, func);
		return false;
	}
	// First check: Motors started
	int motorsNotStarted = 0;
	int timeoutCycles = 20;
	// M100 // First check: Motors started
	while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) && motorsNotStarted < timeoutCycles)
	{
		// getstatus return status enum. stopped=0; on_groud =1. in air=2
		motorsNotStarted++;
		usleep(100000);
	}

	if (motorsNotStarted < timeoutCycles)
	{
		std::cout << "Successful TakeOff!" << std::endl;
	}
	// Second check: In air
	int stillOnGround = 0;
	timeoutCycles = 110;
	while ((vehicle->broadcast->getStatus().flight !=
		DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
		stillOnGround < timeoutCycles)
	{
		stillOnGround++;
		usleep(100000);
	}

	if (stillOnGround < timeoutCycles)
	{
		std::cout << "Aircraft in air!!" << std::endl;
	}
	// Final check: Finished takeoff

	float32_t                 delta;
	//Telemetry::GlobalPosition currentHeight;
	float currentheight = 0;
	float deltaheight = A.ultrasonic_p[0];
	//Telemetry::GlobalPosition deltaHeight =
	//	vehicle->broadcast->getGlobalPosition();
	// not working
	do
	{
		sleep(4);//??what should be us??
		currentheight = A.ultrasonic_p[0];
		delta = fabs(currentheight - deltaheight);
		deltaheight = currentheight;
	} while (delta >= 0.009);

	std::cout << "Aircraft hovering at " << A.ultrasonic_p[0] << "m!\n";
	return true;
}

// landing
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
	char func[50];
	int pkgIndex;
	
	ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
	if (ACK::getError(landingStatus) != ACK::SUCCESS)
	{
		ACK::getErrorCodeMessage(landingStatus, func);
		return false;
	}
	// First check: Landing started
	int landingNotStarted = 0;
	int timeoutCycles = 20;

	while (vehicle->broadcast->getStatus().flight !=
			DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
			landingNotStarted < timeoutCycles)
	{
			landingNotStarted++;
			usleep(100000);
	}
	
	if (landingNotStarted == timeoutCycles)
	{
		std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
		// Cleanup before return
		ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
		if (ACK::getError(ack))
		{
			std::cout << "Error unsubscribing; please restart the drone/FC to get "
				"back to a clean state.\n";
		}
		return false;
	}
	else
	{
		std::cout << "Landing...\n";
	}
	// M100

	while (vehicle->broadcast->getStatus().flight ==
		DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
	{
		sleep(1);
	}

	//may change to altitude
	double gp = 0;
	do
	{
		sleep(2);
		gp = A.ultrasonic_p[0];
		if (A.ultrasonic_r[0] = 0)
			gp = 2;

	} while (gp > 0.1);

	if (gp > 0.1)
	{
		std::cout
			<< "Landing finished, but the aircraft is in an unexpected mode. "
			"Please connect DJI GO.\n";
		return false;
	}
	else
	{
		std::cout << "Successful landing!\n";
	}
	return true;
}

//
bool
moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired, float yOffsetDesired, float zOffsetDesired, float yawDesired, float posThresholdInM, float yawThresholdInDeg)
{


	int responseTimeout = 1;
	int timeoutInMilSec = 10000;
	int controlFreqInHz = 50; // Hz
	int cycleTimeInMs = 1000 / controlFreqInHz;
	int pkgIndex;
	int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs; // 10 cycles
	int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles


	char func[50];
	// Wait for data to come in
	sleep(1); //not sure

	//current position and original position
	position_info current_position;
	position_info origin_position = A;
	// Convert position offset from first position to local coordinates

	Telemetry::Vector3f localOffset;
	localOffset.x = 0;
	localOffset.y = 0;
	localOffset.z = 0;

	// Get initial offset. We will update this in a loop later.
	double xOffsetRemaining = xOffsetDesired - localOffset.x;
	double yOffsetRemaining = yOffsetDesired - localOffset.y;
	double zOffsetRemaining = zOffsetDesired - (-localOffset.z);

	// Conversions
	double yawDesiredRad = DEG2RAD * yawDesired;
	double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

	Telemetry::Quaternion broadcastQ;
	double yawInRad;

	broadcastQ = vehicle->broadcast->getQuaternion();
	yawInRad = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;


	int   elapsedTimeInMs = 0;
	int   withinBoundsCounter = 0;
	int   outOfBounds = 0;
	int   brakeCounter = 0;
	int   speedFactor = 1;
	float xCmd, yCmd, zCmd;
	// There is a deadband in position control
	// the z cmd is absolute height
	// while x and y are in relative
	float zDeadband = 0.12;




	if (vehicle->isM100() || vehicle->isLegacyM600())
	{
		zDeadband = 0.12 * 10;
	}


	if (xOffsetDesired > 0)
		xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
	else if (xOffsetDesired < 0)
		xCmd =
		(xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
	else
		xCmd = 0;

	if (yOffsetDesired > 0)
		yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
	else if (yOffsetDesired < 0)
		yCmd =
		(yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
	else
		yCmd = 0;

	//Telemetry::GlobalPosition currentBroadcastGP;
	//currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
	zCmd = A.ultrasonic_p[0] + zOffsetDesired;// height is ultrasonic distance


													  //repeatly positionandyawctrl

	//positionandyawctrl fail
//	while (elapsedTimeInMs < timeoutInMilSec)
//	{
//		vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);
//
//		usleep(cycleTimeInMs * 1000);
//
//		elapsedTimeInMs += cycleTimeInMs;
//
//		broadcastQ = vehicle->broadcast->getQuaternion();
//		yawInRad = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
//		//set current
//		current_position = A;
//
//		//localOffsetFromOffset(vehicle, localOffset, &current_position, &origin_position);
//
//
//		//! See how much farther we have to go
//		xOffsetRemaining = current_position.obstacle_p[1]- xOffsetDesired;
//		//yOffsetRemaining = current_position.obstacle_p[2] -yOffsetDesired;
//		//zOffsetRemaining = zOffsetDesired - (-localOffset.z);
//		zOffsetRemaining = 0;
//	//std::cout << "XXXX emergencybrake" << std::endl;
//
//	//vehicle->control->emergencyBrake();
//		/*if (xOffsetRemaining < 0)
//			xOffsetRemaining = xOffsetDesired;
//		if (yOffsetRemaining < 0)
//			yOffsetRemaining = yOffsetDesired;*/
//		/*if (zOffsetRemaining < 0)
//			zOffsetRemaining = zOffsetDesired;*/
//	//! See if we need to modify the setpoint
//	if (std::abs(xOffsetRemaining) < speedFactor)
//	{
//		xCmd = xOffsetRemaining;
//	}
//	if (std::abs(yOffsetRemaining) < speedFactor)
//	{
//		yCmd = yOffsetRemaining;
//	}
//
//	if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
//		std::abs(yOffsetRemaining) < posThresholdInM &&
//		std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
//	{
//		//! 1. We are within bounds; start incrementing our in-bound counter
//		withinBoundsCounter += cycleTimeInMs;
//	}
//	else if (std::abs(xOffsetRemaining) < posThresholdInM &&
//		std::abs(yOffsetRemaining) < posThresholdInM &&
//		std::abs(zOffsetRemaining) < zDeadband &&
//		std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
//	{
//		//! 1. We are within bounds; start incrementing our in-bound counter
//		withinBoundsCounter += cycleTimeInMs;
//	}
//	else
//	{
//		if (withinBoundsCounter != 0)
//		{
//			//! 2. Start incrementing an out-of-bounds counter
//			outOfBounds += cycleTimeInMs;
//		}
//	}
//	//! 3. Reset withinBoundsCounter if necessary
//	if (outOfBounds > outOfControlBoundsTimeLimit)
//	{
//		withinBoundsCounter = 0;
//		outOfBounds = 0;
//	}
//	//! 4. If within bounds, set flag and break
//	if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
//	{
//		break;
//	}
//}
	//positionandyawctrl fail
	float distance = 3;
	
	do{
		distance = A.obstacle_p[1];
		vehicle->control->attitudeAndVertPosCtrl(0, -1, 0, zOffsetDesired);
		current_position = A;
		std::cout << "distancee" << distance << std::endl;
		usleep(cycleTimeInMs * 1000);

		elapsedTimeInMs += cycleTimeInMs;

		broadcastQ = vehicle->broadcast->getQuaternion();
		yawInRad = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
		//set current
		

		if (distance < xOffsetDesired)
		{
			std::cout << "break" << std::endl;
			break;
		}
		if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
			std::abs(yOffsetRemaining) < posThresholdInM &&
			std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
			//! 1. We are within bounds; start incrementing our in-bound counter
			//1000 cycletimeinms
			withinBoundsCounter += cycleTimeInMs;
		}
		else if (std::abs(xOffsetRemaining) < posThresholdInM &&
			std::abs(yOffsetRemaining) < posThresholdInM &&
			std::abs(zOffsetRemaining) < zDeadband &&
			std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
			//! 1. We are within bounds; start incrementing our in-bound counter
			withinBoundsCounter += cycleTimeInMs;
		}
		else
		{
			if (withinBoundsCounter != 0)
			{
				//! 2. Start incrementing an out-of-bounds counter
				outOfBounds += cycleTimeInMs;
			}
		}
		//! 3. Reset withinBoundsCounter if necessary
		//2. 10000
		if (outOfBounds > outOfControlBoundsTimeLimit)
		{
			withinBoundsCounter = 0;
			outOfBounds = 0;
		}
		//! 4. If within bounds, set flag and break
		if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
		{
			break;
		}
	} while (distance > xOffsetDesired);
	int x = 0;
	int y = 5;

	while (1)
	{
		vehicle->control->attitudeAndVertPosCtrl(0, -1, 0, zOffsetDesired);
		x++;
		std::cout << "stop" << std::endl;
		usleep(100000);
		if (x >= y)
		{
			break;
		}
		

	}
	
//	vehicle->control->positionAndYawCtrl(0, 0, 0, yawDesiredRad / DEG2RAD);


	if (elapsedTimeInMs >= timeoutInMilSec)
	{
		std::cout << "position movement Task timeout!\n";

		return ACK::FAIL;
	}

	return ACK::SUCCESS;


}


Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
	Telemetry::Vector3f    ans;
	Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

	double q2sqr = quaternion->q2 * quaternion->q2;
	double t0 = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
	double t1 =
		+2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
	double t2 =
		-2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
	double t3 =
		+2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
	double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;

	ans.x = asin(t2);
	ans.y = atan2(t3, t4);
	ans.z = atan2(t1, t0);

	return ans;
}


bool startGlobalPositionBroadcast(Vehicle* vehicle)
{
	uint8_t freq[16];

	/* Channels definition for A3/N3/M600
	* 0 - Timestamp
	* 1 - Attitude Quaternions
	* 2 - Acceleration
	* 3 - Velocity (Ground Frame)
	* 4 - Angular Velocity (Body Frame)
	* 5 - Position
	* 6 - GPS Detailed Information
	* 7 - RTK Detailed Information
	* 8 - Magnetometer
	* 9 - RC Channels Data
	* 10 - Gimbal Data
	* 11 - Flight Status
	* 12 - Battery Level
	* 13 - Control Information
	*/
	freq[0] = DataBroadcast::FREQ_HOLD;
	freq[1] = DataBroadcast::FREQ_HOLD;
	freq[2] = DataBroadcast::FREQ_HOLD;
	freq[3] = DataBroadcast::FREQ_HOLD;
	freq[4] = DataBroadcast::FREQ_HOLD;
	freq[5] = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
	freq[6] = DataBroadcast::FREQ_HOLD;
	freq[7] = DataBroadcast::FREQ_HOLD;
	freq[8] = DataBroadcast::FREQ_HOLD;
	freq[9] = DataBroadcast::FREQ_HOLD;
	freq[10] = DataBroadcast::FREQ_HOLD;
	freq[11] = DataBroadcast::FREQ_HOLD;
	freq[12] = DataBroadcast::FREQ_HOLD;
	freq[13] = DataBroadcast::FREQ_HOLD;

	ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
	if (ACK::getError(ack))
	{
		ACK::getErrorCodeMessage(ack, __func__);
		return false;
	}
	else
	{
		return true;
	}
}

//void localOffsetFromOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
//	position_info* target, position_info* origin)
//{
//		
//	
//		deltaNed.x = target->obstacle_p[1] - origin->obstacle_p[1];//forward
//		deltaNed.y = target->obstacle_p[2] - origin->obstacle_p[2];//forward
//		deltaNed.z = target->obstacle_p[0] - origin->obstacle_p[0];//forward
//		std::cout << "offsetdistacne" << deltaNed.x << "  " << deltaNed.y << "   " << deltaNed.z << std::endl;
//
//}

