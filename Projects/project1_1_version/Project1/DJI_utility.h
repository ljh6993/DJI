#ifndef __DJI_UTILITY_H__
#define __DJI_UTILITY_H__
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>



const int camera = 5;


class DJI_lock
{
public:
	DJI_lock();
	~DJI_lock();
	void         enter();
	void         leave();
private:
	pthread_mutex_t m_lock;
};

class DJI_event
{
public:
	DJI_event();
	~DJI_event();
	int         set_event();
	int         wait_event();
private:
	sem_t		m_sem;
};

void   sleep(int microsecond);

class position_info
{
private:


public:
	position_info();

	//for obstacle_distance
	float obstacle_p[camera];
	//float obstacle_right;
	//float obstacle_rear;
	//float obstacle_left;
	//float obstacle_down;


	//ultrasonic distance
	float ultrasonic_p[camera];
	int ultrasonic_r[camera];

	//motion_position
	float motion_x;
	float motion_y;
	float motion_z;

};
#endif
