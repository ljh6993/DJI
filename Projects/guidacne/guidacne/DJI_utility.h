#ifndef __DJI_UTILITY_H__
#define __DJI_UTILITY_H__
#include <stdio.h>

#ifdef WIN32

#include <Windows.h>
#include <WinBase.h>

class DJI_lock
{
public:
	DJI_lock();
	~DJI_lock();
	void         enter();
	void         leave();
private:
	CRITICAL_SECTION  m_critical_section;
};

class DJI_event
{
public:
	DJI_event();
	~DJI_event();
	int         set_event();
	int         wait_event();
private:
	HANDLE      m_pipe_read;
	HANDLE      m_pipe_write;
};

#else

#include <pthread.h>
#include <semaphore.h>
#define CAMERA_PAIR_NUM 5
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

#endif

void   sleep( int microsecond );


#endif
class position_info
{
private:


public:
	position_info();

	//for obstacle_distance
	float obstacle_p[CAMERA_PAIR_NUM];
	//float obstacle_right;
	//float obstacle_rear;
	//float obstacle_left;
	//float obstacle_down;


	//ultrasonic distance
	float ultrasonic_p[CAMERA_PAIR_NUM];
	int ultrasonic_r[CAMERA_PAIR_NUM];

	//motion_position
	float motion_x;
	float motion_y;
	float motion_z;

};