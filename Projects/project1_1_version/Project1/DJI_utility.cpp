
#include "DJI_utility.h"



position_info::position_info()
{
	this->motion_x = 0;
	this->motion_y = 0;
	this->motion_z = 0;
	for (int i = 0; i < 5; i++)
	{
		this->obstacle_p[i];
		this->ultrasonic_p[i];
		this->ultrasonic_r[i];
	}

}
DJI_lock::DJI_lock()
{
	pthread_mutex_init( &m_lock, NULL );
}

DJI_lock::~DJI_lock()
{
}

void DJI_lock::enter()
{
	pthread_mutex_lock( &m_lock );
}

void DJI_lock::leave()
{
	pthread_mutex_unlock( &m_lock );
}

DJI_event::DJI_event()
{
	sem_init( &m_sem, 0, 0 );
}

DJI_event::~DJI_event()
{
}

int DJI_event::set_event()
{
	int ret = sem_post( &m_sem );
	return ret;
}

int DJI_event::wait_event()
{
	int ret = sem_wait( &m_sem );
	return ret;
}

