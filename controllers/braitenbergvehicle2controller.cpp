/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "bluelightsensor.h"
#include "redlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "braitenbergvehicle2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CBraitenbergVehicle2Controller::CBraitenbergVehicle2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
}

/******************************************************************************/
/******************************************************************************/

CBraitenbergVehicle2Controller::~CBraitenbergVehicle2Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CBraitenbergVehicle2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

    int minSpeed=-700;
    int maxSpeed=-900;

    double* light = m_seLight->GetSensorReading(m_pcEpuck);
    printf("Light Sensor Value: ");
    double sum=0;
    for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i++)
    {
        printf("%2f ", light[i]);
        sum+=light[i];
    }
    printf("\n");

    double speed[] = {(minSpeed+(maxSpeed-minSpeed)*(1-light[3]/sum)),(minSpeed+(maxSpeed-minSpeed)*(light[4]/sum))};
    //Infinity check. This is equivalent to clamping the transfer functions, but more optimized programatically.
    if(sum==0){
        speed[0]=maxSpeed;
        speed[1]=minSpeed;
    }
    m_acWheels->SetSpeed(speed[0],speed[1]);
    printf("speed %2f, %2f ",speed[0],speed[1]);
  
}

/******************************************************************************/
/******************************************************************************/

