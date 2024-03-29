#ifndef BRAITENBERGVEHICLE2CONTROLLER_H_
#define BRAITENBERGVEHICLE2CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CBraitenbergVehicle2Controller : public CController
{
public:

    CBraitenbergVehicle2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CBraitenbergVehicle2Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CBlueLightSensor* m_seBlueLight;
		CRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  

		int m_nWriteToFile;
};

#endif
