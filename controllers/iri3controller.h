#ifndef IRI3CONTROLLER_H_
#define IRI3CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri3Controller : public CController
{
public:

    CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri3Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
    void setRobotIndex(int index);
    void setRobotAmount(int amount);
    void setAssignedLights(int* assignedLights);
    void setCollectionBoard(int* board);

private:
    CEpuck* m_pcEpuck;
    
	CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
	CRealLightSensor* m_seLight;
	CRealBlueLightSensor* m_seBlueLight;
	CRealRedLightSensor* m_seRedLight;
	CContactSensor* m_seContact;
	CGroundSensor* m_seGround;
	CGroundMemorySensor* m_seGroundMemory;
	CBatterySensor* m_seBattery;  
	CBlueBatterySensor* m_seBlueBattery;  
	CRedBatterySensor* m_seRedBattery;  
	CEncoderSensor* m_seEncoder;  
	CCompassSensor* m_seCompass;  

    float m_fOrientation; 
    dVector2 m_vPosition;

    /* Global Variables */
	double 		m_fLeftSpeed;
	double 		m_fRightSpeed;
	double**	m_fActivationTable;
	double 		m_fTime;
	int 		m_robotIndex;
	int 		robotAmount;
	int*		assignedLights;
	int* 		collectionBoard;

	int m_nWriteToFile;

	void ExecuteBehaviors ( void );
	void Coordinator ( void );

	void AvoidObstacles ( unsigned int un_priority );
	void ChargeBattery ( unsigned int un_priority );
	void CollectResources ( unsigned int un_priority );
	void ReturnToBase ( unsigned int un_priority );
	void HelpPartner ( unsigned int un_priority );
};

#endif
