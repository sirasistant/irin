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
	struct _SimpleBase {
	    double centerX;
	    double centerY;
	    double radius;
	}; 
    typedef struct _SimpleBase SimpleBase;

    CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri3Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);
    void setRobotIndex(int index);
    void setRobotAmount(int amount);
    void setAssignedBases(int* assignedBases);
    void setCollectionBoard(int* board);
    void setBases(double* centerX,double* centerY,double* radius);
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
	int*		assignedBases;
	int* 		collectionBoard;
	double		m_lastGround;
	int 		m_ticksSinceScore;
	int 		m_nWriteToFile;
	int 		m_cargoBayLoad;
	SimpleBase*	m_bases;
	int		m_ticksTurning;
	int 		m_ticksSinceTurn;
	double 		m_turningAngle;

	void ExecuteBehaviors ( void );
	void Coordinator ( void );

	void AvoidObstacles ( unsigned int un_priority );
	void ChargeBattery ( unsigned int un_priority );
	void CollectResources ( unsigned int un_priority );
	void ReturnToBase ( unsigned int un_priority );
	void HelpPartner ( unsigned int un_priority );
	void readBaseNeeds (double* baseNeeds);
	double readCargoBaySensor();
	void dropPayload();
	int getBaseUnderRobot();
	double* readBaseLights(int baseNumber);
	const double* readBaseDirections(int baseNumber);
	int readBaseInputNumber(int baseNumber);
	double Distance(double dX0, double dY0, double dX1, double dY1);
};

#endif
