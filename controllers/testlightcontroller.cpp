
/******************* INCLUDES ******************/
/***********************************************/
#define PI 3.14159265
/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include "math.h"

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "reallightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testlightcontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestLightController::CTestLightController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
}

/******************************************************************************/
/******************************************************************************/

CTestLightController::~CTestLightController()
{
}


/******************************************************************************/
/******************************************************************************/

double getGaussianValue(double x,double center,double width){
    return exp(-1*pow(x-center,2)/(2*pow(width,2)))/(2*width*sqrt(2*PI));
}

double getTriangle(double x,double center){
    if(x<center){
        return (1/center)*x;
    }else{
        return 1-(1/(1-center))*(x-center);
    }
}

double getInverseTriangle(double x,double center){
    if(x<center){
        return 1-(1/center)*x;
    }else{
        return (1/(1-center))*(x-center);
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void CTestLightController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
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

