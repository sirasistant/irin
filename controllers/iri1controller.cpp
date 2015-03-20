/******************* INCLUDES ******************/
/***********************************************/
#define PI 3.14159265
/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <algorithm>
#include "math.h"
#include <stdlib.h>
#include <cmath>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng* rng;
extern long int rngSeed;
double last_position[]={0.0,0.0};
double last_orientation=0.0;
double robotWidth=0.053;
double maxError[]={0.0,0.0};

using namespace std;

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

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
    m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
    /* Set Red light Sensor */
    m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
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
    /* Set encoder Sensor */
    m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
    m_seEncoder->InitEncoderSensor(m_pcEpuck);
    /* Set compass Sensor */
    m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);

}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


    /* FASE 1: LECTURA DE SENSORES */

    /* Leer Sensores de Contacto */
    double* contact = m_seContact->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Proximidad */
    double* prox = m_seProx->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Luz */
    double* light = m_seLight->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Luz Azul*/
    double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Luz Roja*/
    double* redlight = m_seRedLight->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Suelo */
    double* ground = m_seGround->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Suelo Memory */
    double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
    /* Leer Battery Sensores de Suelo Memory */
    double* battery = m_seBattery->GetSensorReading(m_pcEpuck);
    /* Leer Blue Battery Sensores de Suelo Memory */
    double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
    /* Leer Red Battery Sensores de Suelo Memory */
    double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);
    /* Leer Encoder */
    double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
    /* Leer Compass */
    double* compass = m_seCompass->GetSensorReading(m_pcEpuck);


    /* FASE 2: CONTROLADOR */

    /* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
    printf("Encoder Sensor Value: ");
    for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i++)
    {
        printf("%2f ", encoder[i]);
    }
    printf("\n");

    double dist1=encoder[0];
    double dist2=encoder[1];
    if(dist1==dist2){
        last_position[0]+=cos(last_orientation)*dist1;
        last_position[1]+=sin(last_orientation)*dist2;
        printf("Same distance on both enconders \n");
    }else{
        if(dist2>dist1){
            double angle=(dist2-dist1)/robotWidth;
            double distanceFromCenter=std::min(dist1,dist2)/angle+robotWidth/2;
            double incrementInLocalsX=+(cos(angle)*distanceFromCenter-distanceFromCenter);
            double incrementInLocalsY=sin(angle)*distanceFromCenter;
            double incrementInGlobalsX=cos(last_orientation)*incrementInLocalsY+cos(last_orientation-PI/2)*incrementInLocalsX;
            double incrementInGlobalsY=sin(last_orientation)*incrementInLocalsY+sin(last_orientation-PI/2)*incrementInLocalsX;
            last_position[0]+=incrementInGlobalsX;
            last_position[1]+=incrementInGlobalsY;
            last_orientation+=angle;
        }else{
            double angle=(dist1-dist2)/robotWidth;
            double distanceFromCenter=std::min(dist1,dist2)/angle+robotWidth/2;
            double incrementInLocalsX=-(cos(angle)*distanceFromCenter-distanceFromCenter);
            double incrementInLocalsY=sin(angle)*distanceFromCenter;
            double incrementInGlobalsX=cos(last_orientation)*incrementInLocalsY+cos(last_orientation-PI/2)*incrementInLocalsX;
            double incrementInGlobalsY=sin(last_orientation)*incrementInLocalsY+sin(last_orientation-PI/2)*incrementInLocalsX;
            last_position[0]+=incrementInGlobalsX;
            last_position[1]+=incrementInGlobalsY;
            last_orientation-=angle;
        }

    }

    double error[]={abs(last_position[0]-(m_pcEpuck->GetPosition()).x),abs(last_position[1]-(m_pcEpuck->GetPosition()).y)};
    if(error[0]>maxError[0]){
        maxError[0]=error[0];
    }
    if(error[1]>maxError[1]){
        maxError[1]=error[1];
    }

    printf("Calculated Robot Position: %2f,%2f\n",last_position[0],last_position[1]);
    printf("Robot Real Position: %2f,%2f\n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y);
    printf("Positioning error: %2f,%2f\n",error[0],error[1]);
    printf("Positioning maximum error: %2f,%2f\n",maxError[0],maxError[1]);

    /* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */


    FILE* filePosition = fopen("outputFiles/robotPosition", "a");
    fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
            f_time, m_pcEpuck->GetPosition().x,
            m_pcEpuck->GetPosition().y,
            m_pcEpuck->GetRotation());
    fclose(filePosition);


    /* Fase 3: ACTUACIÓN */
    /* Option 1: Speed between -1000, 1000*/
    m_acWheels->SetSpeed(-300,-500);

    /* Option 2: Speed between 0,1*/
    //m_acWheels->SetOutput(0,0.5);
    //m_acWheels->SetOutput(1,0.5);

}

/******************************************************************************/
/******************************************************************************/

