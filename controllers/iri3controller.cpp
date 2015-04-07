/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>


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
#include "iri3controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

#define BEHAVIORS 4

#define AVOID_PRIORITY 0
#define CHARGE_PRIORITY 1
#define RETURN_PRIORITY 2
#define COLLECT_PRIORITY 3

#define PROXIMITY_THRESHOLD 0.3
#define BATTERY_THRESHOLD 0.5

#define SPEED 500.0
#define BASE_AMOUNT 2
#define SCORE_SPACING 40
#define MAX_CARGO_LOAD 3.0
#define MAX_TURN_SPACING 50
#define MAX_TURN_DURATION 10


CIri3Controller::CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
    m_ticksSinceTurn=0;
    m_ticksTurning=0;
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

    /* Initilize Variables */
    m_fLeftSpeed = 0.0;
    m_fRightSpeed = 0.0;


    /* Create TABLE for the COORDINATOR */
    m_fActivationTable = new double* [BEHAVIORS];
    for ( int i = 0 ; i < BEHAVIORS ; i++ )
    {
        m_fActivationTable[i] = new double[3];
    }

    for(int i = 0; i < BEHAVIORS; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m_fActivationTable[i][j] = 0.0;
        }
    }



}

/******************************************************************************/
/******************************************************************************/

CIri3Controller::~CIri3Controller()
{
    for ( int i = 0 ; i < BEHAVIORS ; i++ )
    {
        delete [] m_fActivationTable;
    }
}

//Setter methods
void CIri3Controller::setRobotIndex(int index){
    m_robotIndex=index;
}

void CIri3Controller::setRobotAmount(int amount){
    printf("set amount to %i",amount);
    robotAmount=amount;
}

void CIri3Controller::setAssignedBases(int* bases){
    assignedBases=bases;
}

void CIri3Controller::setCollectionBoard(int* board){
    collectionBoard=board;
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
    m_fTime = f_time;

    /* Execute the levels of competence */
    ExecuteBehaviors();

    /* Execute Coordinator */
    Coordinator();


    /* Set Speed to wheels */
    m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

    if (m_nWriteToFile )
    {
        /* INIT: WRITE TO FILES */
        /* Write robot position and orientation */
        FILE* filePosition = fopen("outputFiles/robotPosition", "a");
        fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
        fclose(filePosition);

        /* Write robot wheels speed */
        FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
        fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
        fclose(fileWheels);
        /* END WRITE TO FILES */
    }

}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::ExecuteBehaviors ( void )
{
    for ( int i = 0 ; i < BEHAVIORS ; i++ )
    {
        m_fActivationTable[i][2] = 0.0;
    }
    /* Set Leds to BLACK */
    m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);

    AvoidObstacles ( AVOID_PRIORITY );
    ChargeBattery ( CHARGE_PRIORITY );
    ReturnToBase ( RETURN_PRIORITY );
    CollectResources ( COLLECT_PRIORITY );

}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::Coordinator ( void )
{
    int nBehavior;
    double fAngle = 0.0;
    double behaviorX = 0.0;
    double behaviorY = 0.0;
    double currentX = 0.0;
    double currentY = 0.0;

    int nActiveBehaviors = 0;
    /* For every Behavior Activated, sum angles */
    for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
    {
        if ( m_fActivationTable[nBehavior][2] == 1.0 )
        {
            behaviorX = m_fActivationTable[nBehavior][1]*cos(m_fActivationTable[nBehavior][0]);
            behaviorY = m_fActivationTable[nBehavior][1]*sin(m_fActivationTable[nBehavior][0]);
            currentX += behaviorX;
            currentY += behaviorY;
            nActiveBehaviors++;
        }
    }

    fAngle = atan2(currentY, currentX);

    /* Normalize fAngle */
    while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
    while ( fAngle < -M_PI ) fAngle += 2 * M_PI;

    /* Based on the angle, calc wheels movements */
    double fCLinear = 1.0;
    double fCAngular = 1.0;
    double fC1 = SPEED / M_PI;

    /* Calc Linear Speed */
    double fVLinear = SPEED * fCLinear * ( cos ( fAngle / 2) );

    /*Calc Angular Speed */
    double fVAngular = fAngle;

    m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
    m_fRightSpeed = fVLinear + fC1 * fVAngular;

    if (m_nWriteToFile )
    {
        /* INIT: WRITE TO FILES */
        /* Write coordinator ouputs */
        FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
        fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
        fclose(fileOutput);
        /* END WRITE TO FILES */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::AvoidObstacles ( unsigned int un_priority )
{
    /* Leer Sensores de Proximidad */
    double* prox = m_seProx->GetSensorReading(m_pcEpuck);

    double fMaxProx = 0.0;
    const double* proxDirections = m_seProx->GetSensorDirections();

    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    /* Calc vector Sum */
    for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
    {
        vRepelent.x += prox[i] * cos ( proxDirections[i] );
        vRepelent.y += prox[i] * sin ( proxDirections[i] );

        if ( prox[i] > fMaxProx )
            fMaxProx = prox[i];
    }

    /* Calc pointing angle */
    float fRepelent = atan2(vRepelent.y, vRepelent.x);
    /* Create repelent angle */
    fRepelent -= M_PI;
    /* Normalize angle */
    while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
    while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fRepelent;
    m_fActivationTable[un_priority][1] = fMaxProx;

    /* If above a threshold */
    if ( fMaxProx > PROXIMITY_THRESHOLD )
    {
        /* Set Leds to GREEN */
        m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
        /* Mark Behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
    }

    if (m_nWriteToFile )
    {
        /* INIT WRITE TO FILE */
        /* Write level of competence ouputs */
        FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::CollectResources ( unsigned int un_priority )
{
    m_ticksSinceScore++;
    double* ground = m_seGround->GetSensorReading(m_pcEpuck);
    if(m_lastGround==1.0&&ground[0]==0.0){
        printf("detected resource area entering \n");
        if(m_ticksSinceScore>SCORE_SPACING){
            if(readCargoBaySensor()!=1.0){ //If cargo bay is not full
                printf("Storing cargo \n");
                m_ticksSinceScore=0;
                m_cargoBayLoad++;//Save resource to cargo bay
            }
        }
    }

    if(ground[0]==0.5){
        //detected base area entering
        dropPayload();
    }
    m_lastGround=ground[0];
    /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */

    if(m_ticksTurning>0){

        m_ticksTurning++;
        if(m_ticksTurning>=MAX_TURN_DURATION){
            m_turningAngle=0;
            m_ticksTurning=0;
            m_ticksSinceTurn=0;
        }
    }else{

        m_ticksSinceTurn++;
        if(m_ticksSinceTurn>=MAX_TURN_SPACING){
            m_turningAngle=(M_PI/2.0)*((rand() % 100)/99.0-0.5);
            m_ticksTurning=1;
            m_ticksSinceTurn=0;
        }
    }
    m_fActivationTable[un_priority][0] = m_turningAngle;
    m_fActivationTable[un_priority][1] = 0.15;
    m_fActivationTable[un_priority][2] = 1.0;

    if (m_nWriteToFile )
    {
        /* INIT: WRITE TO FILES */
        /* Write level of competence ouputs */
        FILE* fileOutput = fopen("outputFiles/resourcesOutput", "a");
        fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILES */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::ReturnToBase ( unsigned int un_priority )
{
    int base=assignedBases[m_robotIndex];
    bool helpingFriend=false;
    double* baseNeeds=new double[BASE_AMOUNT];
    readBaseNeeds(baseNeeds);
    double myBaseRank=baseNeeds[base];
    for(int i=0;i<BASE_AMOUNT;i++){
        if(base!=i){
            if(baseNeeds[i]-myBaseRank<-0.3){

                base=i;
                helpingFriend=true;
                break;
            }
        }
    }


    /* Leer Sensores de Luz */
    double* light = readBaseLights(base);

    double fMaxLight = 0.0;
    const double* lightDirections = readBaseDirections(base);

    /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    /* Calc vector Sum */
    for ( int i = 0 ; i < readBaseInputNumber(base) ; i ++ )
    {
        vRepelent.x += light[i] * cos ( lightDirections[i] );
        vRepelent.y += light[i] * sin ( lightDirections[i] );

        if ( light[i] > fMaxLight )
            fMaxLight = light[i];
    }

    /* Calc pointing angle */
    float fRepelent = atan2(vRepelent.y, vRepelent.x);

    /* Normalize angle */
    while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
    while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


    m_fActivationTable[un_priority][0] = fRepelent;
    m_fActivationTable[un_priority][1] = fMaxLight;
    if(readCargoBaySensor()>0.0){
        m_fActivationTable[un_priority][2] = 1.0;
    }else{
        m_fActivationTable[un_priority][2] = 0.0;
    }
    if(helpingFriend){
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);
    }
    if (m_nWriteToFile )
    {
        /* INIT WRITE TO FILE */
        FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, helpingFriend, light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri3Controller::ChargeBattery ( unsigned int un_priority )
{
    /* Leer Battery Sensores */
    double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

    /* Leer Sensores de Luz */
    double* light = m_seLight->GetSensorReading(m_pcEpuck);

    double fMaxLight = 0.0;
    const double* lightDirections = m_seLight->GetSensorDirections();

    /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
    dVector2 vRepelent;
    vRepelent.x = 0.0;
    vRepelent.y = 0.0;

    /* Calc vector Sum */
    for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
    {
        vRepelent.x += light[i] * cos ( lightDirections[i] );
        vRepelent.y += light[i] * sin ( lightDirections[i] );

        if ( light[i] > fMaxLight )
            fMaxLight = light[i];
    }

    /* Calc pointing angle */
    float fRepelent = atan2(vRepelent.y, vRepelent.x);

    /* Normalize angle */
    while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
    while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


    m_fActivationTable[un_priority][0] = fRepelent;
    m_fActivationTable[un_priority][1] = fMaxLight/2;

    /* If battery below a BATTERY_THRESHOLD */
    if ( battery[0] < BATTERY_THRESHOLD )
    {
        /* Set Leds to RED */
        m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);

        /* Mark behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
    }


    if (m_nWriteToFile )
    {
        /* INIT WRITE TO FILE */
        FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }
}


/*
 *This method takes an array of size equal to BASE_AMOUNT
 * and assigns to it a normalized collection leaderboard,
 * where the base that has collected the most is 1
 */

void CIri3Controller::readBaseNeeds (double* baseNeeds) {
    double maxCollection=0;
    for(int i=0;i<BASE_AMOUNT;i++){
        baseNeeds[i]=collectionBoard[i];
        if(baseNeeds[i]>maxCollection){
            maxCollection=baseNeeds[i];
        }
    }
    //normalize array
    for(int i=0;i<BASE_AMOUNT;i++){
        baseNeeds[i]/=maxCollection;
    }
}

/*
 * Reads the virtual cargo bay sensor and returns a normalized
 * load indicator where 1.0 is max and 0.0 is empty
 *
*/
double CIri3Controller::readCargoBaySensor(){
    return m_cargoBayLoad/MAX_CARGO_LOAD;
}

void CIri3Controller::dropPayload(){
    //find out in which base is standing (non robot work, scenario related)
    int base=getBaseUnderRobot();
    if(base!=-1){
        //drop the load
        if(m_cargoBayLoad>0){
            printf("Robot number %i dropping load to base %i . Dropped %i loads \n",m_robotIndex,base,m_cargoBayLoad);
            collectionBoard[base]+= m_cargoBayLoad;
            m_cargoBayLoad =0 ;
        }
    }
}

void CIri3Controller::setBases(double* centerX,double* centerY,double* radius){
    m_bases=new SimpleBase[BASE_AMOUNT];
    for(int i=0;i<BASE_AMOUNT;i++){
        m_bases[i].centerX=centerX[i];
        m_bases[i].centerY=centerY[i];
        m_bases[i].radius=radius[i];
        printf(", simpleBase is:x: %2.2f, y: %2.2f, rad: %2.2f \n",m_bases[i].centerX,m_bases[i].centerY,m_bases[i].radius);
    }
}

int CIri3Controller::getBaseUnderRobot(){
    for(int i=0;i<BASE_AMOUNT;i++){
        double distance=Distance(m_bases[i].centerX,m_bases[i].centerY,m_pcEpuck->GetPosition().x,m_pcEpuck->GetPosition().y);
        double radius=m_bases[i].radius;
        if(abs(distance)<abs(radius)){
            return i;
        }
    }
    return -1;

}
/*
 * This method assigns a basenumber to a light, if paramfile layout is modified, this method
 * should be modified accordingly, since basenumbers are an integer that represents
 * the position of each base in the paramfile
 *
*/
double* CIri3Controller::readBaseLights(int baseNumber){
    if(baseNumber==0){
        return m_seBlueLight->GetSensorReading(m_pcEpuck);
    }else{
        return m_seRedLight->GetSensorReading(m_pcEpuck);
    }
}

/*
 * This method assigns a basenumber to a light direction, if paramfile layout is modified, this method
 * should be modified accordingly, since basenumbers are an integer that represents
 * the position of each base in the paramfile
 *
*/
const double* CIri3Controller::readBaseDirections(int baseNumber){
    if(baseNumber==0){
        return m_seBlueLight->GetSensorDirections();
    }else{
        return m_seRedLight->GetSensorDirections();
    }
}

int CIri3Controller::readBaseInputNumber(int baseNumber){
    if(baseNumber==0){
        return m_seBlueLight->GetNumberOfInputs();
    }else{
        return m_seRedLight->GetNumberOfInputs();
    }
}

double CIri3Controller::Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}




