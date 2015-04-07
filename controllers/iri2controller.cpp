/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <queue>


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
#include "iri2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

const int mapGridX          = 20;
const int mapGridY          = 20;
double    mapLengthX        = 3.0;
double    mapLengthY        = 3.0;
int       robotStartGridX   = 10; 
int       robotStartGridY   = 10;

const   int n=mapGridX; // horizontal size of the map
const   int m=mapGridY; // vertical size size of the map

const   int dir=8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.02
#define PI 3.1415926535

using namespace std;

#define BEHAVIORS 5

#define AVOID_PRIORITY 0
#define CHARGE_PRIORITY 1
#define COLLECT_PRIORITY 2
#define GOAL_PRIORITY 3
#define RETURN_PRIORITY 4



#define PROXIMITY_THRESHOLD 0.3
#define BATTERY_THRESHOLD 0.5

#define SPEED 500.0
#define BASE_AMOUNT 2
#define SCORE_SPACING 40
#define MAX_CARGO_LOAD 3.0
#define MAX_TURN_SPACING 50
#define MAX_TURN_DURATION 10

#define NO_OBSTACLE 0
#define OBSTACLE    1
#define START       2
#define PATH        3
#define END         4
#define NEST        5
#define PREY        6
/******************************************************************************/
/******************************************************************************/

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

public:
    node(int xp, int yp, int d, int p)
    {xPos=xp; yPos=yp; level=d; priority=p;}

    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}

    void updatePriority(const int & xDest, const int & yDest)
    {
        priority=level+estimate(xDest, yDest)*10; //A*
    }

    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i) // i: direction
    {
        level+=(dir==8?(i%2==0?10:14):10);
    }

    // Estimation function for the remaining distance to the goal.
    const int & estimate(const int & xDest, const int & yDest) const
    {
        static int xd, yd, d;
        xd=xDest-xPos;
        yd=yDest-yPos;

        // Euclidian Distance
        d=static_cast<int>(sqrt(xd*xd+yd*yd));

        // Manhattan distance
        //d=abs(xd)+abs(yd);

        // Chebyshev distance
        //d=max(abs(xd), abs(yd));

        return(d);
    }
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator < ( const node & a, const node & b )
{
    return a.getPriority() > b.getPriority();
}


CIri2Controller::CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
    m_basesToCoordinates=new int*[BASE_AMOUNT];
    for(int i=0;i<BASE_AMOUNT;i++){
        m_basesToCoordinates[i]=new int[3];
        m_basesToCoordinates[i][2]=0; //Set all base discovery indicator to 0
    }
    map=new int*[n];
    for(int i=0;i<n;i++){
        map[i]=new int[m];
    }
    onlineMap=new int*[n];
    for(int i=0;i<n;i++){
        onlineMap[i]=new int[m];
    }
    closed_nodes_map=new int*[n];
    for(int i=0;i<n;i++){
        closed_nodes_map[i]=new int[m];
    }
    open_nodes_map=new int*[n];
    for(int i=0;i<n;i++){
        open_nodes_map[i]=new int[m];
    }
    dir_map=new int*[n];
    for(int i=0;i<n;i++){
        dir_map[i]=new int[m];
    }

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

    /* Odometry */
    m_nState              = 0;
    m_nPathPlanningStops  = 0;
    m_fOrientation        = 0.0;
    m_vPosition.x         = 0.0;
    m_vPosition.y         = 0.0;

    /* Set Actual Position to robot Start Grid */
    m_nRobotActualGridX = robotStartGridX;
    m_nRobotActualGridY = robotStartGridY;

    /* Init onlineMap */
    for ( int y = 0 ; y < m ; y++ )
        for ( int x = 0 ; x < n ; x++ )
            onlineMap[x][y] = OBSTACLE;

    /* DEBUG */
    PrintMap(onlineMap);
    /* DEBUG */

    /* Initialize status of foraging */
    m_nForageStatus = 0;

    /* Initialize Nest/Prey variables */
    m_nNestGridX  = 0;
    m_nNestGridY  = 0;
    m_nPreyGridX  = 0;
    m_nPreyGridY  = 0;
    m_nPreyFound  = 0;
    m_nNestFound  = 0;

    /* Initialize PAthPlanning Flag*/
    m_nPathPlanningDone = 0;


}

/******************************************************************************/
/******************************************************************************/

CIri2Controller::~CIri2Controller()
{
    for ( int i = 0 ; i < BEHAVIORS ; i++ )
    {
        delete [] m_fActivationTable;
    }
}

//Setter methods
void CIri2Controller::setRobotIndex(int index){
    m_robotIndex=index;
}

void CIri2Controller::setRobotAmount(int amount){
    printf("set amount to %i",amount);
    robotAmount=amount;
}

void CIri2Controller::setAssignedBases(int* bases){

    assignedBases=bases;
}

void CIri2Controller::setCollectionBoard(int* board){
    collectionBoard=board;
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
    m_fTime = f_time;
    /* Execute the levels of competence */
    ExecuteBehaviors();
    double* ground =m_seGround->GetSensorReading(m_pcEpuck);
    m_lastGround=ground[0];
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

void CIri2Controller::ExecuteBehaviors ( void )
{
    for ( int i = 0 ; i < BEHAVIORS ; i++ )
    {
        m_fActivationTable[i][2] = 0.0;
    }
    /* Set Leds to BLACK */
    m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);

    AvoidObstacles ( AVOID_PRIORITY );
    ChargeBattery ( CHARGE_PRIORITY );
     CollectResources ( COLLECT_PRIORITY );
    ComputeActualCell ( GOAL_PRIORITY );
    PathPlanning ( GOAL_PRIORITY );
    GoGoal ( GOAL_PRIORITY );
    ReturnToBase ( RETURN_PRIORITY );

}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::Coordinator ( void )
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

void CIri2Controller::AvoidObstacles ( unsigned int un_priority )
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

void CIri2Controller::CollectResources ( unsigned int un_priority )
{
    m_ticksSinceScore++;
    double* ground = m_seGround->GetSensorReading(m_pcEpuck);
    if(m_lastGround==1.0&&ground[0]==0.5){
        printf("detected resource area entering \n");
        if(m_ticksSinceScore>SCORE_SPACING){
            if(readCargoBaySensor()!=1.0){ //If cargo bay is not full
                printf("Storing cargo \n");
                m_ticksSinceScore=0;
                m_cargoBayLoad++;//Save resource to cargo bay
            }
        }
    }

    if(ground[0]==0.0){
        //detected base area entering
        dropPayload();
    }

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

void CIri2Controller::ReturnToBase ( unsigned int un_priority )
{
    int base=getObjectiveBase();


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
    m_fActivationTable[un_priority][1] = fMaxLight/8;
    if(readCargoBaySensor()>0.0){
        m_fActivationTable[un_priority][2] = 1.0;//TODO OJOCUIDAO OOJOJOJOJOJOJOJOJO
    }else{
        m_fActivationTable[un_priority][2] = 0.0;
    }
    if(base!=assignedBases[m_robotIndex]){
        m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);
    }
    if (m_nWriteToFile )
    {
        /* INIT WRITE TO FILE */
        FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
        fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, base!=assignedBases[m_robotIndex], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
        fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
        fclose(fileOutput);
        /* END WRITE TO FILE */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ChargeBattery ( unsigned int un_priority )
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
        fBattToForageInhibitor=0;
        /* Set Leds to RED */
        m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);

        /* Mark behavior as active */
        m_fActivationTable[un_priority][2] = 1.0;
    }else{
        fBattToForageInhibitor=1;
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

void CIri2Controller::readBaseNeeds (double* baseNeeds) {
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
double CIri2Controller::readCargoBaySensor(){
    return m_cargoBayLoad/MAX_CARGO_LOAD;
}

void CIri2Controller::dropPayload(){
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

void CIri2Controller::setBases(double* centerX,double* centerY,double* radius){
    m_bases=new SimpleBase[BASE_AMOUNT];
    for(int i=0;i<BASE_AMOUNT;i++){
        m_bases[i].centerX=centerX[i];
        m_bases[i].centerY=centerY[i];
        m_bases[i].radius=radius[i];
        printf(", simpleBase is:x: %2.2f, y: %2.2f, rad: %2.2f \n",m_bases[i].centerX,m_bases[i].centerY,m_bases[i].radius);
    }
}

int CIri2Controller::getBaseUnderRobot(){
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
double* CIri2Controller::readBaseLights(int baseNumber){
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
const double* CIri2Controller::readBaseDirections(int baseNumber){
    if(baseNumber==0){
        return m_seBlueLight->GetSensorDirections();
    }else{
        return m_seRedLight->GetSensorDirections();
    }
}

int CIri2Controller::readBaseInputNumber(int baseNumber){
    if(baseNumber==0){
        return m_seBlueLight->GetNumberOfInputs();
    }else{
        return m_seRedLight->GetNumberOfInputs();
    }
}

double CIri2Controller::Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ComputeActualCell ( unsigned int un_priority )
{
    /* Leer Encoder */
    double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
    /* Leer Sensores de Suelo Memory */
    double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
    double* ground=m_seGround->GetSensorReading(m_pcEpuck);

    CalcPositionAndOrientation (encoder);

    /* DEBUG */
    //printf("POS: X: %2f, %2f\r", m_vPosition.x, m_vPosition.y );
    /* DEBUG */

    /* Calc increment of position, correlating grid and metrics */
    double fXmov = mapLengthX/((double)mapGridX);
    double fYmov = mapLengthY/((double)mapGridY);

    /* Compute X grid */
    double tmp = m_vPosition.x;
    tmp += robotStartGridX * fXmov + 0.5*fXmov;
    m_nRobotActualGridX = (int) (tmp/fXmov);

    /* Compute Y grid */
    tmp = -m_vPosition.y;
    tmp += robotStartGridY * fYmov + 0.5*fYmov;
    m_nRobotActualGridY = (int) (tmp/fYmov);


    /* DEBUG */
    //printf("GRID: X: %d, %d\n", m_nRobotActualGridX, m_nRobotActualGridY);
    /* DEBUG */

    /* Update no-obstacles on map */
    if (  onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != NEST &&
          onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != PREY )
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NO_OBSTACLE;
    m_nForageStatus=readCargoBaySensor()>0?1:0;
    if(ground[0]==0&&m_lastGround!=0){
        printf("i'm in nest, should drop");
        identifyAndStoreNest(m_nRobotActualGridX,m_nRobotActualGridY);
    }
    if ( ground[0]==0.5&&m_lastGround!=0.5)
    {

         //update forage status
        // m_nForageStatus = 1;
         // Asumme Path Planning is done
         m_nPathPlanningDone = 0;
         // Restart PathPlanning state
         m_nState = 0;
        /* Mark prey on map */
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = PREY;
        /* Flag that nest was found */
        m_nPreyFound = 1;
        /* Update nest grid */
        m_nPreyGridX = m_nRobotActualGridX;
        m_nPreyGridY = m_nRobotActualGridY;
        /* DEBUG */
        PrintMap(onlineMap);
        /* DEBUG */
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::PathPlanning ( unsigned int un_priority )
{

    m_nNestFound=m_basesToCoordinates[getObjectiveBase()][2];

    /* Clear Map */
    for ( int y = 0 ; y < m ; y++ )
        for ( int x = 0 ; x < n ; x++ )
            map[x][y]= NO_OBSTACLE;
  //  printf("Trying to pathfind %i %i %i", m_nNestFound , m_nPreyFound , m_nPathPlanningDone );
    if ( m_nNestFound == 1 && m_nPreyFound == 1 && m_nPathPlanningDone == 0 && getObjectiveBase()>-1)
    {

           printf("Gonna pathfind \n");
        m_nPathPlanningStops=0;
        m_fActivationTable[un_priority][2] = 1;
        printf("base %i ",getObjectiveBase());

        /* Obtain start and end desired position */
        int xA, yA, xB, yB;
        if ( m_nForageStatus == 1)
        {
            xA=m_nRobotActualGridX;
            yA=m_nRobotActualGridY;
            xB=getObjectiveNestX();
            yB=getObjectiveNestY();
        }
        else
        {
            xA=m_nRobotActualGridX;
            yA=m_nRobotActualGridY;
            xB=m_nPreyGridX;
            yB=m_nPreyGridY;
        }

        /* DEBUG */
        printf("START: %d, %d - END: %d, %d\n", xA, yA, xB, yB);
        /* DEBUG */
        printf("hola1");
        /* Obtain Map */
        for ( int y = 0 ; y < m ; y++ )
            for ( int x = 0 ; x < n ; x++ )
                if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != NEST && onlineMap[x][y] != PREY)
                    map[x][y] = OBSTACLE;

        printf("hola2");
        /* Obtain optimal path */
        string route=pathFind(xA, yA, xB, yB);
        /* DEBUG */
        if(route=="") cout<<"An empty route generated!"<<endl;
        cout << "Route:" << route << endl;
        //     printf("route Length: %d\n", route.length());
        /* DEBUG */
        /* Obtain number of changing directions */
        for (int i = 1 ; i < route.length() ; i++)
            if (route[i-1] != route[i])
                m_nPathPlanningStops++;
        printf("hola3");
        /* Add last movement */
        m_nPathPlanningStops++;
        /* DEBUG */
        printf("STOPS: %d\n", m_nPathPlanningStops);
        /* DEBUG */
        printf("hola4");
        /* Define vector of desired positions. One for each changing direction */
        m_vPositionsPlanning = new dVector2[m_nPathPlanningStops];

        /* Calc increment of position, correlating grid and metrics */
        double fXmov = mapLengthX/mapGridX;
        double fYmov = mapLengthY/mapGridY;
        /* Get actual position */
        dVector2 actualPos;
        //actualPos.x = robotStartGridX * fXmov;
        actualPos.x = m_nRobotActualGridX * fXmov;
        //actualPos.y = robotStartGridY * fYmov;
        actualPos.y = m_nRobotActualGridY * fYmov;
        /* Fill vector of desired positions */
        int stop = 0;
        int counter = 0;
        /* Check the route and obtain the positions*/
        for (int i = 1 ; i < route.length() ; i++)
        {
            /* For every position in route, increment countr */
            counter++;
            /* If a direction changed */
            if ((route[i-1] != route[i]))
            {
                /* Obtain the direction char */
                char c;
                c = route.at(i-1);

                /* Calc the new stop according to actual position and increment based on the grid */
                m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];
                m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];

                /* Update position for next stop */
                actualPos.x = m_vPositionsPlanning[stop].x;
                actualPos.y = m_vPositionsPlanning[stop].y;

                /* Increment stop */
                stop++;
                /* reset counter */
                counter = 0;
            }
            /* If we are in the last update, calc last movement */
            if (i==(route.length()-1))
            {
                /* Increment counter */
                counter++;
                /* Obtain the direction char */
                char c;
                c = route.at(i);

                /* DEBUG */
                //printf("COUNTER: %d, CHAR: %c\n", counter, c);
                /* END DEBUG */

                /* Calc the new stop according to actual position and increment based on the grid */
                m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];// - robotStartGridX * fXmov;
                m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];// - robotStartGridY * fYmov;

                /* Update position for next stop */
                actualPos.x = m_vPositionsPlanning[stop].x;
                actualPos.y = m_vPositionsPlanning[stop].y;

                /* Increment stop */
                stop++;
                /* reset counter */
                counter = 0;
            }

        }
        /* DEBUG */
        if(route.length()>0)
        {
            int j; char c;
            int x=xA;
            int y=yA;
            map[x][y]=START;
            for ( int i = 0 ; i < route.length() ; i++ )
            {
                c = route.at(i);
                j = atoi(&c);
                x = x+dx[j];
                y = y+dy[j];
                map[x][y] = PATH;
            }
            map[x][y]=END;
            printf("I'm from base %i",assignedBases[m_robotIndex]);
            PrintMap(onlineMap);
            printf("\n\n");
            PrintMap(map);
        }
        /* END DEBUG */

        /* DEBUG */
        //printf("Start: %2f, %2f\n", m_nRobotActualGridX * fXmov, m_nRobotActualGridY * fXmov);
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */

        /* Convert to simulator coordinates */
        for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        {
            m_vPositionsPlanning[i].x -= (mapGridX * fXmov)/2;
            m_vPositionsPlanning[i].y -= (mapGridY * fYmov)/2;
            m_vPositionsPlanning[i].y = - m_vPositionsPlanning[i].y;
        }
        /* DEBUG */
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */


        /* Convert to robot coordinates. FAKE!!.
     * Notice we are only working with initial orientation = 0.0 */
        for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        {
            /* Traslation */
            m_vPositionsPlanning[i].x += ( (robotStartGridX * fXmov) - (mapGridX * fXmov)/2 );
            m_vPositionsPlanning[i].y += ( (robotStartGridY * fXmov) - (mapGridY * fYmov)/2);
            /* Rotation */
            //double compass = m_pcEpuck->GetRotation();
            //m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
            //m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
        }
        /* DEBUG */
        //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
        //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
        /* END DEBUG */

        m_nPathPlanningDone = 1;
    }
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::GoGoal ( unsigned int un_priority )
{

    if ( ((m_nNestFound * fBattToForageInhibitor) == 1 ) && ( (m_nPreyFound *  fBattToForageInhibitor )== 1 ) )
    {
     //   printf("Going goal \n");
        /* Enable Inhibitor to Forage */
        fGoalToForageInhibitor = 0.0;

        /* If something not found at the end of planning, reset plans */
        if (m_nState >= m_nPathPlanningStops )
        {
          //  m_nNestFound  = 0;
          //  m_nNestFound  = 0;
            m_nState      = 0;
            return;
        }

        /* DEBUG */
       // printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x );
      //  printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y );
        /* DEBUG */

        double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
        double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);
        double fGoalDirection = 0;

        /* If on Goal, return 1 */
        if ( ( fabs(fX) <= ERROR_POSITION ) && ( fabs(fY) <= ERROR_POSITION ) )
            m_nState++;

        fGoalDirection = atan2(fY, fX);

        /* Translate fGoalDirection into local coordinates */
        fGoalDirection -= m_fOrientation;
        /* Normalize Direction */
        while ( fGoalDirection > M_PI) fGoalDirection -= 2 * M_PI;
        while ( fGoalDirection < -M_PI) fGoalDirection+=2*M_PI;

        m_fActivationTable[un_priority][0] = fGoalDirection;
        m_fActivationTable[un_priority][1] = 1;
        m_fActivationTable[un_priority][2] = 1;
    }
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::CalcPositionAndOrientation (double *f_encoder)
{
    double dist1=f_encoder[0];
    double dist2=f_encoder[1];
    double robotWidth = CEpuck::WHEELS_DISTANCE;
    if(dist1==dist2)
    {
        m_vPosition.x += cos(m_fOrientation)*dist1;
        m_vPosition.y += sin(m_fOrientation)*dist2;
    }
    else
    {
        if(dist2>dist1)
        {
            double angle=(dist2-dist1)/robotWidth;
            double distanceFromCenter=std::min(dist1,dist2)/angle+robotWidth/2;
            double incrementInLocalsX=+(cos(angle)*distanceFromCenter-distanceFromCenter);
            double incrementInLocalsY=sin(angle)*distanceFromCenter;
            double incrementInGlobalsX=cos(m_fOrientation)*incrementInLocalsY+cos(m_fOrientation-PI/2)*incrementInLocalsX;
            double incrementInGlobalsY=sin(m_fOrientation)*incrementInLocalsY+sin(m_fOrientation-PI/2)*incrementInLocalsX;
            m_vPosition.x += incrementInGlobalsX;
            m_vPosition.y += incrementInGlobalsY;
            m_fOrientation += angle;
        }
        else
        {
            double angle=(dist1-dist2)/robotWidth;
            double distanceFromCenter=std::min(dist1,dist2)/angle+robotWidth/2;
            double incrementInLocalsX=-(cos(angle)*distanceFromCenter-distanceFromCenter);
            double incrementInLocalsY=sin(angle)*distanceFromCenter;
            double incrementInGlobalsX=cos(m_fOrientation)*incrementInLocalsY+cos(m_fOrientation-PI/2)*incrementInLocalsX;
            double incrementInGlobalsY=sin(m_fOrientation)*incrementInLocalsY+sin(m_fOrientation-PI/2)*incrementInLocalsX;
            m_vPosition.x += incrementInGlobalsX;
            m_vPosition.y += incrementInGlobalsY;
            m_fOrientation -= angle;
        }
    }
    while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
    while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri2Controller::pathFind( int xStart, int yStart,
                                  int xFinish, int yFinish )
{
     priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
     int pqi; // pq index
     node* n0;
     node* m0;
     int i, j, x, y, xdx, ydy;
     char c;
    pqi=0;

    // reset the node maps
    for ( y=0 ; y < m ; y++ )
    {
        for ( x = 0 ; x < n ; x++ )
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
   // open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for ( i = 0 ; i < dir ; i++ )
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1
                 || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                            pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::PrintMap ( int** print_map )
{

    /* DEBUG */
       for ( int y = 0 ; y < m ; y++ )

    {
         for ( int x = 0 ; x < n ; x++ )
        {
            if ( print_map[x][y] == 0 )
                cout<<".";
            else if(print_map[x][y]==1)
                cout<<"O"; //obstacle
            else if(print_map[x][y]==2)
                cout<<"S"; //start
            else if(print_map[x][y]==3)
                cout<<"R"; //route
            else if(print_map[x][y]==4)
                cout<<"F"; //finish
            else if(print_map[x][y]==5)
                cout<<"N"; //finish
            else if(print_map[x][y]==6)
                cout<<"P"; //finish
        }
        cout<<endl;
    }
    /* DEBUG */
}
int CIri2Controller::getObjectiveNestX(){
    printf("Second call");
    int base=getObjectiveBase();
    printf("objective nest x");
    printf("is %i \n",m_basesToCoordinates[base][0]);
    return m_basesToCoordinates[base][0];

}
int CIri2Controller::getObjectiveNestY(){
        printf("third call");
    int base=getObjectiveBase();
    printf("objective nest y");
    printf("is %i \n",m_basesToCoordinates[base][1]);
    return m_basesToCoordinates[base][1];
}
void CIri2Controller::identifyAndStoreNest(int x, int y){
   // Asumme Path Planning is done
   m_nPathPlanningDone = 0;
   // Restart PathPlanning state
   m_nState = 0;
   /* Mark nest on map */
   onlineMap[x][y] = NEST;
   /* DEBUG */
    PrintMap(onlineMap);
    int nearestBase=-1;
    double nearestBaseLightAmount=-1;
    for(int i=0;i<BASE_AMOUNT;i++){
        double baseLightAmount=0;
        double* lectures=readBaseLights(i);
        for(int j=0;j<readBaseInputNumber(i);j++){
            baseLightAmount+=lectures[j];
        }
        if(baseLightAmount>nearestBaseLightAmount){
            nearestBase=i;
            nearestBaseLightAmount=baseLightAmount;
        }
    }

    if(nearestBase>-1&&m_basesToCoordinates[nearestBase][2]==0){
        printf("Gonna identify base, ");
        printf(" identified base is: %i with x: %i and y %i \n",nearestBase,x,y);
        m_basesToCoordinates[nearestBase][0]=x;
        m_basesToCoordinates[nearestBase][1]=y;
        m_basesToCoordinates[nearestBase][2]=1;//base has been discovered
    }
}

int CIri2Controller::getObjectiveBase(){
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
    return base;
}


