/*! 
 * \file CtrlStruct_gr5.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR5_H_
#define _CTRL_STRUCT_GR5_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <stdio.h>

NAMESPACE_INIT(ctrlGr5);

#define KP 1.53460309674929
#define KI 1.03251773221888
#define KPHI 0.026
#define RADIUS_BEACON 0.04
#define cot(a) 1/tan(a)
#define MAX_F 80
#define MAX_LIST_ELEMENTS 21

#define BLUE 0
#define RED 1
#define YELLOW 2
#define WHITE 3


typedef struct CtrlState
{
    double Rwheel                       ;
    double b                            ;
    
    double integralR                    ;//KI for right
    double integralL                    ;//KI for left
    double eR_Old                       ;//old error right wheel
    double eL_Old                       ;//old error left wheel
    double eR_New                       ;//new error right wheel
    double eL_New                       ;//new error left wheel
    double time                         ;// present time
    double timeOld                      ;//old time
    double deltaTime                    ;//new time - old time
    
    double pos_x_odoOld                 ;// robot position x
    double pos_y_odoOld                 ;// robot position y
    double pos_theta_odoOld             ;// robot rotation theta
    double pos_x_odo                    ;// robot position x
    double pos_y_odo                    ;// robot position y
    double pos_theta_odo                ;// robot rotation theta
	double pos_theta_odoNW              ;
	double pos_x_tri                    ;
	double pos_y_tri                    ;
	double pos_theta_tri                ;
	double alpha_beacon[3]              ;
	int    countDiscontinuity           ;
	double errPrevious[3][3]            ;
    
    bool   calibrationIsDone            ;
    bool   Xcalibration                 ;
    bool   Ycalibration                 ;
    bool   YstopCalibration             ;
    bool   XstopCalibration             ;
	bool   Tcalibration                 ;
    double omegaRrefCalibration         ;
    double omegaLrefCalibration         ;
    int    countCalibration             ;
	int    map[3]                       ;
	double err[3][3]                    ;
	bool   onTarget                     ;
    
	double omegaRref					;
	double omegaLref					;
	double omegaRrefMotor               ;
	double omegaLrefMotor               ;
    
	double errorR                       ;
	double errorL                       ;
	double errorRold                    ;
	double errorLold                    ;
	double intErrorR                    ;
	double intErrorL                    ;
	double dt                           ;
	double rho                          ;
    
    int RobotState                      ;
	int indexPath                       ;
	bool onNode                         ;
	double t0                           ;
	bool waitIsDone                     ;
	int indexWait                       ;
	int start                           ;
	int goal                            ;
	int indexStartNode                  ;
	int indexGoalNode                   ;
	int currentNode                     ;
	int cntBase                         ;
	int previousState                   ;
	
	//--------- Detect oponent ----------
	double Xoponents                    ;
	double Yoponents                    ;
	double XoponentsSum                 ;
	double YoponentsSum                 ;
	int filterCntOp                     ;
	double XoponentsFiltered            ;
	double YoponentsFiltered            ;
	double distoPponentold              ;
	double distoPponent                 ;
	int cntOpp                          ;
	double sumOpp                       ;
	
    //-----------------------------------
	
    FILE  *pFileSpeed, *pFileTime       ;
	FILE  *pFileXoponent,*pFileYoponent ;
	FILE  *pFileXopFiltered,*pFileYopFiltered;
	FILE  *pFileXodo,*pFileYodo,*pFileThetaOdo;
	FILE  *pFileXtri,*pFileYtri,*pFileThetaTri;
	FILE  *pFileX,*pFileY,*pFileTheta;
	FILE  *pFileDisF,*pFileDis;
	
	 //------------------Kalman---------------------------

#define BETWEEN_WHEELS 0.225
#define WHEEL_RADIUS 0.03 // b
#define KOR 0.07/10 // error of the actualtors
#define KOL 0.07/10
 //part 1 - odometer
  double P[3][3];
  double newP[3][3];
  double Fp_PFpt[3][3];
  double PFpt[3][3];
 double QFut[2][3];
  double Fu_QFut[3][3];
  //part 2 - kalman
  double PHt[3][3];
  double HPHt[3][3];
  double S[3][3];

  double invS[3][3];
  double K[3][3];
  double HtinvS[3][3];
  double error[3][1];

  double Kerror[3][1];
  double newX[3][1];
  double KH[3][3];
  double I_KH[3][3];

  //var

  double Odo_kal[3][1]                ;
  double triang_kal[3][1]             ;
//--------------------------------------------------------------------
	
}CtrlState;





/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
    CtrlState *states; ///< controller states
    CtrlOut *py_outputs; ///< python controller outputs
    
} CtrlStruct;

// function prototypes
void controller_iterate(CtrlStruct *cvs, double omegaRref, double omegaLref);
double limiter(double value, double min, double max);
void calibration(CtrlStruct *cvs);
void getOdometry(CtrlStruct *cvs);
void distanceOfOponents(CtrlStruct *cvs);
double angleOfOponents(CtrlStruct *cvs);
void triangulation(CtrlStruct *cvs);
void triangulationCompute(CtrlStruct *cvs);
void mappingAnglesBeacons(CtrlStruct *cvs);
double wrapAngle(double angle);
void navigation(CtrlStruct *cvs,double X_cible, double Y_cible);
void wait(CtrlStruct* cvs);
void XYoponents(CtrlStruct* cvs);

//--------------------------KALMAN------------------------------
void KalmanFilter(CtrlStruct *cvs);
//mathematical manipulation
void inv_mat_33(double result[3][3], double A[3][3]);
void sous_mat_33_33(double result[3][3], double A[3][3], double B[3][3]);
void sous_mat_31_31(double result[3][1], double A[3][1], double B[3][1]);
void add_mat_33_33(double result[3][3], double A[3][3], double B[3][3]);
void add_mat_31_31(double result[3][1], double A[3][1], double B[3][1]);
void mult_mat_22_23(double result[2][3], double A[2][2], double B[2][3]);
void mult_mat_32_23(double result[3][3], double A[3][2], double B[2][3]);
void mult_mat_33_33(double result[3][3], double A[3][3], double B[3][3]);
void mult_mat_33_31(double result[3][1], double A[3][3], double B[3][1]);
//------------------------------------------------------------------


//------------------------------ Astar -----------------------------------

typedef struct node{
    double x;
    double y;
    double f,g,h;
    int walkable;
    int open;
    int closed;
    int index;
    int adjacent[8];
    struct node *parent; // Adress of parent
}node;

typedef struct map{
    node *nodes;
    int  number;
    int *path;
    int lengthPath;
}map;

map* init_map();
int Astar(int start, int goal, map* map);
void freeMap(map* map);
double heuristique(node n, node goal);
void sort(double *list, int length);
void printPath(map *map);
double g(node a, node b);
void printMap();
int openIsEmpty(map* map);
int minIndexFopen(map* map);
void setParent(map* map, int n);
void printClosed(map* map);

//------------------------- Target functions -----------------------------
void target(CtrlStruct *cvs, map* Map, int start, int goal);

//------------------------------------------------------------------------

//-------------------------------------------------------------------------

CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
